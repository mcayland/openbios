/*
 * OpenBIOS Legacy Virtio driver
 *
 * Copyright (c) 2013 Alexander Graf <agraf@suse.de>
 * Copyright (c) 2016 Mark Cave-Ayland <mark.cave-ayland@ilande.co.uk>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or (at
 * your option) any later version. See the COPYING file in the top-level
 * directory.
 */

#include "config.h"
#include "libc/byteorder.h"
#include "libc/vsprintf.h"
#include "libopenbios/bindings.h"
#include "libopenbios/ofmem.h"
#include "kernel/kernel.h"
#include "drivers/drivers.h"

#include "virtio.h"

#define VRING_WAIT_REPLY_TIMEOUT 10000


static uint8_t virtio_cfg_read8(VDev *vdev, int addr)
{
    return inb((uint16_t)(vdev->io_base + addr));
}

static void virtio_cfg_write8(VDev *vdev, int addr, uint8_t value)
{
    outb(value, vdev->io_base + addr);
}

static uint16_t virtio_cfg_read16(VDev *vdev, int addr)
{
    return inw(vdev->io_base + addr);
}

static void virtio_cfg_write16(VDev *vdev, int addr, uint16_t value)
{
    outw(value, vdev->io_base + addr);
}

static uint32_t virtio_cfg_read32(VDev *vdev, int addr)
{
    return inl(vdev->io_base + addr);
}

static void virtio_cfg_write32(VDev *vdev, int addr, uint32_t value)
{
    outl(value, vdev->io_base + addr);
}

static uint64_t virtio_cfg_read64(VDev *vdev, int addr)
{
    uint64_t q;
    uint8_t *p;
    int i;
    
    for (i = 0, p = (uint8_t *)&q; i < 8; i++, p++) {
        *p = virtio_cfg_read8(vdev, addr + i);
    }
    
    return q;
}

static long virtio_notify(VDev *vdev, int vq_idx, long cookie)
{    
    virtio_cfg_write16(vdev, VIRTIO_PCI_QUEUE_NOTIFY, vq_idx);
    
    return 0;
}

/***********************************************
 *             Virtio functions                *
 ***********************************************/

static void vring_init(VRing *vr, VqInfo *info)
{
    void *p = (void *) (uintptr_t)info->queue;

    vr->id = info->index;
    vr->num = info->num;
    vr->desc = p;
    vr->avail = (void *)((uintptr_t)p + info->num * sizeof(VRingDesc));
    vr->used = (void *)(((unsigned long)&vr->avail->ring[info->num]
               + info->align - 1) & ~(info->align - 1));

    /* Zero out all relevant field */
    vr->avail->flags = 0;
    vr->avail->idx = 0;

    /* We're running with interrupts off anyways, so don't bother */
    vr->used->flags = VRING_USED_F_NO_NOTIFY;
    vr->used->idx = 0;
    vr->used_idx = 0;
    vr->next_idx = 0;
    vr->cookie = 0;
}

static int vring_notify(VDev *vdev, VRing *vr)
{
    return virtio_notify(vdev, vr->id, vr->cookie);
}

static void vring_send_buf(VRing *vr, void *p, int len, int flags)
{
    ucell mode;
    
    /* For follow-up chains we need to keep the first entry point */
    if (!(flags & VRING_HIDDEN_IS_CHAIN)) {
        vr->avail->ring[vr->avail->idx % vr->num] = vr->next_idx;
    }

    vr->desc[vr->next_idx].addr = ofmem_translate((ucell)p, &mode);
    vr->desc[vr->next_idx].len = len;
    vr->desc[vr->next_idx].flags = flags & ~VRING_HIDDEN_IS_CHAIN;
    vr->desc[vr->next_idx].next = vr->next_idx;
    vr->desc[vr->next_idx].next++;
    vr->next_idx++;

    /* Chains only have a single ID */
    if (!(flags & VRING_DESC_F_NEXT)) {
        vr->avail->idx++;
    }
}

static int vr_poll(VDev *vdev, VRing *vr)
{
    if (vr->used->idx == vr->used_idx) {
        vring_notify(vdev, vr);
        return 0;
    }

    vr->used_idx = vr->used->idx;
    vr->next_idx = 0;
    vr->desc[0].len = 0;
    vr->desc[0].flags = 0;
    return 1; /* vr has been updated */
}

/*
 * Wait for the host to reply.
 *
 * timeout is in msecs if > 0.
 *
 * Returns 0 on success, 1 on timeout.
 */
static int vring_wait_reply(VDev *vdev)
{
    ucell target_ms, get_ms;
    
    fword("get-msecs");
    target_ms = POP();
    target_ms += vdev->wait_reply_timeout;

    /* Wait for any queue to be updated by the host */
    do {
        int i, r = 0;

        for (i = 0; i < vdev->nr_vqs; i++) {
            r += vr_poll(vdev, &vdev->vrings[i]);
        }
        
        if (r) {
            return 0;
        }
        
        fword("get-msecs");
        get_ms = POP();

    } while (!vdev->wait_reply_timeout || (get_ms < target_ms));

    return 1;
}

/***********************************************
 *               Virtio block                  *
 ***********************************************/

static int virtio_blk_read_many(VDev *vdev,
                                uint64_t offset, void *load_addr, int len)
{
    VirtioBlkOuthdr out_hdr;
    u8 status;
    VRing *vr = &vdev->vrings[vdev->cmd_vr_idx];
    uint8_t discard[VIRTIO_SECTOR_SIZE];
    
    uint64_t start_sector = offset / virtio_get_block_size(vdev);
    int head_len = offset & (virtio_get_block_size(vdev) - 1);
    uint64_t end_sector = (offset + len + virtio_get_block_size(vdev) - 1) /
                            virtio_get_block_size(vdev);   
    int tail_len = end_sector * virtio_get_block_size(vdev) - (offset + len);
    
    /* Tell the host we want to read */
    out_hdr.type = VIRTIO_BLK_T_IN;
    out_hdr.ioprio = 99;
    out_hdr.sector = virtio_sector_adjust(vdev, start_sector);

    vring_send_buf(vr, &out_hdr, sizeof(out_hdr), VRING_DESC_F_NEXT);

    /* Discarded head */
    if (head_len) {
        vring_send_buf(vr, discard, head_len,
                       VRING_DESC_F_WRITE | VRING_HIDDEN_IS_CHAIN |
                       VRING_DESC_F_NEXT);    
    }

    /* This is where we want to receive data */
    vring_send_buf(vr, load_addr, len,
                   VRING_DESC_F_WRITE | VRING_HIDDEN_IS_CHAIN |
                   VRING_DESC_F_NEXT);

    /* Discarded tail */
    if (tail_len) {
        vring_send_buf(vr, discard, tail_len,
                       VRING_DESC_F_WRITE | VRING_HIDDEN_IS_CHAIN |
                       VRING_DESC_F_NEXT);
    }
    
    /* status field */
    vring_send_buf(vr, &status, sizeof(u8),
                   VRING_DESC_F_WRITE | VRING_HIDDEN_IS_CHAIN);

    /* Now we can tell the host to read */
    vring_wait_reply(vdev);

    return status;
}

int virtio_read_many(VDev *vdev, uint64_t offset, void *load_addr, int len)
{
    switch (vdev->senseid) {
    case VIRTIO_ID_BLOCK:
        return virtio_blk_read_many(vdev, offset, load_addr, len);
    }
    return -1;
}

static int virtio_read(VDev *vdev, uint64_t offset, void *load_addr, int len)
{
    return virtio_read_many(vdev, offset, load_addr, len);
}

int virtio_get_block_size(VDev *vdev)
{
    switch (vdev->senseid) {
    case VIRTIO_ID_BLOCK:
        return vdev->config.blk.blk_size << vdev->config.blk.physical_block_exp;
    }
    return 0;
}

static void
ob_virtio_open(VDev **_vdev)
{
    VDev *vdev = *_vdev;
    phandle_t ph;

    vdev->pos = 0;

    /* interpose disk-label */
    ph = find_dev("/packages/disk-label");
    fword("my-args");
    PUSH_ph( ph );
    fword("interpose");

    RET(-1);
}

static void
ob_virtio_close(VDev **_vdev)
{
    return;
}

/* ( pos.d -- status ) */
static void
ob_virtio_seek(VDev **_vdev)
{
    VDev *vdev = *_vdev;
    uint64_t pos;

    pos = ((uint64_t)POP()) << 32;
    pos |= POP();

    /* Make sure we are within the physical limits */
    if (pos < (vdev->config.blk.capacity * virtio_get_block_size(vdev))) {
        vdev->pos = pos;
        PUSH(0);
    } else {
        PUSH(1);
    }

    return;
}

/* ( addr len -- actual ) */
static void
ob_virtio_read(VDev **_vdev)
{
    VDev *vdev = *_vdev;	
    ucell len = POP();
    uint8_t *addr = (uint8_t *)POP();

    virtio_read(vdev, vdev->pos, addr, len);

    vdev->pos += len;

    PUSH(len);
}

static void set_virtio_alias(const char *path, int idx)
{
    phandle_t aliases;
    char name[9];

    aliases = find_dev("/aliases");

    snprintf(name, sizeof(name), "virtio%d", idx);

    set_property(aliases, name, path, strlen(path) + 1);
}

static void
ob_virtio_initialize(VDev **_vdev)
{
    phandle_t ph = get_cur_dev();
    int len, i;
    uint8_t status;

    VDev *vdev;
    VRing *block = malloc(sizeof(VRing) * VIRTIO_MAX_VQS);
    void *ring_area;

    vdev = malloc(sizeof(VDev));
    vdev->io_base = get_int_property(ph, "_address", &len);
    push_str("_address");
    feval("delete-property");

    /* Indicate we recognise the device */
    status = virtio_cfg_read8(vdev, VIRTIO_PCI_STATUS);
    status |= VIRTIO_CONFIG_S_ACKNOWLEDGE | VIRTIO_CONFIG_S_DRIVER;
    virtio_cfg_write8(vdev, VIRTIO_PCI_STATUS, status);

    vdev->senseid = VIRTIO_ID_BLOCK;
    vdev->nr_vqs = 1;
    vdev->cmd_vr_idx = 0;
    vdev->wait_reply_timeout = VRING_WAIT_REPLY_TIMEOUT;
    vdev->scsi_block_size = VIRTIO_SCSI_BLOCK_SIZE;
    vdev->blk_factor = 1;

    vdev->vrings = block;
    ofmem_posix_memalign(&ring_area, VIRTIO_RING_SIZE * VIRTIO_MAX_VQS, PAGE_SIZE);
    vdev->ring_area = ring_area;

    for (i = 0; i < vdev->nr_vqs; i++) {
        VqInfo info = {
            .queue = (uintptr_t) vdev->ring_area + (i * VIRTIO_RING_SIZE),
            .align = VIRTIO_PCI_VRING_ALIGN,
            .index = i,
            .num = 0,
        };

        virtio_cfg_write16(vdev, VIRTIO_PCI_QUEUE_SEL, i);
        info.num = virtio_cfg_read16(vdev, VIRTIO_PCI_QUEUE_NUM);

        vring_init(&vdev->vrings[i], &info);

        /* Set block information */
        vdev->guessed_disk_nature = VIRTIO_GDN_NONE;
        vdev->config.blk.blk_size = VIRTIO_SECTOR_SIZE;
        vdev->config.blk.physical_block_exp = 0;

        /* Read sectors */
        vdev->config.blk.capacity = virtio_cfg_read64(vdev, 0x14);

        /* Set queue address */
        virtio_cfg_read32(vdev, VIRTIO_PCI_QUEUE_PFN);
        virtio_cfg_write32(vdev, VIRTIO_PCI_QUEUE_PFN,
                           va2pa(info.queue) >> VIRTIO_PCI_QUEUE_ADDR_SHIFT);
    }

    /* Initialisation complete */
    status |= VIRTIO_CONFIG_S_DRIVER_OK;
    virtio_cfg_write8(vdev, VIRTIO_PCI_STATUS, status);

    *_vdev = vdev;
}

DECLARE_UNNAMED_NODE(ob_virtio, 0, sizeof(VDev *));

NODE_METHODS(ob_virtio) = {
    { NULL,        ob_virtio_initialize    },
    { "open",      ob_virtio_open          },
    { "close",     ob_virtio_close         },
    { "seek",      ob_virtio_seek          },
    { "read",      ob_virtio_read          },
};

void ob_virtio_init(const char *path, const char *dev_name, uint64_t base,
                    uint64_t offset, int idx)
{
    PUSH(offset);
    fword("encode-int");
    push_str("_address");
    fword("property");

    REGISTER_NODE_METHODS(ob_virtio, path);

    set_virtio_alias(path, idx);
}
