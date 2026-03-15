#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <fcntl.h>
// #include <sys/types.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include "../../include/gowin_pcie_bar_drv_uapi.h"
#include "../libs/gowin_utils.h"

#define BAR_SIZE (1024 * 4)
#define DMA_SIZE (1024 * 16)

int DBG_INFO = 1;
int DUMP_INFO = 1;

typedef struct __gowin_bar {
    volatile uint32_t ctrl;     //* 0x0000 - global enable
    volatile uint32_t chsw;     //* 0x0004 - channel enable
    volatile uint32_t intr;     //* 0x0008 - irq enable
    volatile uint32_t aclr;     //* 0x000C - auto irq clear
    volatile uint32_t ista;     //* 0x0010 - irq status and clear
    volatile uint32_t rsv1[3];  //* 0x0014-0x001F - reservation
    volatile uint32_t devctrl;  //* 0x0020 - device control
    volatile uint32_t rsv2[55]; //* 0x0024-0x00FF - reservation

    // volatile uint32_t ddr_base_lo;
    // volatile uint32_t ddr_base_hi;
    // volatile uint32_t ddr_len;
    // volatile uint32_t ddr_ctrl;

    struct {
        volatile uint32_t rdma_src_lo;   //* 0x0100 - lower address of system memory
        volatile uint32_t rdma_src_hi;   //* 0x0104 - upper address of system memory
        volatile uint32_t rdma_len;      //* 0x0108 - length for RDMA
        volatile uint32_t rdma_tag;      //* 0x010C - reference TAG for RDMA
        volatile uint32_t rdma_it_level; //* 0x0110 - RDMA irq trigger level
        volatile uint32_t rdma_status;   //* 0x0114 - current RDMA queue level (RO)
        volatile uint32_t rdma_rsv[58];  //* 0x0118-0x01FF - reservation

        volatile uint32_t wdma_dst_lo;   //* 0x0200 - lower address of system memory
        volatile uint32_t wdma_dst_hi;   //* 0x0204 - upper address of system memory
        volatile uint32_t wdma_len;      //* 0x0208 - length for WDMA
        volatile uint32_t wdma_tag;      //* 0x020C - reference TAG for RDMA
        volatile uint32_t wdma_it_level; //* 0x0210 - WDMA irq trigger level
        volatile uint32_t wdma_status;   //* 0x0214 - current WDMA queue level (RO)
        volatile uint32_t wdma_rsv[58];  //* 0x0218-0x02FF - reservation
    } channel[8];
} GowinBar;

typedef struct process {
    uint64_t dma_src;
    uint8_t *mem_src;

    uint64_t dma_dst;
    uint8_t *mem_dst;

    int fd;
    GowinBar *gwbar;
} Process;

Process *init_proc() {
    int fd = dev_open(NULL);
    if (fd < 0) {
        fprintf(stderr, "Failed to open the device (%s)\n", strerror(errno));
        return NULL;
    }

    Process *proc = (Process *)malloc(sizeof(Process));
    if (proc == NULL) {
        fprintf(stderr, "Failed to allocate memory\n");
        return NULL;
    }

    proc->dma_src = request_mem(fd, 0, DMA_SIZE);
    if (proc->dma_src == 0) {
        return NULL;
    }
    proc->mem_src = mmap_mem(fd, 0, DMA_SIZE);
    if (proc->mem_src == NULL) {
        return NULL;
    }
    proc->dma_dst = request_mem(fd, 1, DMA_SIZE);
    if (proc->dma_dst == 0) {
        return NULL;
    }
    proc->mem_dst = mmap_mem(fd, 1, DMA_SIZE);
    if (proc->mem_dst == NULL) {
        return NULL;
    }

    proc->fd = fd;

    uint32_t *bar = mmap_bar(fd, 0, BAR_SIZE);
    if (bar == NULL) {
        return NULL;
    }
    proc->gwbar = (GowinBar *)bar;

    return proc;
}

void dest_proc(Process *proc) {
    if (proc == NULL) {
        return;
    }
    if (proc->gwbar) {
        munmap(proc->gwbar, BAR_SIZE);
    }
    if (proc->mem_dst) {
        munmap(proc->mem_dst, DMA_SIZE);
    }
    if (proc->dma_dst) {
        release_mem(proc->fd, 1);
    }
    if (proc->mem_src) {
        munmap(proc->mem_src, DMA_SIZE);
    }
    if (proc->dma_src) {
        release_mem(proc->fd, 0);
    }
    free(proc);
}

int main(int argc, char *argv[]) {
    volatile int val;
    struct gowin_ioctl_param param = {0};
    Process *proc = init_proc();
    if (proc == NULL) {
        return -1;
    }

    param.cfg_type = 2; // dword
    param.cfg_where = 0x88; // register Device Control/Status

    while (1) {
        if (!ioctl(proc->fd, GOWIN_CONFIG_READ_DWORD, &param) &&
            param.cfg_dword != 0xFFFFFFFF) {
            val = (param.cfg_dword & 0xFF1F) | (1 << 5);
            proc->gwbar->devctrl = val; // payload
            break;
        }
    }

    uint8_t rx_tag = 16; // receive
    uint8_t tx_tag = 16; // transmit

    volatile uint8_t *sp = proc->mem_src; // source
    volatile uint64_t sa = proc->dma_src;
    volatile uint8_t *dp = proc->mem_dst; // destination
    volatile uint64_t da = proc->dma_dst;

    int cnt_n = 4; //! test
    int cnt_try = 1; //! test

    fprintf(stdout, "\nFirst  Number: ");
    uint8_t x;
    scanf("%hhu", &x);
    ((uint32_t *)sp)[0] = x;

    fprintf(stdout, "Second Number: ");
    uint8_t y;
    scanf("%hhu", &y);
    ((uint32_t *)sp)[1] = y;

    int block_size = (cnt_n * 4 + 1023) & (~1023);

    ioctl(proc->fd, GOWIN_IRQ_ENABLE, 0); // turn on IR on channel 0
    ioctl(proc->fd, GOWIN_IRQ_ENABLE, 1); // turn on IR on channel 1

    if (DBG_INFO) {
        fprintf(stdout, "check DMA enable: 0x%08x\n", proc->gwbar->ctrl);
    }

    val = proc->gwbar->ctrl;
    val |= 1; // turn on controller
    proc->gwbar->ctrl = val;

    if (DBG_INFO) {
        fprintf(stdout, "check DMA enable: 0x%08x\n", proc->gwbar->ctrl);
    }

    proc->gwbar->intr = 1;
    proc->gwbar->channel[0].wdma_it_level = 16;
    proc->gwbar->channel[0].rdma_it_level = 16;

    // h2c

    if (DBG_INFO) {
        for (int i = 0; i < DMA_SIZE / 4; i++) {
            fprintf(stdout, "0x%02x ", sp[i * 4]);
        }
    }

    if (DBG_INFO) {
        fprintf(stdout, "start copy to card\n");
    }

    for (int i = 0; i < cnt_try; i++) {
        proc->gwbar->channel[0].rdma_src_lo = sa & 0xFFFFFFFF;
        proc->gwbar->channel[0].rdma_src_hi = (sa >> 32) & 0xFFFFFFFF;
        proc->gwbar->channel[0].rdma_len = cnt_n;
        proc->gwbar->channel[0].rdma_tag = rx_tag++;

        if (cnt_try != 1) {
            sa += block_size;
            sp += block_size;
            if (sa + block_size > proc->dma_src + DMA_SIZE) {
                sp = proc->mem_src;
                sa = proc->dma_src;
            }
        }
    }

    if (DBG_INFO) {
        for (int i = 0; i < DMA_SIZE / 4; i++) {
            fprintf(stdout, "0x%02x ", sp[i * 4]);
        }
    }

    volatile int h2c_done = 0;
    do {
        h2c_done = proc->gwbar->channel[0].rdma_status & 0xFF;
        if (DUMP_INFO) {
            fprintf(stdout, "loop1 (h2c_done)\n");
        }
    } while (255 == h2c_done);

    // c2h

    if (DBG_INFO) {
        fprintf(stdout, "start copy to host\n");
    }

    proc->gwbar->intr = 1 << 16;

    for (int i = 0; i < cnt_try; i++) {
        proc->gwbar->channel[0].wdma_dst_lo = da & 0xFFFFFFFF;
        proc->gwbar->channel[0].wdma_dst_hi = (da >> 32) & 0xFFFFFFFF;
        proc->gwbar->channel[0].wdma_len = cnt_n;
        proc->gwbar->channel[0].wdma_tag = tx_tag++;

        if (cnt_try != 1) {
            da += block_size;
            dp += block_size;
            if (da + block_size > proc->dma_dst + DMA_SIZE) {
                dp = proc->mem_dst;
                da = proc->dma_dst;
            }
        }
    }

    volatile int c2h_done = 0;
    do {
        c2h_done = proc->gwbar->channel[0].wdma_status & 0xFF;
        if (DUMP_INFO) {
            fprintf(stdout, "loop2 (c2h_done)\n");
        }
    } while (255 == c2h_done);

    fprintf(stdout, "Result: %u (waiting %hhu)\n", ((uint32_t *)dp)[0], x + y);
    if (DBG_INFO) {
        for (int i = 0; i < DMA_SIZE / 4; i++) {
            fprintf(stdout, "0x%02x ", dp[i * 4]);
        }
    }

    ioctl(proc->fd, GOWIN_IRQ_DISABLE, 1); // turn off IR on channel 1
    ioctl(proc->fd, GOWIN_IRQ_DISABLE, 0); // turn off IR on channel 0

    dest_proc(proc);
    return 0;
}
