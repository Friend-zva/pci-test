#include <errno.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include "../../include/gowin_pcie_bar_drv_uapi.h"
#include "../libs/gowin_utils.h"

#define BAR_SIZE (1024 * 4)
#define DMA_SIZE (1024 * 16) //! 16 for pretty printing

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

void toggle_controller(GowinBar *gwbar, int flag_enable) {
    if (DBG_INFO) {
        printf("check DMA enable: 0x%08x\n", gwbar->ctrl);
    }
    uint32_t val = gwbar->ctrl;
    val = flag_enable ? (val | 1) : (val & ~1);
    gwbar->ctrl = val;
    if (DBG_INFO) {
        printf("check DMA enable: 0x%08x\n", gwbar->ctrl);
    }
}

volatile sig_atomic_t flag_exit = 0;

void handle_sigint(int sig) { flag_exit = 1; }

int main(int argc, char *argv[]) {
    signal(SIGINT, handle_sigint);
    volatile int val;

    Process *proc = init_proc();
    if (proc == NULL) {
        return -1;
    }
    GowinBar *gwbar = proc->gwbar;

    struct gowin_ioctl_param param = {0};
    param.cfg_type = 2;     // dword
    param.cfg_where = 0x88; // Device Control/Status register //* where read dword

    while (1) {
        if (!ioctl(proc->fd, GOWIN_CONFIG_READ_DWORD, &param) &&
            param.cfg_dword != 0xFFFFFFFF) {             //* read cfg_where
            val = (param.cfg_dword & 0xFF1F) | (1 << 5); // payload 256B
            gwbar->devctrl = val;
            break;
        }
    }

    uint8_t rx_tag = 16; // receive
    uint8_t tx_tag = 16; // transmit

    volatile uint8_t *sp = proc->mem_src; // source
    volatile uint64_t sa = proc->dma_src;
    volatile uint8_t *dp = proc->mem_dst; // destination
    volatile uint64_t da = proc->dma_dst;

    uint32_t cnt = 64; // 64 * 4 = 256B
    int size = DMA_SIZE / 2;
    int size_dump = 32; //* (size / 2) - full
    int loop = size / (cnt * 4);

    for (int i = 0; i < size; i++) {
        *(uint16_t *)(&sp[i * 2]) = i % 65536;
    }

    uint32_t block_size = (cnt * 4 + 1023) & (~1023);

    ioctl(proc->fd, GOWIN_IRQ_ENABLE, 0); // turn on IR on channel 0
    ioctl(proc->fd, GOWIN_IRQ_ENABLE, 1); // turn on IR on channel 1
    toggle_controller(gwbar, 1);

    gwbar->intr = 0x00010001;
    gwbar->channel[0].wdma_it_level = 16;
    gwbar->channel[0].rdma_it_level = 16;

    int step = 0;
    int h2c_count = 0, c2h_count = 0;
    volatile int h2c_level = 0, c2h_level = 0;
    const int max_level = 4; // RQ_CC_NUM = 4

    while (c2h_count < loop || h2c_count < loop) {
        if (DBG_INFO) {
            if (DUMP_INFO) {
                for (int i = 0; i < size_dump; i++) {
                    uint16_t lo = *(uint16_t *)(&sp[i * 4]);
                    uint16_t hi = *(uint16_t *)(&sp[i * 4 + 2]);
                    printf("(0x%04x, 0x%04x) ", lo, hi);
                }
                printf("\n");
            }
            printf("start copy to card\n");
        }

        h2c_level = gwbar->channel[0].rdma_status & 0xFF;
        step = (h2c_level < max_level) ? (max_level - h2c_level) : 0;

        while (h2c_count < loop && step > 0) {
            gwbar->channel[0].rdma_src_lo = sa & 0xFFFFFFFC;
            gwbar->channel[0].rdma_src_hi = (sa >> 32) & 0xFFFFFFFF;
            gwbar->channel[0].rdma_len = cnt;
            gwbar->channel[0].rdma_tag = rx_tag++;

            sa += block_size;
            sp += block_size;
            if (sa + block_size > proc->dma_src + DMA_SIZE) {
                sp = proc->mem_src;
                sa = proc->dma_src;
            }

            if (rx_tag >= 32) {
                rx_tag = 16;
            }
            step--;
            h2c_count++;
        }
        do {
            h2c_level = gwbar->channel[0].rdma_status & 0xFF;
        } while (!flag_exit && h2c_level == 0xFF);

        if (h2c_count == loop) {
            do {
                val = 0xC0000000 & gwbar->channel[0].rdma_status;
            } while (!flag_exit && val != 0xC0000000);
        }

        if (DBG_INFO) {
            printf("start copy to host\n");
        }

        c2h_level = gwbar->channel[0].wdma_status;
        step = (c2h_level < max_level) ? (max_level - c2h_level) : 0;

        while (c2h_count < loop && step > 0) {
            gwbar->channel[0].wdma_dst_lo = da & 0xFFFFFFFC;
            gwbar->channel[0].wdma_dst_hi = (da >> 32) & 0xFFFFFFFF;
            gwbar->channel[0].wdma_len = cnt;
            gwbar->channel[0].wdma_tag = tx_tag++;

            da += block_size;
            dp += block_size;
            if (da + block_size > proc->dma_dst + DMA_SIZE) {
                dp = proc->mem_dst;
                da = proc->dma_dst;
            }

            if (tx_tag >= 32) {
                tx_tag = 16;
            }
            step--;
            c2h_count++;
        }
        do {
            c2h_level = gwbar->channel[0].wdma_status & 0xFF;
        } while (!flag_exit && c2h_level == 0xFF);

        if (c2h_count == loop) {
            do {
                val = 0xC0000000 & gwbar->channel[0].wdma_status;
            } while (!flag_exit && val != 0xC0000000);
        }

        if (DUMP_INFO) {
            for (int i = 0; i < size_dump; i++) {
                uint32_t val = *(uint32_t *)(&dp[i * 4]);
                printf("0x%08x ", val);
            }
            printf("\n");
        }

        if (flag_exit) {
            break;
        }
    }

    printf("Result: 0x%08x (waiting 0x%08x)\n", ((uint32_t *)dp)[3],
           ((uint16_t *)sp)[3 * 2] + ((uint16_t *)sp)[3 * 2 + 1]);

    toggle_controller(gwbar, 0);
    ioctl(proc->fd, GOWIN_IRQ_DISABLE, 1); // turn off IR on channel 1
    ioctl(proc->fd, GOWIN_IRQ_DISABLE, 0); // turn off IR on channel 0

    dest_proc(proc);
    return 0;
}
