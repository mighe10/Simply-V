/*
 * Author: Michele Giugliano <michele.giugliano2@studenti.unina.it>
 * Description:
 *   CDMA Simple Mode + Interrupts using UninaSoC HAL (PLIC + IRQ handlers)
 *   Fully compatible with Simply-V software stack.
 */

#include "uninasoc.h"
#include "plic.h"
#include "xaxicdma.h"
#include "xaxicdma_hw.h"
#include <stdint.h>
#include <stdio.h>

/* ============================================================
 * CDMA Base Address
 * ============================================================ */
extern const volatile uint32_t _peripheral_AXI_CDMA_start;
#define CDMA_BASEADDR ((uintptr_t)&_peripheral_AXI_CDMA_start)

/* ============================================================
 * Transfer Parameters
 * ============================================================ */
#define WORDS  16u
#define BYTES  (WORDS * 4u)

/* CDMA interrupt source ID (defined in SoC design – yours is #6) */
#define CDMA_IRQ_ID 6

/* ============================================================
 * DMA Buffers (aligned)
 * ============================================================ */
__attribute__((section(".dma"), aligned(64)))
static uint32_t S_buf[WORDS];

__attribute__((section(".dma"), aligned(64)))
static uint32_t D_buf[WORDS];

/* ============================================================
 * CDMA Driver Structures
 * ============================================================ */
static XAxiCdma Cdma;

static XAxiCdma_Config CdmaCfg = {
    .DeviceId    = 0,
    .BaseAddress = 0,
    .HasDRE      = 1,
    .IsLite      = 0,
    .DataWidth   = 32,
    .BurstLen    = 16,
    .AddrWidth   = 32
};

volatile int cdma_done = 0;

/* ============================================================
 * Custom External Interrupt Handler (overrides weak version)
 * ============================================================ */
void _ext_handler(void) __irq_handler__;

void _ext_handler(void)
{
    uint32_t id = plic_claim();

    if (id == CDMA_IRQ_ID) {
        uint32_t sr = XAxiCdma_ReadReg(Cdma.BaseAddr, XAXICDMA_SR_OFFSET);

        if (sr & XAXICDMA_XR_IRQ_IOC_MASK)
            cdma_done = 1;

        if (sr & XAXICDMA_XR_IRQ_ERROR_MASK)
            printf("[ISR] CDMA ERROR! SR=0x%08x\n", sr);

        /* Ack IRQ inside CDMA */
        XAxiCdma_WriteReg(Cdma.BaseAddr, XAXICDMA_SR_OFFSET, XAXICDMA_XR_IRQ_ALL_MASK);
    }

    /* Always complete IRQ */
    plic_complete(id);
}

/* ============================================================
 * Minimal debug helpers
 * ============================================================ */
static void dump_preview(const char *tag, volatile uint32_t *src, volatile uint32_t *dst)
{
    printf("%s\n", tag);
    for (uint32_t i = 0; i < WORDS; i++)
        printf("SRC[%u]=0x%08x | DST[%u]=0x%08x\n", i, src[i], i, dst[i]);
}

/* ============================================================
 * Enable global machine external interrupts
 * ============================================================ */
static inline void enable_global_interrupts(void)
{
    /* Enable MIE in mstatus */
    uint32_t mstatus;
    __asm__ volatile("csrr %0, mstatus" : "=r"(mstatus));
    mstatus |= (1 << 3);
    __asm__ volatile("csrw mstatus, %0" :: "r"(mstatus));

    /* Enable MEIE in mie */
    uint32_t mie;
    __asm__ volatile("csrr %0, mie" : "=r"(mie));
    mie |= (1 << 11);
    __asm__ volatile("csrw mie, %0" :: "r"(mie));
}

/* ============================================================
 *                           MAIN
 * ============================================================ */
int main(void)
{
    uninasoc_init();
    printf("\n=== CDMA Test with PLIC Interrupt ===\n");

    /* Register custom external ISR in vector table */
    extern void _ext_handler(void);
    __asm__ volatile("csrw mtvec, %0" :: "r"(&_ext_handler));

    /* Prepare buffers */
    for (uint32_t i = 0; i < WORDS; i++) {
        S_buf[i] = 0xA5A5A5A5 ^ i;
        D_buf[i] = 0xFFFFFFFF;
    }

    printf("[ADDR] S_buf=0x%08x  D_buf=0x%08x\n",
           (unsigned)S_buf, (unsigned)D_buf);

    /* =======================================================
     *                 Initialize PLIC
     * ======================================================= */
    plic_init();

    /* Priorities array — CDMA gets priority 1 */
    uint32_t prio[7] = {0};
    prio[CDMA_IRQ_ID] = 1;
    plic_configure(prio, 7);

    /* Enable all sources configured */
    plic_enable_all();

    /* Enable CPU level MEIE */
    enable_global_interrupts();

    /* =======================================================
     *                 Initialize CDMA
     * ======================================================= */
    Cdma.BaseAddr = CDMA_BASEADDR;

    XAxiCdma_Reset(&Cdma);
    while (!XAxiCdma_ResetIsDone(&Cdma));

    if (XAxiCdma_CfgInitialize(&Cdma, &CdmaCfg, CDMA_BASEADDR) != 0) {
        printf("CDMA init failed!\n");
        while (1);
    }

    /* Enable CDMA interrupts */
    XAxiCdma_IntrEnable(&Cdma,
        XAXICDMA_XR_IRQ_IOC_MASK |
        XAXICDMA_XR_IRQ_ERROR_MASK);

    dump_preview("Before transfer:", S_buf, D_buf);

    /* =======================================================
     *                 Start transfer
     * ======================================================= */
    cdma_done = 0;

    int st = XAxiCdma_SimpleTransfer(
        &Cdma,
        (uintptr_t)S_buf,
        (uintptr_t)D_buf,
        BYTES,
        NULL, NULL
    );

    if (st != XST_SUCCESS) {
        printf("SimpleTransfer ERROR %d!\n", st);
        while (1);
    }

    /* Wait for interrupt */
    uint32_t timeout = 0;
    while (!cdma_done && timeout++ < 2000000)
        __asm__ volatile("nop");

    if (!cdma_done)
        printf("[WARN] Timeout — ISR not triggered!\n");

    /* =======================================================
     *                 Verify transfer
     * ======================================================= */
    dump_preview("After transfer:", S_buf, D_buf);

    uint32_t err = 0;
    for (uint32_t i = 0; i < WORDS; i++)
        if (S_buf[i] != D_buf[i]) err++;

    if (!err)
        printf("\nTransfer OK — %u words match.\n", WORDS);
    else
        printf("\nTransfer ERROR — mismatches=%u!\n", err);

    while (1);
    return 0;
}

