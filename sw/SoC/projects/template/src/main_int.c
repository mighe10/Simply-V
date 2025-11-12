#include "uninasoc.h"
#include "xaxicdma.h"
#include "xaxicdma_hw.h"
#include "plic.h"
#include <stdint.h>
#include <stdio.h>

/* ====== Config ====== */
extern const volatile uint32_t _peripheral_AXI_CDMA_start;
#define CDMA_BASEADDR ((uintptr_t)&_peripheral_AXI_CDMA_start)

#define WORDS  16u
#define BYTES  (WORDS * 4u)

/* IRQ ID del CDMA (da SoC.sv) */
#define CDMA_IRQ_ID  6

/* ====== Stato globale ====== */
static XAxiCdma Cdma;
static XAxiCdma_Config CdmaCfg = {
    .DeviceId    = 0,
    .BaseAddress = CDMA_BASEADDR,
    .HasDRE      = 1,
    .IsLite      = 0,
    .DataWidth   = 32,
    .BurstLen    = 16,
    .AddrWidth   = 32
};

/* buffer in RAM (no indirizzi fissi); volatile per evitare ottimizzazioni */
static volatile uint32_t src[WORDS];
static volatile uint32_t dst[WORDS];

static volatile int cdma_done = 0;

/* ====== Helpers ====== */
static inline void enable_global_meie(void) {
    /* mstatus.MIE = 1 */
    uint32_t mstatus;
    __asm__ volatile ("csrr %0, mstatus" : "=r"(mstatus));
    mstatus |= (1u << 3);
    __asm__ volatile ("csrw mstatus, %0" :: "r"(mstatus));
    /* mie.MEIE = 1 (bit 11) */
    uint32_t mie;
    __asm__ volatile ("csrr %0, mie" : "=r"(mie));
    mie |= (1u << 11);
    __asm__ volatile ("csrw mie, %0" :: "r"(mie));
}

static void dump_preview(const char* tag) {
    printf("%s\n", tag);
    for (uint32_t i = 0; i < (WORDS < 8 ? WORDS : 8); ++i) {
        printf("SRC[%u] = 0x%08x | DST[%u] = 0x%08x\n",
               i, (unsigned)src[i], i, (unsigned)dst[i]);
    }
}

/* ====== External IRQ handler (PLIC) ====== */
void _ext_handler(void) __attribute__((interrupt("machine")));
void _ext_handler(void) {
    uint32_t id = plic_claim();

    if (id == CDMA_IRQ_ID) {
        uint32_t sr = XAxiCdma_ReadReg(Cdma.BaseAddr, XAXICDMA_SR_OFFSET);

        if (sr & XAXICDMA_XR_IRQ_IOC_MASK) {
            /* fine trasferimento */
            cdma_done = 1;
        }
        if (sr & XAXICDMA_XR_IRQ_ERROR_MASK) {
            printf("[ISR] CDMA ERROR (SR=0x%08x)\n", (unsigned)sr);
        }

        /* clear IRQ bits scrivendo gli stessi bit nello SR */
        XAxiCdma_WriteReg(Cdma.BaseAddr, XAXICDMA_SR_OFFSET, XAXICDMA_XR_IRQ_ALL_MASK);
        plic_complete(id);
        return;
    }

    /* default: complete comunque */
    plic_complete(id);
}

/* ====== MAIN ====== */
int main(void) {
    uninasoc_init();
    printf("\n=== CDMA interrupt test (no timeouts) ===\n");

    /* Init CDMA (niente reset a catena fra i round) */
    if (XAxiCdma_CfgInitialize(&Cdma, &CdmaCfg, CDMA_BASEADDR) != 0) {
        printf("[CDMA] CfgInitialize failed\n");
        while (1) { }
    }

    /* Abilita IRQ CDMA: IOC + ERROR (su Control Register via driver) */
    XAxiCdma_IntrEnable(&Cdma, XAXICDMA_XR_IRQ_IOC_MASK | XAXICDMA_XR_IRQ_ERROR_MASK);

    /* Init PLIC: priorità base =1, CDMA=2, poi enable-all */
    plic_init();
    uint32_t prio[8];
    for (int i = 0; i < 8; ++i) prio[i] = 1;
    prio[CDMA_IRQ_ID] = 2;
    plic_configure(prio, 8);
    plic_enable_all();

    /* Global interrupts: mstatus.MIE e mie.MEIE */
    enable_global_meie();

    /* ====== Riempi buffer con pattern, nessuna sovrapposizione al codice ====== */
    for (uint32_t i = 0; i < WORDS; ++i) {
        src[i] = (i * 0x11111111u) ^ 0xA5A5A5A5u;
        dst[i] = 0xFFFFFFFFu;
    }
    dump_preview("Before transfer:");

    /* ====== Avvio trasferimento su indirizzi dei buffer ====== */
    cdma_done = 0;
    int st = XAxiCdma_SimpleTransfer(&Cdma,
                                     (uintptr_t)src,
                                     (uintptr_t)dst,
                                     BYTES,
                                     NULL, NULL);
    if (st != 0) {
        printf("[CDMA] SimpleTransfer failed (%d)\n", st);
        while (1) { }
    }

    /* ====== Spin FINO all’IRQ (niente timeout) ====== */
    while (!cdma_done) { /* wfi soft */
        __asm__ volatile ("nop");
    }

    /* ====== Verifica ====== */
    uint32_t errors = 0;
    for (uint32_t i = 0; i < WORDS; ++i) {
        if (dst[i] != src[i]) { ++errors; }
    }
    dump_preview("After transfer:");

    if (errors == 0) {
        printf("Transfer OK — all %u words match\n", (unsigned)WORDS);
    } else {
        printf("Transfer ERROR — mismatches=%u\n", (unsigned)errors);
    }

    while (1) { }
    return 0;
}
