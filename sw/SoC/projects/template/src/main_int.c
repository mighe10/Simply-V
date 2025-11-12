#include "uninasoc.h"
#include "xaxicdma.h"
#include "xaxicdma_hw.h"
#include "plic.h"
#include "tinyIO.h"
#include <stdint.h>

#define print_str(S) c_printf(S)

// --- Indirizzi dal linker ---
extern const volatile uint32_t _peripheral_AXI_CDMA_start;
#define CDMA_BASEADDR ((uintptr_t)&_peripheral_AXI_CDMA_start)

// --- BRAM map ---
#define MEM_BASEADDR  0x00000000u
#define SRC_ADDR      (MEM_BASEADDR + 0x0000u)
#define DST_ADDR      (MEM_BASEADDR + 0x1000u)
#define WORDS         16u
#define BYTES         (WORDS * 4u)

// --- ID IRQ del CDMA (come in SoC.sv) ---
#define CDMA_IRQ_ID   6

// ---------------------------------------------------------------------
// Utility di stampa (senza %)
// ---------------------------------------------------------------------
static void hex32_to_str(uint32_t v, char out[11]) {
    static const char H[] = "0123456789ABCDEF";
    out[0] = '0'; out[1] = 'x';
    for (int i = 0; i < 8; ++i)
        out[2 + i] = H[(v >> ((7 - i) * 4)) & 0xF];
    out[10] = '\0';
}

static void udec_to_str(uint32_t v, char out[12]) {
    char tmp[12]; int n = 0;
    if (v == 0) { out[0] = '0'; out[1] = '\0'; return; }
    while (v && n < 11) { tmp[n++] = '0' + (v % 10); v /= 10; }
    for (int i = 0; i < n; ++i) out[i] = tmp[n - 1 - i];
    out[n] = '\0';
}

// ---------------------------------------------------------------------
// Globali CDMA
// ---------------------------------------------------------------------
static XAxiCdma cdma;
static XAxiCdma_Config cdma_cfg = {
    .DeviceId   = 0,
    .BaseAddress= CDMA_BASEADDR,
    .HasDRE     = 1,
    .IsLite     = 0,
    .DataWidth  = 32,
    .BurstLen   = 16,
    .AddrWidth  = 32
};

volatile int cdma_done = 0;

// ---------------------------------------------------------------------
// ISR Esterna (chiamata dal PLIC)
// ---------------------------------------------------------------------
void _ext_handler(void)
{
    uint32_t id = plic_claim();

    if (id == CDMA_IRQ_ID) {
        uint32_t sr = XAxiCdma_ReadReg(cdma.BaseAddr, XAXICDMA_SR_OFFSET);

        if (sr & XAXICDMA_XR_IRQ_IOC_MASK) {
            print_str("[ISR] CDMA complete\r\n");
            cdma_done = 1;
        }

        if (sr & XAXICDMA_XR_IRQ_ERROR_MASK) {
            print_str("[ISR] CDMA ERROR\r\n");
        }

        // Clear interrupt bits
        XAxiCdma_WriteReg(cdma.BaseAddr, XAXICDMA_SR_OFFSET, XAXICDMA_XR_IRQ_ALL_MASK);
        plic_complete(id);
    } else {
        plic_complete(id);
    }
}

// ---------------------------------------------------------------------
// MAIN
// ---------------------------------------------------------------------
int main(void)
{
    uninasoc_init();
    print_str("\r\n=== CDMA Interrupt Test Start ===\r\n");

    // --- Reset + init CDMA ---
    XAxiCdma_Reset(&cdma);
    for (volatile uint32_t d = 0; d < 10000; ++d);
    XAxiCdma_CfgInitialize(&cdma, &cdma_cfg, CDMA_BASEADDR);

    // --- Init PLIC ---
    plic_init();
    uint32_t priorities[8];
    for (int i = 0; i < 8; i++) priorities[i] = 1;
    priorities[CDMA_IRQ_ID] = 2; // più alta
    plic_configure(priorities, 8);
    plic_enable_all();

    // --- Abilita global interrupt (MIE=1, MEIE=1) ---
    asm volatile("csrrsi zero, mstatus, 8"); // MIE
    asm volatile("li t0, (1 << 11); csrrs zero, mie, t0"); // MEIE

    // --- Buffers ---
    volatile uint32_t *src = (uint32_t *)SRC_ADDR;
    volatile uint32_t *dst = (uint32_t *)DST_ADDR;

    for (uint32_t i = 0; i < WORDS; i++) {
        src[i] = (i * 0x11111111u) ^ 0xA5A5A5A5u;
        dst[i] = 0xFFFFFFFFu;
    }

    print_str("SRC[0..3] before: ");
    for (uint32_t i = 0; i < 4; i++) {
        char buf[11]; hex32_to_str(src[i], buf);
        c_printf(buf); c_printf(" ");
    }
    print_str("\r\nDST[0..3] before: ");
    for (uint32_t i = 0; i < 4; i++) {
        char buf[11]; hex32_to_str(dst[i], buf);
        c_printf(buf); c_printf(" ");
    }
    c_printf("\r\n");

    // --- Abilita interrupt CDMA (IOC + ERROR) ---
    XAxiCdma_IntrEnable(&cdma, XAXICDMA_XR_IRQ_IOC_MASK | XAXICDMA_XR_IRQ_ERROR_MASK);

    // --- Start transfer ---
    print_str("Starting CDMA transfer...\r\n");
    int ret = XAxiCdma_SimpleTransfer(&cdma, SRC_ADDR, DST_ADDR, BYTES, NULL, NULL);
    if (ret != 0) {
        print_str("Transfer start failed\r\n");
        while (1);
    }

    // --- Attesa interrupt ---
    int timeout = 20000000;
    while (!cdma_done && timeout--) {
        // piccolo delay software
        __asm__ __volatile__("nop");
    }

    if (!cdma_done) {
        print_str("[ERR] Timeout - no interrupt?\r\n");
    }

    // --- Verifica risultato ---
    print_str("Checking results...\r\n");
    uint32_t errors = 0;
    for (uint32_t i = 0; i < WORDS; i++) {
        if (dst[i] != src[i]) errors++;
    }

    print_str("SRC[0..3] after: ");
    for (uint32_t i = 0; i < 4; i++) {
        char buf[11]; hex32_to_str(src[i], buf);
        c_printf(buf); c_printf(" ");
    }
    print_str("\r\nDST[0..3] after: ");
    for (uint32_t i = 0; i < 4; i++) {
        char buf[11]; hex32_to_str(dst[i], buf);
        c_printf(buf); c_printf(" ");
    }
    c_printf("\r\n");

    if (errors == 0) {
        print_str("Transfer OK — Interrupt mode works!\r\n");
    } else {
        char ebuf[12]; udec_to_str(errors, ebuf);
        c_printf("Transfer ERROR — mismatches: "); c_printf(ebuf); c_printf("\r\n");
    }

    while (1);
    return 0;
}

