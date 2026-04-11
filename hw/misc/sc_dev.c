// sc-dev.c
// TODO: hw/riscv/virt.c     [VIRT_SC_DEV] =       {  0x8000000,         0x100 },
// MMIO callbacks to SystemC device

#include <sys/socket.h>
#include <sys/un.h>

#include "qemu/osdep.h"
#include "qapi/error.h" 		 /* Provides error_fatal() handler */
#include "qemu/log.h"			 /* Error logs */
#include "hw/sysbus.h"  		 /* Provides all sysbus registering functions */
#include "hw/qdev-properties.h"  /* QEMU device properties */
#include "hw/irq.h"              /* IRQ functions */
#include "system/dma.h"          /* DMA functions */
#include "qemu/timer.h"          /* Timer functions */

#include "hw/misc/sc_dev.h"

#define TYPE_SC_DEV "sc_dev"
typedef struct ScDevState ScDevState;
DECLARE_INSTANCE_CHECKER(ScDevState, SC_DEV, TYPE_SC_DEV)

/* Memory controller registers & bit flags */
#define REG_CONTROL   0x00       // Control bits (START, READ/WRITE, COMPUTE, IRQ enable)
#define REG_ADDR      0x04       // Target SRAM address
#define REG_LEN       0x08       // Transfer size (currently 4 bytes/1 word)
#define REG_WDATA     0x0C       // Data the CPU wants to write to SRAM
#define REG_RDATA     0x10       // Data read back from SRAM
#define REG_STATUS    0x14       // Status bits (BUSY, DONE, ERROR)

#define CTRL_START   1u << 0     // b0001, 0x01
#define CTRL_WRITE   1u << 1     // b0010, 0x02
#define CTRL_IRQEN   1u << 3     // b0100, 0x04
#define CTRL_COMPUTE   1u << 4   // b1000, 0x08

// CTRL STATUS bits
#define STAT_BUSY   1u << 0      // b0001, 0x01
#define STAT_DONE   1u << 1      // b0010, 0x02
#define STAT_ERR    1u << 2      // b0100, 0x04

/* DMA registers & bit flags */
#define DMA_BUFFER_SIZE (16 * 1024 * 1024)   // 16MB maximum burst size

#define DMA_CONTROL   0x20        // DMA control
#define DMA_SRC_LO    0x24        // Lower 32-bits of 64-bit address (Guest RAM)
#define DMA_SRC_HI    0x28        // Upper 32-bits of 64-bit address (Guest RAM)
#define DMA_DST       0x2C        // 32-bit address (Device SRAM)
#define DMA_LEN       0x30        // Length of transfer
#define DMA_STAT      0x34        // DMA status

/* DMA Control Bit Flags */
#define DMA_START     (1 << 0)
#define DMA_DIR_READ  (1 << 1)   /* 0: RAM->SRAM, 1: SRAM->RAM */

/* DMA Status Bit Flags */
#define DMA_BUSY      (1 << 0)
#define DMA_DONE      (1 << 1)
#define DMA_ERR       (1 << 2)

/* Timing registers*/
#define REG_EXCESS_TIME_LO  0x40  // SystemC simulator time correction (upper 32-bits)
#define REG_EXCESS_TIME_HI  0x44  // SystemC simulator time correction (lower 32-bits)

#define REG_TIMING_CLEAR    0x48  // Write to clear timing data
#define TIMING_CLEAR        (1 << 0) 

/* IRQ registers & bit flags */
#define REG_IRQ_STATUS  0x60      // Read to find which IRQs are pending
#define REG_IRQ_CLEAR   0x64      // Write to clear pending IRQs
#define REG_IRQ_ENABLE  0x68      // Enable or disable IRQs

#define IRQ_CTRL_DONE  (1 << 0)   // b001 - bit 0
#define IRQ_DMA_DONE   (1 << 1)   // b010 - bit 1
#define IRQ_JOB_DONE   (1 << 2)   // b100 - bit 2

/* Job registers & bit flags */
#define REG_JOB_CONTROL     0x80 // write 1 to start Job (end-to-end inference)
#define REG_JOB_STATUS      0x84 // DONE/BUSY/ERR

#define REG_JOB_IN_SRC_LO   0x88
#define REG_JOB_IN_SRC_HI   0x8C
#define REG_JOB_IN_DST      0x90
#define REG_JOB_IN_LEN      0x94

#define REG_JOB_OUT_SRC     0x98
#define REG_JOB_OUT_DST_LO  0x9C
#define REG_JOB_OUT_DST_HI  0xA0
#define REG_JOB_OUT_LEN     0xA4

#define JOB_START           (1u << 0)

#define JOB_BUSY            (1u << 0)
#define JOB_DONE            (1u << 1)
#define JOB_ERR             (1u << 2)


/* Forward declarations */
static void sc_dev_instance_init(Object *obj);
static void sc_dev_realize(DeviceState *dev, Error **errp);
static void sc_dev_unrealize(DeviceState *dev);

static int8_t sc_dev_tlm_transaction(ScDevState *state, 
									   uint8_t is_write, uint8_t is_dma, 
                                       uint64_t addr, void *buf, uint32_t size);
static uint64_t sc_dev_read(void *opaque, hwaddr addr, unsigned int size);
static void sc_dev_write(void *opaque, hwaddr addr, uint64_t data, unsigned int size);
static void sc_dev_dma(ScDevState *state);
static void sc_dev_run_job(ScDevState *state);
// static void sc_dev_job_bh(void *opaque);

static void sc_dev_update_irq(ScDevState* state);
static void sc_dev_raise_irq(ScDevState* state, uint32_t irq);
static void sc_dev_clear_irq(ScDevState* state, uint32_t irq);

/* Device state structure 
   Keep track of the status of the device in this board. Specific registers & IRQs may be defined here 
 */
struct ScDevState {
	SysBusDevice parent_obj;
	MemoryRegion iomem;   // Memory mapped space for the device
    AddressSpace *dma_as; // DMA address space

    /* DMA registers */
    uint64_t dma_src;
    uint32_t dma_dst;
    uint32_t dma_len;
    uint32_t dma_ctrl;
    uint32_t dma_stat;

    /* Interrupts */
    qemu_irq irq_dma;         // DMA completion
    qemu_irq irq_ctrl;        // Controller/CIM completion
    qemu_irq irq_job;         // End-to-end inference completion

    uint32_t irq_status;      // Pending IRQs (bit 0: controller, bit 1: DMA)
    uint32_t irq_enable;      // Enabled IRQs (bit 0: controller, bit 1: DMA)

    /*  Job registers/state 
     *  Job mode: runs one inference end-to-end
     *  1. Perform DMA write-to-device using sc_dev_dma() logic
     *  2. Trigger compute (SystemC ctrl transaction)
     *  3. Perform DMA read-from-device
    */
    uint32_t job_control;
    uint32_t job_status;

    uint64_t job_in_src;
    uint32_t job_in_dst;
    uint32_t job_in_len;

    uint32_t job_out_src;
    uint64_t job_out_dst;
    uint32_t job_out_len;

    bool job_mode_active;

    /* Bottom half: Run Job asynchronously */
    // QEMUBH *job_bh;
    // bool job_pending;
    // bool job_running;

    /* Timing */
    int64_t excess_time_ns;   // Accumulated "excess" time due to simulated SystemC time being faster than wall-clock time

    /* Socket connection to external device */
    int sc_sock;
    char *socket_path;
};

#define BRIDGE_HEADER_SIZE sizeof(struct bridge_msg)

struct bridge_msg {
    uint8_t  is_write;
    uint8_t  is_dma;
    uint8_t  ctrl_irq; /* IRQ forwarded from SystemC memory controller*/
    uint8_t  reserved; /* Ensure word alignment */

    uint64_t addr;
    uint32_t size;

    uint64_t simulated_ns;
    int8_t   status;
    uint8_t  pad[7];   /* keep alignment/size neat */
} __attribute__((packed));  /* Ensure no padding */

static int send_all(int fd, const void *buf, size_t len)
{
    const uint8_t *p = buf;
    while (len) {
        ssize_t n = send(fd, p, len, MSG_NOSIGNAL); // Prevents crash on disconnect
        if (n <= 0) return -1;
        p += (size_t)n;
        len -= (size_t)n;
    }
    return 0;
}

static int recv_all(int fd, void *buf, size_t len)
{
    uint8_t *p = buf;
    while (len) {
        ssize_t n = recv(fd, p, len, MSG_WAITALL);
        if (n <= 0) return -1;
        p += (size_t)n;
        len -= (size_t)n;
    }
    return 0;
}

/* Helper: Send and receive messages formatted for TLM transactions. 
   Returns status of transaction, forwarded from SystemC
 */
static int8_t sc_dev_tlm_transaction(ScDevState *state,
                                     uint8_t is_write, uint8_t is_dma,
                                     uint64_t addr, void *buf, uint32_t size)
{
    struct bridge_msg msg = {
        .is_write = is_write,
        .is_dma = is_dma,
        .addr = addr,
        .size = size,
        .simulated_ns = 0,
        .status = 0,  /* Code for incomplete message */
    };    
    struct bridge_msg response;
    int64_t t_rt_before, t_rt_after, t_round_trip; // Used to time full round trip simulated time (socket send -> SystemC -> socket recv)


	/* Ensure bridge socket connection */
    if (state->sc_sock < 0) {
        qemu_log_mask(LOG_GUEST_ERROR, "sc_dev: socket not connected\n");
        return -1;
    }

    /* Safety: never claim more than payload capacity per transaction */
    if (size > DMA_BUFFER_SIZE) {
        qemu_log_mask(LOG_GUEST_ERROR, "sc_dev: TLM size %u exceeds DMA_BUFFER_SIZE\n", size);
        return -1;
    }

    /* Mark time before entering SystemC time */
    t_rt_before = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);

    /* ---- send header ---- */
    if (send_all(state->sc_sock, &msg, BRIDGE_HEADER_SIZE) < 0) {
        qemu_log_mask(LOG_GUEST_ERROR, "sc_dev: failed to send TLM header\n");
        return -1;
    }

    /* send payload only for writes with size > 0 */
    if (is_write && size > 0 && buf) {
        if (send_all(state->sc_sock, buf, size) < 0) {
            qemu_log_mask(LOG_GUEST_ERROR, "sc_dev: failed to send TLM payload\n");
            return -1;
        }
    }

    /* ---- recv response header ---- */
    if (recv_all(state->sc_sock, &response, BRIDGE_HEADER_SIZE) < 0) {
        qemu_log_mask(LOG_GUEST_ERROR, "sc_dev: failed to recv TLM header\n");
        return -1;
    }

    /* recv payload only for reads with size > 0 */
    if (!is_write && size > 0 && buf) {
        if (recv_all(state->sc_sock, buf, size) < 0) {
            qemu_log_mask(LOG_GUEST_ERROR, "sc_dev: failed to recv TLM payload\n");
            return -1;
        }
    }

    /* Mark time after returning from SystemC time */
    t_rt_after = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
    t_round_trip = t_rt_after - t_rt_before;

    /* Accumulate correction for excess: wall clock time - simulated time */
    state->excess_time_ns += t_round_trip - response.simulated_ns;

    // qemu_log("sc_dev: %s %s addr=0x%04lx size=%04u | sim=%05" PRIu64 " wall=%06" PRIu64 " overhead=%" PRId64 "\n",
    //      is_dma ? " DMA" : "MMIO",
    //      is_write ? "WR" : "RD",
    //      (unsigned long)addr, (unsigned)size,
    //      (uint64_t)response.simulated_ns,
    //      (uint64_t)t_round_trip,
    //      (int64_t)(t_round_trip - response.simulated_ns));

    /* Forward controller IRQ (if not in Job mode) */
    if (response.ctrl_irq && !state->job_mode_active) {
        sc_dev_raise_irq(state, IRQ_CTRL_DONE);
    }

    return response.status;
}

/* Memory read callback */
static uint64_t sc_dev_read(void *opaque, hwaddr addr, unsigned int size)
{
    ScDevState *state = SC_DEV(opaque);

    switch (addr) {
        /* DMA registers */
        case DMA_STAT:
            return state->dma_stat;

        /* IRQ registers */
        case REG_IRQ_STATUS:
            return state->irq_status;

        case REG_IRQ_ENABLE:
            return state->irq_enable;

        /* Job registers */
        case REG_JOB_STATUS:
            return state->job_status;

        /* Timing registers */
        case REG_EXCESS_TIME_LO:
            return (uint32_t)(state->excess_time_ns);
        
        case REG_EXCESS_TIME_HI:
            return (uint32_t)(state->excess_time_ns >> 32);

        /* Forward register access transaction to SystemC and receive response */
        default:
            uint8_t buf[8] = {0};  // Stack buffer for read data
            uint32_t data = 0;     // First 4 control bytes
            int8_t ret;            // TLM transaction status (tlm::tlm_response_status)
            
            if((ret = sc_dev_tlm_transaction(state, 0, 0, addr, buf, size)) < 0) {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "sc_dev: TLM read error at 0x%lx: %d\n",
                              (unsigned long)addr, ret);
                return 0; // TODO: figue out a way to return the TLM error number or some other defined (non-negative) error value
            }

            memcpy(&data, buf, sizeof(uint32_t));
            return data;
    }
}

/* Helper: forward register access to device */
static void sc_dev_forward_write(ScDevState *state, hwaddr addr, uint64_t data, unsigned int size)
{
    uint8_t buf[8] = {0};  // Buffer for write data
    int8_t ret;            // TLM transaction status

    memcpy(buf, &data, size);

    ret = sc_dev_tlm_transaction(state, 1, 0, addr, buf, size);
    if (ret < 0) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "sc_dev: TLM write error at 0x%lx: %d\n",
                      (unsigned long)addr, ret);
    }
}

/* Memory write callback */
static void sc_dev_write(void *opaque, hwaddr addr,
                         uint64_t data, unsigned int size)
{
    ScDevState *state = SC_DEV(opaque);

    switch (addr) {
        /* DMA registers */
        case DMA_CONTROL:
            state->dma_ctrl = data;

            if (data & DMA_START) {
                sc_dev_dma(state);
            }
            break;

        case DMA_SRC_LO:
            state->dma_src &= 0xffffffff00000000ULL;
            state->dma_src |= (uint32_t)data;
            break;

        case DMA_SRC_HI:
            state->dma_src &= 0x00000000ffffffffULL;
            state->dma_src |= ((uint64_t)data << 32); 
            break;

        case DMA_DST:
            state->dma_dst = (uint32_t)data;
            break;

        case DMA_LEN:
            state->dma_len = data;
            break;


        /* IRQ registers */
        case REG_IRQ_ENABLE:
            state->irq_enable = data;
            sc_dev_update_irq(state);
            break;
        
        case REG_IRQ_CLEAR:
            sc_dev_clear_irq(state, data);

            if (data & IRQ_DMA_DONE) {
                state->dma_stat = 0;
            }

            /* Ack the underlying controller condition so SystemC irq goes low.
             * SystemC controller uses W1C to set REG_STATUS
             */
            if (data & IRQ_CTRL_DONE) {
                sc_dev_forward_write(state, REG_STATUS, STAT_DONE, 4);
            }
            break;


        /* Job registers */
        case REG_JOB_IN_SRC_LO:
            state->job_in_src &= 0xffffffff00000000ULL;
            state->job_in_src |= (uint32_t)data;
            break;
        case REG_JOB_IN_SRC_HI:
            state->job_in_src &= 0x00000000ffffffffULL;
            state->job_in_src |= ((uint64_t)(uint32_t)data << 32);
            break;
        case REG_JOB_IN_DST:
            state->job_in_dst = (uint32_t)data;
            break;
        case REG_JOB_IN_LEN:
            state->job_in_len = (uint32_t)data;
            break;
            
        case REG_JOB_OUT_SRC:
            state->job_out_src = (uint32_t)data;
            break;
        case REG_JOB_OUT_DST_LO:
            state->job_out_dst &= 0xffffffff00000000ULL;
            state->job_out_dst |= (uint32_t)data;
            break;
        case REG_JOB_OUT_DST_HI:
            state->job_out_dst &= 0x00000000ffffffffULL;
            state->job_out_dst |= ((uint64_t)(uint32_t)data << 32);
            break;
        case REG_JOB_OUT_LEN:
            state->job_out_len = (uint32_t)data;
            break;
            
        // case REG_JOB_CONTROL:
        //     state->job_control = (uint32_t)data;

        //     /* Schedule job separately through QEMU bottom half */
        //     if (state->job_control & JOB_START) {
        //         /* prevent re-entry / overlapping jobs */
        //         if (!state->job_running) {
        //             state->job_status = JOB_BUSY;
        //             state->job_pending = true;
        //             qemu_bh_schedule(state->job_bh);
        //         } else {
        //             /* already running: set error */
        //             state->job_status = JOB_ERR;
        //         }
        //     }
        //     break;

        case REG_JOB_CONTROL:
            state->job_control = (uint32_t)data;

            if (state->job_control & JOB_START) {
                /*
                 * Synchronous job execution: this MMIO write will block until
                 * input DMA + compute + output DMA complete.
                 */
                if (!state->job_mode_active) {
                    sc_dev_run_job(state);
                } else {
                    state->job_status = JOB_ERR;
                }
            }
            break;


        /* Timing registers */
        case REG_TIMING_CLEAR:
            state->excess_time_ns = 0;
            break;


        /* Forward register access transaction to SystemC and receive response */
        default:
            sc_dev_forward_write(state, addr, data, size);
            break;
    }
}

/* Use DMA to access data stored in memory, then send to CIM device 
   Registers must be set before use:
                          Source address (data to write)
                          Dest address (where to write to)
                          Data length
*/
static void sc_dev_dma(ScDevState *state)
{
    uint8_t *buf = g_malloc(DMA_BUFFER_SIZE); // Temporary heap storage for data read from memory
    uint32_t left = state->dma_len;           // Chunk size a single TLM transaction
    uint32_t chunk;                           // Chunk size a single TLM transaction
    MemTxResult result;                       // Result of DMA memory read

    /* Note: I wrote this DMA code with only writes (guest -> dev) in mind. However, because
     * guest runs in 64-bit and the device in 32-bit, src and dest are not interchangable here.
     * Therefore I will bodge a small fix, where I keep the original register meanings:
     *
     *  - dma_src (64-bit) is ALWAYS guest RAM (AKA guest) address
     *  - dma_dst (32-bit) is ALWAYS device SRAM address
     *
     * Direction bit will decide transfer direction:
     *  - dir_read == 0 : RAM -> DEV  
     *  - dir_read == 1 : DEV -> RAM 
     */
    uint64_t ram  = state->dma_src;                         // always guest RAM
    uint64_t dev  = (uint32_t)state->dma_dst;               // always device SRAM (32-bit)
    bool dir_read = (state->dma_ctrl & DMA_DIR_READ) != 0;  // 1: DEV->RAM

    /* TODO: Test to make sure DMA doesn't run if not all three registers are populated */
    if (!ram || !dev || !left) {
        qemu_log_mask(LOG_GUEST_ERROR, "sc_dev: dma_src (RAM), dma_dst (DEV), or dma_len not set.\n");
        state->dma_stat = DMA_ERR;
        return;
    }

    state->dma_stat = DMA_BUSY;

    while (left) {
        chunk = MIN(left, DMA_BUFFER_SIZE);

        /* Write (Guest -> Device) */
        if (!dir_read) {
            result = dma_memory_read(state->dma_as, ram, buf, chunk, MEMTXATTRS_UNSPECIFIED);
            if (result != MEMTX_OK) {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "sc_dev: DMA RAM read failed at 0x%lx: %d\n",
                              (unsigned long)ram, result);
                state->dma_stat = DMA_ERR;
                return;
            }

            if (sc_dev_tlm_transaction(state, 1, 1, dev, buf, chunk) < 0) {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "sc_dev: DMA write-to-device failed (dev=0x%lx len=%u)\n",
                              (unsigned long)dev, chunk);
                state->dma_stat = DMA_ERR;
                return;
            }
        
        /* Read (Device -> Guest) */
        } else {
            /* Read from SystemC/device SRAM into buf */
            if (sc_dev_tlm_transaction(state, 0, 1, dev, buf, chunk) < 0) {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "sc_dev: DMA read-from-device failed (dev=0x%lx len=%u)\n",
                              (unsigned long)dev, chunk);
                state->dma_stat = DMA_ERR;
                return;
            }

            /* Write buf into guest RAM */
            result = dma_memory_write(state->dma_as, ram, buf, chunk, MEMTXATTRS_UNSPECIFIED);
            if (result != MEMTX_OK) {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "sc_dev: DMA RAM write failed at 0x%lx: %d\n",
                              (unsigned long)ram, result);
                state->dma_stat = DMA_ERR;
                return;
            }
        }
        
        /* Increment addresses and decrement remaining chunks */
        ram  += chunk;
        dev  += chunk;
        left -= chunk;
    }

    /* Raise DMA completion IRQ */
    state->dma_ctrl &= ~DMA_START;
    state->dma_stat = DMA_DONE;

    /* Only raise IRQ if not part of a Job */
    if (!state->job_mode_active) sc_dev_raise_irq(state, IRQ_DMA_DONE);

    /* Zero out DMA registers when finished */
    state->dma_src = 0;
    state->dma_dst = 0;
    state->dma_len = 0;

    /* Clean up the heap memory */
    g_free(buf);
}

/* Perform an end-to-end inference and raise one interrupt on completion*/
static void sc_dev_run_job(ScDevState *state)
{
    /* basic validation */
    if (!state->job_in_src || !state->job_in_dst || !state->job_in_len ||
        !state->job_out_dst || !state->job_out_src || !state->job_out_len) {
        state->job_status = JOB_ERR;
        return;
    }

    state->job_status = JOB_BUSY;
    state->job_mode_active = true;

    /* ---------- Step 1: input DMA (RAM -> SRAM) ---------- */
    state->dma_src  = state->job_in_src;
    state->dma_dst  = state->job_in_dst;
    state->dma_len  = state->job_in_len;
    state->dma_ctrl = DMA_START;            /* write direction */
    sc_dev_dma(state);                      /* performs the copy + socket txns */
    /* IRQ_DMA_DONE is not raised in Job mode */


    /* ---------- Step 2: compute ---------- */
    /* Trigger compute in SystemC via existing MMIO control path.*/
    sc_dev_forward_write(state, REG_LEN, 4, 4);
    sc_dev_forward_write(state, REG_CONTROL, (CTRL_IRQEN | CTRL_COMPUTE | CTRL_START), 4);

    /* IRQ_CTRL_DONE is not raised in Job mode */


    /* ---------- Step 3: output DMA (SRAM -> RAM) ---------- */
    state->dma_src  = state->job_out_dst;   /* dma_src is always RAM */
    state->dma_dst  = state->job_out_src;   /* dma_dst always DEV SRAM address */
    state->dma_len  = state->job_out_len;
    state->dma_ctrl = DMA_START | DMA_DIR_READ;
    sc_dev_dma(state);

    /* done */
    state->job_mode_active = false;
    state->job_status = JOB_DONE;

    /* right before sc_dev_raise_irq(state, IRQ_JOB_DONE); */
    // struct timespec ts = {.tv_sec = 0, .tv_nsec = 100000000}; // 100ms
    // nanosleep(&ts, NULL);

    sc_dev_raise_irq(state, IRQ_JOB_DONE);
}

/*
 * Job bottom half (BH) handler.
 *
 * We schedule this BH when the guest writes JOB_START. The MMIO write callback
 * returns immediately to the guest, and the actual Job execution runs later in
 * QEMU’s main event loop context (BH), not inline in the vCPU MMIO path.
 *
 * Note: sc_dev_run_job() is currently synchronous: it performs input DMA,
 * triggers compute via the SystemC socket bridge, then performs output DMA,
 * all sequentially and blocking. This means the BH itself may run for a while,
 * but the guest sees a "doorbell + wait" programming model: issue JOB_START,
 * then block waiting for IRQ_JOB_DONE.
 */
// static void sc_dev_job_bh(void *opaque)
// {
//     ScDevState *state = opaque;

//     if (!state->job_pending || state->job_running) {
//         return;
//     }

//     state->job_pending = false;
//     state->job_running = true;

//     /* Run the job */
//     sc_dev_run_job(state);

//     state->job_running = false;
// }

/* IRQ Helper Functions */
static void sc_dev_update_irq(ScDevState* state)
{
    // Raise level if IRQs are enabled and the specific IRQ bit is set to 1
    int dma_level = (state->irq_enable & state->irq_status & IRQ_DMA_DONE) ? 1 : 0;
    int ctrl_level = (state->irq_enable & state->irq_status & IRQ_CTRL_DONE) ? 1 : 0;
    int job_level = (state->irq_enable & state->irq_status & IRQ_JOB_DONE) ? 1 : 0;

    qemu_set_irq(state->irq_dma, dma_level);
    qemu_set_irq(state->irq_ctrl, ctrl_level);
    qemu_set_irq(state->irq_job, job_level);
}

static void sc_dev_raise_irq(ScDevState* state, uint32_t irq)
{
    state->irq_status |= irq;
    sc_dev_update_irq(state);
}

static void sc_dev_clear_irq(ScDevState* state, uint32_t irq)
{
    state->irq_status &= ~irq;
    sc_dev_update_irq(state);
}

/* File operations associated with the device
   “E.g. When CPU reads this memory region, call sc_dev_read()”
*/
static const MemoryRegionOps sc_dev_ops = {
	.read = sc_dev_read,
	.write = sc_dev_write,
	.endianness = DEVICE_NATIVE_ENDIAN,
    
    /* Declare valid number of bytes addressable */
    .valid = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
};

/* Device realization.
 */
static void sc_dev_realize(DeviceState *dev, Error **errp)
{
    ScDevState *state = SC_DEV(dev);
    struct sockaddr_un addr = {0};
    const char *path = state->socket_path ? state->socket_path : "/tmp/systemc.sock";

    state->sc_sock = socket(AF_UNIX, SOCK_STREAM, 0);
    if (state->sc_sock < 0) {
        error_setg_errno(errp, errno, "sc_dev: cannot create socket");
        return;
    }

    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, path, sizeof(addr.sun_path) - 1);

    if (connect(state->sc_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        error_setg_errno(errp, errno, "sc_dev: cannot connect to SystemC at %s", path);
        close(state->sc_sock);
        state->sc_sock = -1;
        return;
    }
    
    qemu_log("sc_dev: connected to SystemC at %s\n", path);

    // Declare DMA address space
    state->dma_as = &address_space_memory;
    
    // Initialize registers
    state->dma_src = 0;  
    state->dma_dst = 0;  
    state->dma_len = 0;  
    state->dma_ctrl = 0;  
    state->dma_stat = 0; 
    
    qemu_log("sc_dev: DMA intialized\n");

    /* Initialize IRQ lines */
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &state->irq_dma);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &state->irq_ctrl);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &state->irq_job);

    qemu_log("sc_dev: IRQ lines intialized\n");

    /* Create a new bottom half for Job */
    // state->job_pending = false;
    // state->job_running = false;
    // state->job_bh = qemu_bh_new(sc_dev_job_bh, state);

    // qemu_log("sc_dev: \"Job\" bottom half intialized\n");

}

/* Device unrealization (socket cleanup) */
static void sc_dev_unrealize(DeviceState *dev)
{
    ScDevState *state = SC_DEV(dev);

    if (state->sc_sock >= 0) {
        close(state->sc_sock);
        state->sc_sock = -1;
    }

    // if (state->job_bh) {
    //     qemu_bh_delete(state->job_bh);
    //     state->job_bh = NULL;
    // }
}

/* Device properties */
static const Property sc_dev_properties[] = {
    DEFINE_PROP_STRING("socket-path", ScDevState, socket_path),
};

/* Initialize device state */
static void sc_dev_instance_init(Object *obj)
{
	ScDevState *state = SC_DEV(obj);

	/* Allocate memory (memory map region), file operations associated, and size */
    memory_region_init_io(&state->iomem, obj, &sc_dev_ops, state, TYPE_SC_DEV, 0x100); // TODO: May need to change 0x100 to something else
	
	/* Let the system bus know that this memory region is handled by this device */
	sysbus_init_mmio(SYS_BUS_DEVICE(obj), &state->iomem); 

    /* Initially mark socket as disconnected */
	state->sc_sock = -1;
}

/* Class initialization */
static void sc_dev_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = sc_dev_realize;
    dc->unrealize = sc_dev_unrealize;
    device_class_set_props(dc, sc_dev_properties);
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

/* Define info about the device */
static const TypeInfo sc_dev_info = {
	.name = TYPE_SC_DEV,
	.parent = TYPE_SYS_BUS_DEVICE,
	.instance_size = sizeof(ScDevState),
	.instance_init = sc_dev_instance_init,
	.class_init = sc_dev_class_init,
};

/* Register type (as a static type) */
static void sc_dev_register_types(void)
{
    type_register_static(&sc_dev_info);
}

/* Initialize type */
type_init(sc_dev_register_types)


/* Public */
/* Create the SystemC device (can be called from the board initialization):
     Create object, realize it, map MMIO, return pointer
 */
DeviceState *sc_dev_create(hwaddr addr)
{
	DeviceState *dev = qdev_new(TYPE_SC_DEV);
	sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
	sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, addr);
	return dev;
}