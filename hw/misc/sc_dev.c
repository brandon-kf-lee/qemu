
// TODO: hw/riscv/virt.c     [VIRT_SC_DEV] =       {  0x8000000,         0x100 },
// MMIO callbacks to SystemC device

#include <sys/socket.h>
#include <sys/un.h>

#include "qemu/osdep.h"
#include "qapi/error.h" 		 /* Provides error_fatal() handler */
#include "qemu/log.h"			 /* Error logs */
#include "hw/sysbus.h"  		 /* Provides all sysbus registering functions */
#include "hw/qdev-properties.h"  /* QEMU device properties */
#include "system/dma.h"          /* DMA functions */

#include "hw/misc/sc_dev.h"

#define TYPE_SC_DEV "sc_dev"
typedef struct ScDevState ScDevState;
DECLARE_INSTANCE_CHECKER(ScDevState, SC_DEV, TYPE_SC_DEV)

#define REG_ID 	0x0
#define CHIP_ID	0xBA000001

/* DMA */
#define DMA_BUFFER_SIZE 4096   // 4KB

#define DMA_CONTROL   0x20
#define DMA_SRC_LO    0x24
#define DMA_SRC_HI    0x28
#define DMA_DST       0x2C
#define DMA_LEN       0x30 
#define DMA_STAT      0x34 

#define DMA_START     (1 << 0)
#define DMA_BUSY      (1 << 0)
#define DMA_DONE      (1 << 1)
#define DMA_ERR       (1 << 2)

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

/* Device state structure 
   Keep track of the status of the device in this board. Specific registers & IRQs may be defined here 
 */
struct ScDevState {
	SysBusDevice parent_obj;
	MemoryRegion iomem;  // Memory mapped space for the device
    AddressSpace *dma_as; // DMA address space
	
    uint64_t chip_id;

    /* DMA registers */
    uint64_t dma_src;
    uint64_t dma_dst;
    uint32_t dma_len;
    uint32_t dma_ctrl;
    uint32_t dma_stat;

	/* Socket connection to external device */
    int sc_sock;
    char *socket_path;
};

struct bridge_msg {
    uint8_t  is_write;
    uint8_t  is_dma;
    uint16_t reserved; /* Ensure word alignment */

    uint64_t addr;
    uint32_t size;

    uint8_t  data[DMA_BUFFER_SIZE];

    int8_t status;
} __attribute__((packed));  /* Ensure no padding */


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
        .status = 0,  /* Code for incomplete message */
    };    
    struct bridge_msg response;
    ssize_t ret;

    /* Copy data buffer into message if writing */
    if (is_write && buf) {
        memcpy(msg.data, buf, size);
    }

	/* Ensure bridge socket connection */
    if (state->sc_sock < 0) {
        qemu_log_mask(LOG_GUEST_ERROR, "sc_dev: socket not connected\n");
        return -1;
    }

    /* Send request, error if message wasn't sent */
    ret = send(state->sc_sock, &msg, sizeof(msg), 0);
    if (ret != sizeof(msg)) {
        qemu_log_mask(LOG_GUEST_ERROR, "sc_dev: failed to send TLM message\n");
        return -1;
    }

    /* Receive response, error if nothing was received */
    ret = recv(state->sc_sock, &response, sizeof(response), MSG_WAITALL);
    if (ret != sizeof(response)) {
        qemu_log_mask(LOG_GUEST_ERROR, "sc_dev: failed to receive TLM response\n");
        return -1;
    }
    
    /* Copy response data back to caller's buffer (for reads) */
    if (!is_write && buf) {
        memcpy((void*)buf, response.data, size);
    }
    
    return response.status;
}

/* Memory read callback */
static uint64_t sc_dev_read(void *opaque, hwaddr addr, unsigned int size)
{
    ScDevState *state = SC_DEV(opaque);

    switch (addr) {
        //case REG_ID:
        //    return state->chip_id;

        case DMA_STAT:
            return state->dma_stat;

        default:
            /* Forward transaction to SystemC and receive response */
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
            state->dma_dst = data;
            break;

        case DMA_LEN:
            state->dma_len = data;
            break;

        /* Control registers (Write but not DMA) */
        default:
            uint8_t buf[8] = {0};  // Buffer for write data
            int8_t ret;            // TLM transaction status

            memcpy(buf, &data, size);
            
            if ((ret = sc_dev_tlm_transaction(state, 1, 0, addr, buf, size)) < 0) {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "sc_dev: TLM write error at 0x%lx: %d\n",
                              (unsigned long)addr, ret);
            }
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
    uint8_t buf[DMA_BUFFER_SIZE];    // Temporary storage for data read from memory
    uint64_t addr = state->dma_src;  // Guest address to read
    uint64_t dest = state->dma_dst;  // Device address to write into
    uint32_t left = state->dma_len;  // Total length of read
    uint32_t chunk = 0;              // Chunk size a single TLM transaction
    MemTxResult result;              // Result of DMA memory read

    /* TODO: Test to make sure DMA doesn't run if not all three registers are populated */
    if (!addr || !dest || !left) {
        qemu_log_mask(LOG_GUEST_ERROR, "dma_src, dma_dst, or dma_len not set.\n");
        state->dma_stat = DMA_ERR;
        return;
    }    
    
    state->dma_stat = DMA_BUSY;

    while (left) {
        chunk = MIN(left, DMA_BUFFER_SIZE);

        /* Read guest RAM */
        result = dma_memory_read(state->dma_as, addr, buf, chunk, MEMTXATTRS_UNSPECIFIED);

        if (result != MEMTX_OK) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "sc_dev: DMA read failed at 0x%lx: %d\n",
                          (unsigned long)addr, result);
            state->dma_stat = DMA_ERR;
            return;
        }

        /* Send to SystemC the chunked buffer data */
        if (sc_dev_tlm_transaction(state, 1, 1, dest, buf, chunk) < 0) {
            state->dma_stat = DMA_ERR;
            return;
        }

        /* Increment addresses and decrement remaining chunks */
        addr += chunk;
        dest += chunk;
        left -= chunk;
    }

    state->dma_stat = DMA_DONE;
    
    /* Zero out DMA registers when finished */
    state->dma_src = 0;  
    state->dma_dst = 0;  
    state->dma_len = 0;  
}


/* File operations associated with the device
   “E.g. When CPU reads this memory region, call sc_dev_read()”
*/
static const MemoryRegionOps sc_dev_ops = {
	.read = sc_dev_read,
	.write = sc_dev_write,
	.endianness = DEVICE_NATIVE_ENDIAN,
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
    
    qemu_log("sc_dev: DMA intialized");
}

/* Device unrealization (socket cleanup) */
static void sc_dev_unrealize(DeviceState *dev)
{
    ScDevState *state = SC_DEV(dev);

    if (state->sc_sock >= 0) {
        close(state->sc_sock);
        state->sc_sock = -1;
    }
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

	state->chip_id = CHIP_ID;
	state->sc_sock = -1;  /* Mark as disconnected initially */
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