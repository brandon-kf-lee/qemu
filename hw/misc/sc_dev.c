
// TODO: hw/riscv/virt.c     [VIRT_SC_DEV] =       {  0x8000000,         0x100 },
// MMIO callbacks to SystemC device

#include <sys/socket.h>
#include <sys/un.h>

#include "qemu/osdep.h"
#include "qapi/error.h" 		 /* Provides error_fatal() handler */
#include "qemu/log.h"			 /* Error logs */
#include "hw/sysbus.h"  		 /* Provides all sysbus registering functions */
#include "hw/qdev-properties.h"  /* QEMU device properties */

#include "hw/misc/sc_dev.h"

#define TYPE_SC_DEV "sc_dev"
typedef struct ScDevState ScDevState;
DECLARE_INSTANCE_CHECKER(ScDevState, SC_DEV, TYPE_SC_DEV)

#define REG_ID 	0x0
#define CHIP_ID	0xBA000001

/* Forward declarations */
static void sc_dev_instance_init(Object *obj);
static void sc_dev_realize(DeviceState *dev, Error **errp);
static void sc_dev_unrealize(DeviceState *dev);
static uint64_t sc_dev_read(void *opaque, hwaddr addr, unsigned int size);
static void sc_dev_write(void *opaque, hwaddr addr, uint64_t data, unsigned int size);

/* Device state structure 
   Keep track of the status of the device in this board. Specific registers & IRQs may be defined here 
 */
struct ScDevState {
	SysBusDevice parent_obj;
	MemoryRegion iomem;  // Memory mapped space for the device
	uint64_t chip_id;

	/* Socket connection to SystemC */
    int sc_sock;
    char *socket_path;
};

/* Stucture of a TLM message */
struct tlm_msg {
    uint8_t  is_write;
    uint64_t addr;
    uint32_t size;
    uint64_t data;
} __attribute__((packed));  /* Ensure no padding */


/* Helper: Send and receive TLM transactions */
static uint64_t sc_dev_tlm_transaction(ScDevState *state, 
									   uint8_t is_write, uint64_t addr, uint32_t size, uint64_t data)
{
    struct tlm_msg msg = {
        .is_write = is_write,
        .addr = addr,
        .size = size,
        .data = data,
    };
    struct tlm_msg response;
    ssize_t ret;

	/* Ensure connection */
    if (state->sc_sock < 0) {
        qemu_log_mask(LOG_GUEST_ERROR, "sc_dev: socket not connected\n");
        return 0xDEADBEEF;
    }

    /* Send request */
    ret = send(state->sc_sock, &msg, sizeof(msg), 0);
    if (ret != sizeof(msg)) {
        qemu_log_mask(LOG_GUEST_ERROR, "sc_dev: failed to send TLM message\n");
        return 0xDEADBEEF;
    }

    /* Receive response */
    ret = recv(state->sc_sock, &response, sizeof(response), MSG_WAITALL);
    if (ret != sizeof(response)) {
        qemu_log_mask(LOG_GUEST_ERROR, "sc_dev: failed to receive TLM response\n");
        return 0xDEADBEEF;
    }

    return response.data;
}

/* Memory read callback */
static uint64_t sc_dev_read(void *opaque, hwaddr addr, unsigned int size)
{
    ScDevState *state = SC_DEV(opaque);

    switch (addr) {
        case REG_ID:
            return state->chip_id;

        default:
            /* Forward transaction to SystemC */
            return sc_dev_tlm_transaction(state, 0, addr, size, 0);
    }
}

/* Memory write callback */
static void sc_dev_write(void *opaque, hwaddr addr, uint64_t data, unsigned int size)
{
    ScDevState *state = SC_DEV(opaque);

    /* Forward all writes to SystemC */
    sc_dev_tlm_transaction(state, 1, addr, size, data);
}


/* File operations associated with the device
   “Eg. When CPU reads this memory region, call sc_dev_read()”
*/
static const MemoryRegionOps sc_dev_ops = {
	.read = sc_dev_read,
	.write = sc_dev_write,
	.endianness = DEVICE_NATIVE_ENDIAN,
};

/* Device realization. Socket is connected here 
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
    //DEFINE_PROP_END_OF_LIST(),
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