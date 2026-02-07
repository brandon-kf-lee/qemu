
// TODO: hw/riscv/virt.c     [VIRT_SC_DEV] =       {  0x8000000,         0x100 },
// MMIO callbacks to SystemC device

#include <sys/socket.h>
#include <sys/un.h>

#include "qemu/osdep.h"
#include "qapi/error.h" /* Provides error_fatal() handler */
#include "hw/sysbus.h"  /* Provides all sysbus registering func */
#include "hw/misc/sc_dev.h"

#define TYPE_SC_DEV "sc_dev"
typedef struct ScDevState ScDevState;
DECLARE_INSTANCE_CHECKER(ScDevState, SC_DEV, TYPE_SC_DEV)

#define REG_ID 	0x0
#define CHIP_ID	0xBA000001

/* Forward declarations */
static void sc_dev_instance_init(Object *obj);
static uint64_t sc_dev_read(void *opaque, hwaddr addr, unsigned int size);


/* Keep track of the status of the device in this board. Specific registers & IRQs may be defined here */
struct ScDevState {
	SysBusDevice parent_obj;
	MemoryRegion iomem;  // Memory mapped space for the device
	uint64_t chip_id;
};

/* Minimal file operations associated to the device (TODO: Add write functionality) 
   “When CPU reads this memory region, call sc_dev_read()”
*/
static const MemoryRegionOps sc_dev_ops = {
	.read = sc_dev_read,
	.endianness = DEVICE_NATIVE_ENDIAN,
};

/* Memory read callback */
static uint64_t sc_dev_read(void *opaque, hwaddr addr, unsigned int size)
{
	ScDevState *state = opaque;

	switch (addr) {
	    case REG_ID:
	    	return state->chip_id;

	    default:
	    	return 0xDEADBEEF;
	}

	return 0;
}

/* Initialize device state using memory_region_init_io by allocating:
     its memory (memory map region), 
     file operations associated to it, and
     its size.
   Let the system bus know that this memory region is handled by this device by using sysbus_init_mmio.
*/
static void sc_dev_instance_init(Object *obj)
{
	ScDevState *state = SC_DEV(obj);

    memory_region_init_io(&state->iomem, obj, &sc_dev_ops, state, TYPE_SC_DEV, 0x100); // TODO: May need to change 0x100 to something else
	sysbus_init_mmio(SYS_BUS_DEVICE(obj), &state->iomem); 

	state->chip_id = CHIP_ID;
}

/* Create a new type to define info for the device */
static const TypeInfo sc_dev_info = {
	.name = TYPE_SC_DEV,
	.parent = TYPE_SYS_BUS_DEVICE,
	.instance_size = sizeof(ScDevState),
	.instance_init = sc_dev_instance_init,
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





// struct tlm_msg {
//     uint8_t  is_write;
//     uint64_t addr;
//     uint32_t size;
//     uint64_t data;
// };

// static int sc_sock;

// // Connect to the SystemC socket on host machine
// static void mydev_realize(DeviceState *dev, Error **errp)
// {
//     struct sockaddr_un addr = {0};

//     sc_sock = socket(AF_UNIX, SOCK_STREAM, 0);
//     addr.sun_family = AF_UNIX;
//     strcpy(addr.sun_path, "/tmp/systemc.sock");

//     if (connect(sc_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
//         error_report("Cannot connect to SystemC");
//         exit(1);
//     }
// }

// static uint64_t dev_mmio_read(void *opaque, hwaddr addr, unsigned size)
// {
//     struct tlm_msg msg = {0, addr, size, 0};
//     send(sc_sock, &msg, sizeof(msg), 0);
//     recv(sc_sock, &msg, sizeof(msg), 0);
//     return msg.data;
// }

// static void dev_mmio_write(void *opaque, hwaddr addr,
//                              uint64_t value, unsigned size)
// {
//     struct tlm_msg msg = {1, addr, size, value};
//     send(sc_sock, &msg, sizeof(msg), 0);
//     recv(sc_sock, &msg, sizeof(msg), 0);
// }