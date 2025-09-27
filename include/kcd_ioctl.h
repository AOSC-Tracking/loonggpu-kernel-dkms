/*
 * Copyright (c) 2024 Loongson, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef KCD_IOCTL_H_INCLUDED
#define KCD_IOCTL_H_INCLUDED

#include <drm/drm.h>
#include <linux/ioctl.h>

/*
 * - 1.1 - initial version
 * - 1.3 - Add SMI events support
 * - 1.4 - Indicate new SRAM EDC bit in device properties
 * - 1.5 - Add SVM API
 * - 1.6 - Query clear flags in SVM get_attr API
 * - 1.9 - Add available memory ioctl
 * - 1.10 - Add SMI profiler event log
 * - 1.11 - Add unified memory for ctx save/restore area
 * - 1.12 - Add DMA buf export ioctl
 * - 1.14 - Update kcd_event_data
 */
#define KCD_IOCTL_MAJOR_VERSION 1
#define KCD_IOCTL_MINOR_VERSION 1

struct kcd_ioctl_get_version_args {
	__u32 major_version;	/* from KCD */
	__u32 minor_version;	/* from KCD */
};

/* For kcd_ioctl_create_queue_args.queue_type. */
#define KCD_IOC_QUEUE_TYPE_COMPUTE		0x0
#define KCD_IOC_QUEUE_TYPE_SDMA			0x1

#define KCD_MAX_QUEUE_PERCENTAGE	100
#define KCD_MAX_QUEUE_PRIORITY		15

struct kcd_ioctl_create_queue_args {
	__u64 ring_base_address;	/* to KCD */
	__u64 write_pointer_address;	/* from KCD */
	__u64 read_pointer_address;	/* from KCD */
	__u64 doorbell_offset;		/* from KFD */

	__u32 ring_size;		/* to KCD */
	__u32 gpu_id;		/* to KCD */
	__u32 queue_type;		/* to KCD */
	__u32 queue_percentage;	/* to KCD */
	__u32 queue_priority;	/* to KCD */
	__u32 queue_id;		/* from KCD */

	__u64 ctx_save_restore_address; /* to KFD */
	__u32 ctx_save_restore_size;	/* to KFD */
	__u32 pad;
};

struct kcd_ioctl_destroy_queue_args {
	__u32 queue_id;		/* to KCD */
	__u32 pad;
};

struct kcd_ioctl_update_queue_args {
	__u64 ring_base_address;	/* to KCD */

	__u32 queue_id;		/* to KCD */
	__u32 ring_size;		/* to KCD */
	__u32 queue_percentage;	/* to KCD */
	__u32 queue_priority;	/* to KCD */
};


/*
 * All counters are monotonic. They are used for profiling of compute jobs.
 * The profiling is done by userspace.
 *
 * In case of GPU reset, the counter should not be affected.
 */

struct kcd_ioctl_get_clock_counters_args {
	__u64 gpu_clock_counter;	/* from KCD */
	__u64 cpu_clock_counter;	/* from KCD */
	__u64 system_clock_counter;	/* from KCD */
	__u64 system_clock_freq;	/* from KCD */

	__u32 gpu_id;		/* to KCD */
	__u32 pad;
};

struct kcd_process_device_apertures {
	__u64 lds_base;		/* from KCD */
	__u64 lds_limit;		/* from KCD */
	__u64 scratch_base;		/* from KCD */
	__u64 scratch_limit;		/* from KCD */
	__u64 gpuvm_base;		/* from KCD */
	__u64 gpuvm_limit;		/* from KCD */
	__u32 gpu_id;		/* from KCD */
	__u32 pad;
};

/*
 * LGKCD_IOC_GET_PROCESS_APERTURES is deprecated. Use
 * LGKCD_IOC_GET_PROCESS_APERTURES_NEW instead, which supports an
 * unlimited number of GPUs.
 */
#define NUM_OF_SUPPORTED_GPUS 7
struct kcd_ioctl_get_process_apertures_args {
	struct kcd_process_device_apertures
			process_apertures[NUM_OF_SUPPORTED_GPUS];/* from KCD */

	/* from KCD, should be in the range [1 - NUM_OF_SUPPORTED_GPUS] */
	__u32 num_of_nodes;
	__u32 pad;
};

struct kcd_ioctl_get_process_apertures_new_args {
	/* User allocated. Pointer to struct kcd_process_device_apertures
	 * filled in by Kernel
	 */
	__u64 kcd_process_device_apertures_ptr;
	/* to KCD - indicates amount of memory present in
	 *  kcd_process_device_apertures_ptr
	 * from KCD - Number of entries filled by KCD.
	 */
	__u32 num_of_nodes;
	__u32 pad;
};

struct kcd_ioctl_submit_queue_args {
	__u32 queue_id;		/* to KCD */
	__u32 pad;
};

/* Matching VDD_EVENTTYPE */
#define KCD_IOC_EVENT_SIGNAL			0
#define KCD_IOC_EVENT_NODECHANGE		1
#define KCD_IOC_EVENT_DEVICESTATECHANGE		2
#define KCD_IOC_EVENT_HW_EXCEPTION		3
#define KCD_IOC_EVENT_SYSTEM_EVENT		4
#define KCD_IOC_EVENT_QUEUE_EVENT		7
#define KCD_IOC_EVENT_MEMORY			8

#define KCD_IOC_WAIT_RESULT_COMPLETE		0
#define KCD_IOC_WAIT_RESULT_TIMEOUT		1
#define KCD_IOC_WAIT_RESULT_FAIL		2

#define KCD_SIGNAL_EVENT_LIMIT			4096

/* For kcd_event_data.hw_exception_data.reset_type. */
#define KCD_HW_EXCEPTION_WHOLE_GPU_RESET	0
#define KCD_HW_EXCEPTION_PER_ENGINE_RESET	1

/* For kcd_event_data.hw_exception_data.reset_cause. */
#define KCD_HW_EXCEPTION_GPU_HANG	0

/* For kcd_vdd_memory_exception_data.ErrorType */
#define KCD_MEM_ERR_POISON_CONSUMED	2
#define KCD_MEM_ERR_GPU_HANG		3

struct kcd_ioctl_create_event_args {
	__u64 event_page_offset;	/* from KCD */
	__u32 event_trigger_data;	/* from KCD - signal events only */
	__u32 event_type;		/* to KCD */
	__u32 auto_reset;		/* to KCD */
	__u32 node_id;		/* to KCD - only valid for certain
							event types */
	__u32 event_id;		/* from KCD */
	__u32 event_slot_index;	/* from KCD */
};

struct kcd_ioctl_destroy_event_args {
	__u32 event_id;		/* to KCD */
	__u32 pad;
};

struct kcd_ioctl_set_event_args {
	__u32 event_id;		/* to KCD */
	__u32 pad;
};

struct kcd_ioctl_reset_event_args {
	__u32 event_id;		/* to KCD */
	__u32 pad;
};

struct kcd_memory_exception_failure {
	__u32 NotPresent;	/* Page not present or supervisor privilege */
	__u32 ReadOnly;	/* Write access to a read-only page */
	__u32 NoExecute;	/* Execute access to a page marked NX */
	__u32 imprecise;	/* Can't determine the	exact fault address */
};

/* memory exception data */
struct kcd_vdd_memory_exception_data {
	struct kcd_memory_exception_failure failure;
	__u64 va;
	__u32 gpu_id;
	__u32 ErrorType; /* 0 = no RAS error,
			  * 1 = ECC_SRAM,
			  * 2 = Link_SYNFLOOD (poison),
			  * 3 = GPU hang (not attributable to a specific cause),
			  * other values reserved
			  */
};

/* hw exception data */
struct kcd_vdd_hw_exception_data {
	__u32 reset_type;
	__u32 reset_cause;
	__u32 memory_lost;
	__u32 gpu_id;
};

/* vdd signal event data */
struct kcd_vdd_signal_event_data {
	__u64 last_event_age;	/* to and from KCD */
};

/* Event data */
struct kcd_event_data {
	union {
		/* From KCD */
		struct kcd_vdd_memory_exception_data memory_exception_data;
		struct kcd_vdd_hw_exception_data hw_exception_data;
		/* To and From KCD */
		struct kcd_vdd_signal_event_data signal_event_data;
	};
	__u64 kcd_event_data_ext;	/* pointer to an extension structure
					   for future exception types */
	__u32 event_id;		/* to KCD */
	__u32 pad;
};

struct kcd_ioctl_wait_events_args {
	__u64 events_ptr;		/* pointed to struct
					   kcd_event_data array, to KCD */
	__u32 num_events;		/* to KCD */
	__u32 wait_for_all;		/* to KCD */
	__u32 timeout;		/* to KCD */
	__u32 wait_result;		/* from KCD */
};

struct kcd_ioctl_acquire_vm_args {
	__u32 drm_fd;	/* to KCD */
	__u32 gpu_id;	/* to KCD */
};

/* Allocation flags: memory types */
#define KCD_IOC_ALLOC_MEM_FLAGS_VRAM		(1 << 0)
#define KCD_IOC_ALLOC_MEM_FLAGS_GTT		(1 << 1)
#define KCD_IOC_ALLOC_MEM_FLAGS_USERPTR		(1 << 2)
#define KCD_IOC_ALLOC_MEM_FLAGS_DOORBELL	(1 << 3)
/* Allocation flags: attributes/access options */
#define KCD_IOC_ALLOC_MEM_FLAGS_WRITABLE	(1 << 31)
#define KCD_IOC_ALLOC_MEM_FLAGS_EXECUTABLE	(1 << 30)
#define KCD_IOC_ALLOC_MEM_FLAGS_PUBLIC		(1 << 29)

/* Allocate memory for later SVM (shared virtual memory) mapping.
 *
 * @va_addr:     virtual address of the memory to be allocated
 *               all later mappings on all GPUs will use this address
 * @size:        size in bytes
 * @handle:      buffer handle returned to user mode, used to refer to
 *               this allocation for mapping, unmapping and freeing
 * @mmap_offset: for CPU-mapping the allocation by mmapping a render node
 *               for userptrs this is overloaded to specify the CPU address
 * @gpu_id:      device identifier
 * @flags:       memory type and attributes. See KCD_IOC_ALLOC_MEM_FLAGS above
 */
struct kcd_ioctl_alloc_memory_of_gpu_args {
	__u64 va_addr;		/* to KCD */
	__u64 size;		/* to KCD */
	__u64 handle;		/* from KCD */
	__u64 mmap_offset;	/* to KCD (userptr), from KCD (mmap offset) */
	__u32 gpu_id;		/* to KCD */
	__u32 flags;
};

/* Free memory allocated with kcd_ioctl_alloc_memory_of_gpu
 *
 * @handle: memory handle returned by alloc
 */
struct kcd_ioctl_free_memory_of_gpu_args {
	__u64 handle;		/* to KCD */
};

/* Inquire available memory with kcd_ioctl_get_available_memory
 *
 * @available: memory available for alloc
 */
struct  kcd_ioctl_get_available_memory_args {
	__u64 available;	/* from KCD */
	__u32 gpu_id;		/* to KCD */
	__u32 pad;
};

/* Map memory to one or more GPUs
 *
 * @handle:                memory handle returned by alloc
 * @device_ids_array_ptr:  array of gpu_ids (__u32 per device)
 * @n_devices:             number of devices in the array
 * @n_success:             number of devices mapped successfully
 *
 * @n_success returns information to the caller how many devices from
 * the start of the array have mapped the buffer successfully. It can
 * be passed into a subsequent retry call to skip those devices. For
 * the first call the caller should initialize it to 0.
 *
 * If the ioctl completes with return code 0 (success), n_success ==
 * n_devices.
 */
struct kcd_ioctl_map_memory_to_gpu_args {
	__u64 handle;			/* to KCD */
	__u64 device_ids_array_ptr;	/* to KCD */
	__u32 n_devices;		/* to KCD */
	__u32 n_success;		/* to/from KCD */
};

/* Unmap memory from one or more GPUs
 *
 * same arguments as for mapping
 */
struct kcd_ioctl_unmap_memory_from_gpu_args {
	__u64 handle;			/* to KCD */
	__u64 device_ids_array_ptr;	/* to KCD */
	__u32 n_devices;		/* to KCD */
	__u32 n_success;		/* to/from KCD */
};

struct kcd_ioctl_get_dmabuf_info_args {
	__u64 size;		/* from KCD */
	__u64 metadata_ptr;	/* to KCD */
	__u32 metadata_size;	/* to KCD (space allocated by user)
				 * from KCD (actual metadata size)
				 */
	__u32 gpu_id;	/* from KCD */
	__u32 flags;		/* from KCD (KCD_IOC_ALLOC_MEM_FLAGS) */
	__u32 dmabuf_fd;	/* to KCD */
};

struct kcd_ioctl_import_dmabuf_args {
	__u64 va_addr;	/* to KCD */
	__u64 handle;	/* from KCD */
	__u32 gpu_id;	/* to KCD */
	__u32 dmabuf_fd;	/* to KCD */
};

struct kcd_ioctl_export_dmabuf_args {
	__u64 handle;		/* to KCD */
	__u32 flags;		/* to KCD */
	__u32 dmabuf_fd;	/* from KCD */
};

/*
 * KCD SMI(System Management Interface) events
 */
enum kcd_smi_event {
	KCD_SMI_EVENT_NONE = 0, /* not used */
	KCD_SMI_EVENT_VMFAULT = 1, /* event start counting at 1 */
	KCD_SMI_EVENT_THERMAL_THROTTLE = 2,
	KCD_SMI_EVENT_GPU_PRE_RESET = 3,
	KCD_SMI_EVENT_GPU_POST_RESET = 4,
	KCD_SMI_EVENT_MIGRATE_START = 5,
	KCD_SMI_EVENT_MIGRATE_END = 6,
	KCD_SMI_EVENT_PAGE_FAULT_START = 7,
	KCD_SMI_EVENT_PAGE_FAULT_END = 8,
	KCD_SMI_EVENT_QUEUE_EVICTION = 9,
	KCD_SMI_EVENT_QUEUE_RESTORE = 10,
	KCD_SMI_EVENT_UNMAP_FROM_GPU = 11,

	/*
	 * max event number, as a flag bit to get events from all processes,
	 * this requires super user permission, otherwise will not be able to
	 * receive event from any process. Without this flag to receive events
	 * from same process.
	 */
	KCD_SMI_EVENT_ALL_PROCESS = 64
};

enum KCD_MIGRATE_TRIGGERS {
	KCD_MIGRATE_TRIGGER_PREFETCH,
	KCD_MIGRATE_TRIGGER_PAGEFAULT_GPU,
	KCD_MIGRATE_TRIGGER_PAGEFAULT_CPU,
	KCD_MIGRATE_TRIGGER_TTM_EVICTION
};

enum KCD_QUEUE_EVICTION_TRIGGERS {
	KCD_QUEUE_EVICTION_TRIGGER_SVM,
	KCD_QUEUE_EVICTION_TRIGGER_USERPTR,
	KCD_QUEUE_EVICTION_TRIGGER_TTM,
	KCD_QUEUE_EVICTION_TRIGGER_SUSPEND
};

enum KCD_SVM_UNMAP_TRIGGERS {
	KCD_SVM_UNMAP_TRIGGER_MMU_NOTIFY,
	KCD_SVM_UNMAP_TRIGGER_MMU_NOTIFY_MIGRATE,
	KCD_SVM_UNMAP_TRIGGER_UNMAP_FROM_CPU
};

#define KCD_SMI_EVENT_MASK_FROM_INDEX(i) (1ULL << ((i) - 1))
#define KCD_SMI_EVENT_MSG_SIZE	96

struct kcd_ioctl_smi_events_args {
	__u32 gpuid;	/* to KCD */
	__u32 anon_fd;	/* from KCD */
};

struct kcd_ioctl_ipc_export_handle_args {
	__u64 handle;		/* to KCD */
	__u32 share_handle[4];	/* from KCD */
	__u32 gpu_id;		/* to KCD */
	__u32 flags;		/* to KCD */
};

struct kcd_ioctl_ipc_import_handle_args {
	__u64 handle;		/* from KCD */
	__u64 va_addr;		/* to KCD */
	__u64 mmap_offset;	/* from KCD */
	__u32 share_handle[4];	/* to KCD */
	__u32 gpu_id;		/* to KCD */
	__u32 flags;		/* from KCD */
};

/* Guarantee host access to memory */
#define KCD_IOCTL_SVM_FLAG_HOST_ACCESS 0x00000001
/* Fine grained coherency between all devices with access */
#define KCD_IOCTL_SVM_FLAG_COHERENT    0x00000002
/* Reserved */
#define KCD_IOCTL_SVM_FLAG_RESERVED  0x00000004
/* GPUs only read, allows replication */
#define KCD_IOCTL_SVM_FLAG_GPU_RO      0x00000008
/* Allow execution on GPU */
#define KCD_IOCTL_SVM_FLAG_GPU_EXEC    0x00000010
/* GPUs mostly read, may allow similar optimizations as RO, but writes fault */
#define KCD_IOCTL_SVM_FLAG_GPU_READ_MOSTLY     0x00000020
/* Keep GPU memory mapping always valid as if XNACK is disable */
#define KCD_IOCTL_SVM_FLAG_GPU_ALWAYS_MAPPED   0x00000040
/* Fine grained coherency between all devices using device-scope atomics */
#define KCD_IOCTL_SVM_FLAG_EXT_COHERENT        0x00000080

/**
 * kcd_ioctl_svm_op - SVM ioctl operations
 *
 * @KCD_IOCTL_SVM_OP_SET_ATTR: Modify one or more attributes
 * @KCD_IOCTL_SVM_OP_GET_ATTR: Query one or more attributes
 */
enum kcd_ioctl_svm_op {
	KCD_IOCTL_SVM_OP_SET_ATTR,
	KCD_IOCTL_SVM_OP_GET_ATTR
};

/** kcd_ioctl_svm_location - Enum for preferred and prefetch locations
 *
 * GPU IDs are used to specify GPUs as preferred and prefetch locations.
 * Below definitions are used for system memory or for leaving the preferred
 * location unspecified.
 */
enum kcd_ioctl_svm_location {
	KCD_IOCTL_SVM_LOCATION_SYSMEM = 0,
	KCD_IOCTL_SVM_LOCATION_UNDEFINED = 0xffffffff
};

/**
 * kcd_ioctl_svm_attr_type - SVM attribute types
 *
 * @KCD_IOCTL_SVM_ATTR_PREFERRED_LOC: gpuid of the preferred location, 0 for
 *                                    system memory
 * @KCD_IOCTL_SVM_ATTR_PREFETCH_LOC: gpuid of the prefetch location, 0 for
 *                                   system memory. Setting this triggers an
 *                                   immediate prefetch (migration).
 * @KCD_IOCTL_SVM_ATTR_ACCESS:
 * @KCD_IOCTL_SVM_ATTR_ACCESS_IN_PLACE:
 * @KCD_IOCTL_SVM_ATTR_NO_ACCESS: specify memory access for the gpuid given
 *                                by the attribute value
 * @KCD_IOCTL_SVM_ATTR_SET_FLAGS: bitmask of flags to set (see
 *                                KCD_IOCTL_SVM_FLAG_...)
 * @KCD_IOCTL_SVM_ATTR_CLR_FLAGS: bitmask of flags to clear
 * @KCD_IOCTL_SVM_ATTR_GRANULARITY: migration granularity
 *                                  (log2 num pages)
 */
enum kcd_ioctl_svm_attr_type {
	KCD_IOCTL_SVM_ATTR_PREFERRED_LOC,
	KCD_IOCTL_SVM_ATTR_PREFETCH_LOC,
	KCD_IOCTL_SVM_ATTR_ACCESS,
	KCD_IOCTL_SVM_ATTR_ACCESS_IN_PLACE,
	KCD_IOCTL_SVM_ATTR_NO_ACCESS,
	KCD_IOCTL_SVM_ATTR_SET_FLAGS,
	KCD_IOCTL_SVM_ATTR_CLR_FLAGS,
	KCD_IOCTL_SVM_ATTR_GRANULARITY
};

/**
 * kcd_ioctl_svm_attribute - Attributes as pairs of type and value
 *
 * The meaning of the @value depends on the attribute type.
 *
 * @type: attribute type (see enum @kcd_ioctl_svm_attr_type)
 * @value: attribute value
 */
struct kcd_ioctl_svm_attribute {
	__u32 type;
	__u32 value;
};

/**
 * kcd_ioctl_svm_args - Arguments for SVM ioctl
 *
 * @op specifies the operation to perform (see enum
 * @kcd_ioctl_svm_op).  @start_addr and @size are common for all
 * operations.
 *
 * A variable number of attributes can be given in @attrs.
 * @nattr specifies the number of attributes. New attributes can be
 * added in the future without breaking the ABI. If unknown attributes
 * are given, the function returns -EINVAL.
 *
 * @KCD_IOCTL_SVM_OP_SET_ATTR sets attributes for a virtual address
 * range. It may overlap existing virtual address ranges. If it does,
 * the existing ranges will be split such that the attribute changes
 * only apply to the specified address range.
 *
 * @KCD_IOCTL_SVM_OP_GET_ATTR returns the intersection of attributes
 * over all memory in the given range and returns the result as the
 * attribute value. If different pages have different preferred or
 * prefetch locations, 0xffffffff will be returned for
 * @KCD_IOCTL_SVM_ATTR_PREFERRED_LOC or
 * @KCD_IOCTL_SVM_ATTR_PREFETCH_LOC resepctively. For
 * @KCD_IOCTL_SVM_ATTR_SET_FLAGS, flags of all pages will be
 * aggregated by bitwise AND. That means, a flag will be set in the
 * output, if that flag is set for all pages in the range. For
 * @KCD_IOCTL_SVM_ATTR_CLR_FLAGS, flags of all pages will be
 * aggregated by bitwise NOR. That means, a flag will be set in the
 * output, if that flag is clear for all pages in the range.
 * The minimum migration granularity throughout the range will be
 * returned for @KCD_IOCTL_SVM_ATTR_GRANULARITY.
 *
 * Querying of accessibility attributes works by initializing the
 * attribute type to @KCD_IOCTL_SVM_ATTR_ACCESS and the value to the
 * GPUID being queried. Multiple attributes can be given to allow
 * querying multiple GPUIDs. The ioctl function overwrites the
 * attribute type to indicate the access for the specified GPU.
 */
struct kcd_ioctl_svm_args {
	__u64 start_addr;
	__u64 size;
	__u32 op;
	__u32 nattr;
	/* Variable length array of attributes */
	struct kcd_ioctl_svm_attribute attrs[];
};

/* Queue status return for suspend/resume */
#define KCD_DBG_QUEUE_ERROR_BIT		30
#define KCD_DBG_QUEUE_INVALID_BIT	31
#define KCD_DBG_QUEUE_ERROR_MASK	(1 << KCD_DBG_QUEUE_ERROR_BIT)
#define KCD_DBG_QUEUE_INVALID_MASK	(1 << KCD_DBG_QUEUE_INVALID_BIT)

/**
 * kcd_ioctl_submit_queue_args
 *
 * Arguments to submit a queue.
 *
 *     @queue_id - the queue id to be submitted
 *
 */
#define LGKCD_IOCTL_BASE 'K'
#define LGKCD_IO(nr)			_IO(LGKCD_IOCTL_BASE, nr)
#define LGKCD_IOR(nr, type)		_IOR(LGKCD_IOCTL_BASE, nr, type)
#define LGKCD_IOW(nr, type)		_IOW(LGKCD_IOCTL_BASE, nr, type)
#define LGKCD_IOWR(nr, type)		_IOWR(LGKCD_IOCTL_BASE, nr, type)

#define LGKCD_IOC_GET_VERSION			\
		LGKCD_IOR(0x01, struct kcd_ioctl_get_version_args)

#define LGKCD_IOC_CREATE_QUEUE			\
		LGKCD_IOWR(0x02, struct kcd_ioctl_create_queue_args)

#define LGKCD_IOC_DESTROY_QUEUE		\
		LGKCD_IOWR(0x03, struct kcd_ioctl_destroy_queue_args)

#define LGKCD_IOC_GET_CLOCK_COUNTERS		\
		LGKCD_IOWR(0x05, struct kcd_ioctl_get_clock_counters_args)

#define LGKCD_IOC_GET_PROCESS_APERTURES	\
		LGKCD_IOR(0x06, struct kcd_ioctl_get_process_apertures_args)

#define LGKCD_IOC_UPDATE_QUEUE			\
		LGKCD_IOW(0x07, struct kcd_ioctl_update_queue_args)

#define LGKCD_IOC_CREATE_EVENT			\
		LGKCD_IOWR(0x08, struct kcd_ioctl_create_event_args)

#define LGKCD_IOC_DESTROY_EVENT		\
		LGKCD_IOW(0x09, struct kcd_ioctl_destroy_event_args)

#define LGKCD_IOC_SET_EVENT			\
		LGKCD_IOW(0x0A, struct kcd_ioctl_set_event_args)

#define LGKCD_IOC_RESET_EVENT			\
		LGKCD_IOW(0x0B, struct kcd_ioctl_reset_event_args)

#define LGKCD_IOC_WAIT_EVENTS			\
		LGKCD_IOWR(0x0C, struct kcd_ioctl_wait_events_args)

#define LGKCD_IOC_GET_PROCESS_APERTURES_NEW	\
		LGKCD_IOWR(0x14,		\
			struct kcd_ioctl_get_process_apertures_new_args)

#define LGKCD_IOC_ACQUIRE_VM			\
		LGKCD_IOW(0x15, struct kcd_ioctl_acquire_vm_args)

#define LGKCD_IOC_ALLOC_MEMORY_OF_GPU		\
		LGKCD_IOWR(0x16, struct kcd_ioctl_alloc_memory_of_gpu_args)

#define LGKCD_IOC_FREE_MEMORY_OF_GPU		\
		LGKCD_IOW(0x17, struct kcd_ioctl_free_memory_of_gpu_args)

#define LGKCD_IOC_MAP_MEMORY_TO_GPU		\
		LGKCD_IOWR(0x18, struct kcd_ioctl_map_memory_to_gpu_args)

#define LGKCD_IOC_UNMAP_MEMORY_FROM_GPU	\
		LGKCD_IOWR(0x19, struct kcd_ioctl_unmap_memory_from_gpu_args)

#define LGKCD_IOC_GET_DMABUF_INFO		\
		LGKCD_IOWR(0x1C, struct kcd_ioctl_get_dmabuf_info_args)

#define LGKCD_IOC_IMPORT_DMABUF		\
		LGKCD_IOWR(0x1D, struct kcd_ioctl_import_dmabuf_args)

#define LGKCD_IOC_SMI_EVENTS			\
		LGKCD_IOWR(0x1F, struct kcd_ioctl_smi_events_args)

#define LGKCD_IOC_SVM	LGKCD_IOWR(0x20, struct kcd_ioctl_svm_args)

#define LGKCD_IOC_AVAILABLE_MEMORY		\
		LGKCD_IOWR(0x21, struct kcd_ioctl_get_available_memory_args)

#define LGKCD_IOC_EXPORT_DMABUF		\
		LGKCD_IOWR(0x22, struct kcd_ioctl_export_dmabuf_args)

#define LGKCD_IOC_SUBMIT_QUEUE			\
		LGKCD_IOWR(0x25, struct kcd_ioctl_submit_queue_args)

#define LGKCD_COMMAND_START		0x01
#define LGKCD_COMMAND_END		0x26

/* non-upstream ioctls */
#define LGKCD_IOC_IPC_IMPORT_HANDLE                                    \
		LGKCD_IOWR(0x80, struct kcd_ioctl_ipc_import_handle_args)

#define LGKCD_IOC_IPC_EXPORT_HANDLE		\
		LGKCD_IOWR(0x81, struct kcd_ioctl_ipc_export_handle_args)

#define LGKCD_COMMAND_START_2		0x80
#define LGKCD_COMMAND_END_2		0x82

#endif
