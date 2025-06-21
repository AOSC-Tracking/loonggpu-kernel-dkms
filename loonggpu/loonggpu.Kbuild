###########################################################################
# Kbuild fragment for loonggpu.ko
###########################################################################

#
# Define Loongson_{SOURCES,OBJECTS}
#

include $(src)/loonggpu/loonggpu-sources.Kbuild
include $(src)/loonggpu-bridge/loonggpu-bridge.Kbuild
include $(src)/lgkcd/lgkcd.Kbuild

LOONGGPU_OBJECTS = $(patsubst %.c,%.o,$(LOONGGPU_SOURCES))
LOONGGPU_BRIDGE_OBJECTS = $(patsubst %.c,%.o,$(LOONGGPU_BRIDGE_SOURCES))
LGKCD_OBJECTS = $(patsubst %.c,%.o,$(LGKCD_SOURCES))

obj-m += loonggpu.o
loonggpu-y := $(LOONGGPU_OBJECTS)
loonggpu-y += $(LOONGGPU_BRIDGE_OBJECTS)
loonggpu-y += $(LGKCD_OBJECTS)

LOONGGPU_KO = loonggpu/loonggpu.ko

#
# Define loonggpu.ko-specific CFLAGS.
#

LOONGGPU_CFLAGS += -I$(src)/loonggpu
LOONGGPU_CFLAGS += -DLOONGGPU_UNDEF_LEGACY_BIT_MACROS

ifeq ($(LG_BUILD_TYPE),release)
 LOONGGPU_CFLAGS += -UDEBUG -U_DEBUG -DNDEBUG
endif

ifeq ($(LG_BUILD_TYPE),develop)
 LOONGGPU_CFLAGS += -UDEBUG -U_DEBUG -DNDEBUG -DNV_MEM_LOGGER
endif

ifeq ($(LG_BUILD_TYPE),debug)
 LOONGGPU_CFLAGS += -DDEBUG -D_DEBUG -UNDEBUG -DNV_MEM_LOGGER
endif

$(call ASSIGN_PER_OBJ_CFLAGS, $(LOONGGPU_OBJECTS), $(LOONGGPU_CFLAGS))


# Linux kernel v5.12 and later looks at "always-y", Linux kernel versions
# before v5.6 looks at "always"; kernel versions between v5.12 and v5.6
# look at both.

always += $(LOONGGPU_INTERFACE)
always-y += $(LOONGGPU_INTERFACE)

$(obj)/$(LOONGGPU_INTERFACE): $(addprefix $(obj)/,$(LOONGGPU_OBJECTS))
	$(LD) -r -o $@ $^


#
# Register the conftests needed by loonggpu.ko
#
LG_OBJECTS_DEPEND_ON_CONFTEST += $(LOONGGPU_OBJECTS)
LG_OBJECTS_DEPEND_ON_CONFTEST += $(LOONGGPU_BRIDGE_OBJECTS)
LG_OBJECTS_DEPEND_ON_CONFTEST += $(LGKCD_OBJECTS)

LG_CONFTEST_TYPE_COMPILE_TESTS += mm_struct_has_mmap_lock
LG_CONFTEST_TYPE_COMPILE_TESTS += drm_driver_prime_flag_present
LG_CONFTEST_TYPE_COMPILE_TESTS += drm_driver_irq_shared_flag_present

LG_CONFTEST_FUNCTION_COMPILE_TESTS += drm_sched_stop
LG_CONFTEST_FUNCTION_COMPILE_TESTS += drm_sched_job_cleanup
LG_CONFTEST_FUNCTION_COMPILE_TESTS += drm_fb_helper_fill_info
LG_CONFTEST_FUNCTION_COMPILE_TESTS += drm_need_swiotlb
LG_CONFTEST_FUNCTION_COMPILE_TESTS += mmu_notifier_put
LG_CONFTEST_FUNCTION_COMPILE_TESTS += drm_gem_object_put
LG_CONFTEST_FUNCTION_COMPILE_TESTS += i2c_new_client_device
LG_CONFTEST_FUNCTION_COMPILE_TESTS += ttm_fbdev_mmap
LG_CONFTEST_FUNCTION_COMPILE_TESTS += ttm_bo_init_mm
LG_CONFTEST_FUNCTION_COMPILE_TESTS += ttm_bo_clean_mm
LG_CONFTEST_FUNCTION_COMPILE_TESTS += ttm_resource_alloc
LG_CONFTEST_FUNCTION_COMPILE_TESTS += ttm_bo_pipeline_move
LG_CONFTEST_FUNCTION_COMPILE_TESTS += ttm_tt_bind
LG_CONFTEST_FUNCTION_COMPILE_TESTS += ttm_bo_mem_put
LG_CONFTEST_FUNCTION_COMPILE_TESTS += ttm_tt_set_populated
LG_CONFTEST_FUNCTION_COMPILE_TESTS += ttm_resource_manager_evict_all
LG_CONFTEST_FUNCTION_COMPILE_TESTS += dma_resv_wait_timeout
LG_CONFTEST_FUNCTION_COMPILE_TESTS += drm_sched_job_arm
LG_CONFTEST_FUNCTION_COMPILE_TESTS += ttm_bo_lock_delayed_workqueue
LG_CONFTEST_FUNCTION_COMPILE_TESTS += drm_fb_helper_alloc_info
LG_CONFTEST_FUNCTION_COMPILE_TESTS += pwm_free
LG_CONFTEST_FUNCTION_COMPILE_TESTS += ttm_bo_mmap
LG_CONFTEST_FUNCTION_COMPILE_TESTS += drm_prime_sg_to_dma_addr_array
LG_CONFTEST_FUNCTION_COMPILE_TESTS += pci_map_page
LG_CONFTEST_FUNCTION_COMPILE_TESTS += ttm_bo_dma_acc_size
LG_CONFTEST_FUNCTION_COMPILE_TESTS += dma_resv_reserve_fences
LG_CONFTEST_FUNCTION_COMPILE_TESTS += dma_resv_add_fence
LG_CONFTEST_FUNCTION_COMPILE_TESTS += ttm_bo_evict_mm
LG_CONFTEST_FUNCTION_COMPILE_TESTS += ttm_tt_destroy_common
LG_CONFTEST_FUNCTION_COMPILE_TESTS += ttm_dma_tt_fini
LG_CONFTEST_FUNCTION_COMPILE_TESTS += swiotlb_nr_tbl
LG_CONFTEST_FUNCTION_COMPILE_TESTS += drm_sched_dependency_optimized
LG_CONFTEST_FUNCTION_COMPILE_TESTS += prime_handle_to_fd
LG_CONFTEST_FUNCTION_COMPILE_TESTS += pwm_apply_state
LG_CONFTEST_FUNCTION_COMPILE_TESTS += mmu_notifier_unregister_no_release
LG_CONFTEST_FUNCTION_COMPILE_TESTS += mmu_notifier_unregister

LG_CONFTEST_TYPE_COMPILE_TESTS += ttm_bo_device_init
LG_CONFTEST_TYPE_COMPILE_TESTS += ttm_eu_reserve_buffers
LG_CONFTEST_TYPE_COMPILE_TESTS += drm_syncobj_find_fence
LG_CONFTEST_TYPE_COMPILE_TESTS += ttm_bo_move_to_lru_tail_has_bulk_arg
LG_CONFTEST_TYPE_COMPILE_TESTS += mmu_notifier_ops_invalidate_range_start_has_range_arg
LG_CONFTEST_TYPE_COMPILE_TESTS += drm_hdmi_avi_infoframe_from_display_mode
LG_CONFTEST_TYPE_COMPILE_TESTS += drm_hdmi_avi_infoframe_from_display_mode_const_conn
LG_CONFTEST_TYPE_COMPILE_TESTS += ttm_buffer_object_has_base
LG_CONFTEST_TYPE_COMPILE_TESTS += drm_driver_has_gem_prime_res_obj
LG_CONFTEST_TYPE_COMPILE_TESTS += ttm_validate_buffer_has_num_shared
LG_CONFTEST_TYPE_COMPILE_TESTS += drm_crtc_state_has_async_flip
LG_CONFTEST_TYPE_COMPILE_TESTS += drm_driver_gem_prime_export_has_dev_arg
LG_CONFTEST_TYPE_COMPILE_TESTS += loongson_screen_state
LG_CONFTEST_TYPE_COMPILE_TESTS += drm_connector_for_each_possible_encoder
LG_CONFTEST_TYPE_COMPILE_TESTS += drm_bridge_attach
LG_CONFTEST_TYPE_COMPILE_TESTS += drm_sched_priority
LG_CONFTEST_TYPE_COMPILE_TESTS += drm_sched_priority_has_min
LG_CONFTEST_TYPE_COMPILE_TESTS += drm_sched_entity_init
LG_CONFTEST_TYPE_COMPILE_TESTS += drm_crtc_helper_funcs
LG_CONFTEST_TYPE_COMPILE_TESTS += drm_driver_has_get_vblank_timestamp
LG_CONFTEST_TYPE_COMPILE_TESTS += drm_fb_helper_init_has_max_conn_count
LG_CONFTEST_TYPE_COMPILE_TESTS += drm_prime_pages_to_sg
LG_CONFTEST_TYPE_COMPILE_TESTS += ttm_resource_manager
LG_CONFTEST_TYPE_COMPILE_TESTS += ttm_bus_placement_has_size
LG_CONFTEST_TYPE_COMPILE_TESTS += get_user_pages_remote_has_tsk
LG_CONFTEST_TYPE_COMPILE_TESTS += ttm_buffer_obj_has_offset
LG_CONFTEST_TYPE_COMPILE_TESTS += ttm_backend_func
LG_CONFTEST_TYPE_COMPILE_TESTS += ttm_tt_populate_has_bdev
LG_CONFTEST_TYPE_COMPILE_TESTS += ttm_tt_has_state
LG_CONFTEST_TYPE_COMPILE_TESTS += ttm_bo_driver_has_invalidate
LG_CONFTEST_TYPE_COMPILE_TESTS += ttm_bo_driver_has_ttm_tt_bind
LG_CONFTEST_TYPE_COMPILE_TESTS += ttm_bo_device_has_glob
LG_CONFTEST_TYPE_COMPILE_TESTS += ttm_bo_device_init_has_man
LG_CONFTEST_TYPE_COMPILE_TESTS += drm_device_open_count
LG_CONFTEST_TYPE_COMPILE_TESTS += loongson_system_configuration_has_vgabios_addr
LG_CONFTEST_TYPE_COMPILE_TESTS += ttm_resource_manager_init_has_bdev
LG_CONFTEST_TYPE_COMPILE_TESTS += drm_sched_backend_ops_timedout_job_ret_sched_stat
LG_CONFTEST_TYPE_COMPILE_TESTS += drm_sched_entity_push_job
LG_CONFTEST_TYPE_COMPILE_TESTS += drm_sched_backend_ops_has_prepare
LG_CONFTEST_TYPE_COMPILE_TESTS += dma_resv_usage_bookkeep
LG_CONFTEST_TYPE_COMPILE_TESTS += drm_plane_atomic_check_has_atomic_state_arg
LG_CONFTEST_TYPE_COMPILE_TESTS += vga_client_register
LG_CONFTEST_TYPE_COMPILE_TESTS += fb_info_has_apertures
LG_CONFTEST_TYPE_COMPILE_TESTS += drm_fb_helper_prepare
LG_CONFTEST_TYPE_COMPILE_TESTS += drm_mode_config_has_fb_base
LG_CONFTEST_TYPE_COMPILE_TESTS += pci_set_dma_mask
LG_CONFTEST_TYPE_COMPILE_TESTS += ttm_device_has_lru_lock
LG_CONFTEST_TYPE_COMPILE_TESTS += dma_resv_get_excl
LG_CONFTEST_TYPE_COMPILE_TESTS += drm_plane_helper_destroy
LG_CONFTEST_TYPE_COMPILE_TESTS += ttm_range_mgr_node
LG_CONFTEST_TYPE_COMPILE_TESTS += ttm_resource_manager_func_alloc_has_res_arg
LG_CONFTEST_TYPE_COMPILE_TESTS += bo_pin_count
LG_CONFTEST_TYPE_COMPILE_TESTS += get_user_pages_remote_has_vmas
LG_CONFTEST_TYPE_COMPILE_TESTS += get_user_pages_has_vmas
LG_CONFTEST_TYPE_COMPILE_TESTS += ttm_sg_tt_init_has_caching
LG_CONFTEST_TYPE_COMPILE_TESTS += drm_sched_init_has_device
LG_CONFTEST_TYPE_COMPILE_TESTS += ttm_bo_init_reserved_has_size
LG_CONFTEST_TYPE_COMPILE_TESTS += ttm_global
LG_CONFTEST_TYPE_COMPILE_TESTS += ttm_resource_free_double_ptr
LG_CONFTEST_TYPE_COMPILE_TESTS += ttm_dma_tt
LG_CONFTEST_TYPE_COMPILE_TESTS += drm_sched_priority_has_unset
LG_CONFTEST_TYPE_COMPILE_TESTS += drm_driver_has_gem_free
LG_CONFTEST_TYPE_COMPILE_TESTS += vgacon_text_force
LG_CONFTEST_TYPE_COMPILE_TESTS += ttm_resource_has_num_pages
LG_CONFTEST_TYPE_COMPILE_TESTS += ttm_bo_mem_space
LG_CONFTEST_TYPE_COMPILE_TESTS += ttm_buffer_object_has_resv
LG_CONFTEST_TYPE_COMPILE_TESTS += drm_device_has_pdev
LG_CONFTEST_TYPE_COMPILE_TESTS += ttm_buffer_object_has_mem
LG_CONFTEST_TYPE_COMPILE_TESTS += drm_driver_has_gem_prime_export
LG_CONFTEST_TYPE_COMPILE_TESTS += dma_buf_ops_has_map
LG_CONFTEST_TYPE_COMPILE_TESTS += dma_resv_get_list
LG_CONFTEST_TYPE_COMPILE_TESTS += drm_gem_dmabuf_kmap
LG_CONFTEST_TYPE_COMPILE_TESTS += drm_driver_syncobj_timeline_present
LG_CONFTEST_TYPE_COMPILE_TESTS += drm_sched_init_has_device_rq
LG_CONFTEST_TYPE_COMPILE_TESTS += drm_sched_init_has_device_rq_submit_wq
LG_CONFTEST_TYPE_COMPILE_TESTS += drm_sched_job_init_has_credits
LG_CONFTEST_TYPE_COMPILE_TESTS += drm_buddy_free_list_has_flags
LG_CONFTEST_TYPE_COMPILE_TESTS += drm_buddy_block_trim_has_start
LG_CONFTEST_TYPE_COMPILE_TESTS += drm_mode_config_funcs_has_output_poll_changed
LG_CONFTEST_TYPE_COMPILE_TESTS += drm_driver_has_lastclose
LG_CONFTEST_TYPE_COMPILE_TESTS += drm_fb_helper_single_add_all_connectors
LG_CONFTEST_TYPE_COMPILE_TESTS += pwm_request
LG_CONFTEST_TYPE_COMPILE_TESTS += atomic_check
LG_CONFTEST_TYPE_COMPILE_TESTS += ttm_placement_has_busy_placement
LG_CONFTEST_TYPE_COMPILE_TESTS += ttm_bo_populate
LG_CONFTEST_TYPE_COMPILE_TESTS += zone_managed_pages
LG_CONFTEST_TYPE_COMPILE_TESTS += sysfs_emit
LG_CONFTEST_TYPE_COMPILE_TESTS += kobj_type_has_attr_group
LG_CONFTEST_TYPE_COMPILE_TESTS += use_mm
LG_CONFTEST_TYPE_COMPILE_TESTS += vm_flags_set
LG_CONFTEST_TYPE_COMPILE_TESTS += dma_resv_replace_fences
LG_CONFTEST_TYPE_COMPILE_TESTS += mode_config_has_fb_modifiers
LG_CONFTEST_TYPE_COMPILE_TESTS += mmu_notifier_ops_has_free_notifier
LG_CONFTEST_TYPE_COMPILE_TESTS += pwm_add_table
LG_CONFTEST_TYPE_COMPILE_TESTS += drm_sched_init_has_struct_arg
