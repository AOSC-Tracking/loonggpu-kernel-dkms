###########################################################################
# Kbuild file for Loongson Linux GPU driver kernel modules
###########################################################################

#
# The parent makefile is expected to define:
#
# LG_KERNEL_SOURCES : The root of the kernel source tree.
# LG_KERNEL_OUTPUT : The kernel's output tree.
# LG_KERNEL_MODULES : A whitespace-separated list of modules to build.
# ARCH : The target CPU architecture: x86_64|arm64|powerpc
#
# Kbuild provides the variables:
#
# $(src) : The directory containing this Kbuild file.
# $(obj) : The directory where the output from this build is written.
#

LG_BUILD_TYPE ?= release

#
# Utility macro ASSIGN_PER_OBJ_CFLAGS: to control CFLAGS on a
# per-object basis, Kbuild honors the 'CFLAGS_$(object)' variable.
# E.g., "CFLAGS_lg.o" for CFLAGS that are specific to lg.o. Use this
# macro to assign 'CFLAGS_$(object)' variables for multiple object
# files.
#
# $(1): The object files.
# $(2): The CFLAGS to add for those object files.
#
# With kernel git commit 54b8ae66ae1a3454a7645d159a482c31cd89ab33, the
# handling of object-specific CFLAGs, CFLAGS_$(object) has changed. Prior to
# this commit, the CFLAGS_$(object) variable was required to be defined with
# only the the object name (<CFLAGS_somefile.o>). With the aforementioned git
# commit, it is now required to give Kbuild relative paths along-with the
# object name (CFLAGS_<somepath>/somefile.o>). As a result, CFLAGS_$(object)
# is set twice, once with a relative path to the object files and once with
# just the object files.
#
ASSIGN_PER_OBJ_CFLAGS = \
 $(foreach _cflags_variable, \
 $(notdir $(1)) $(1), \
 $(eval $(addprefix CFLAGS_,$(_cflags_variable)) += $(2)))


#
# Include the specifics of the individual Loongson kernel modules.
#
# Each of these should:
# - Append to 'obj-m', to indicate the kernel module that should be built.
# - Define the object files that should get built to produce the kernel module.
# - Tie into conftest (see the description below).
#

LG_UNDEF_BEHAVIOR_SANITIZER ?=
ifeq ($(LG_UNDEF_BEHAVIOR_SANITIZER),1)
 UBSAN_SANITIZE := y
endif

$(foreach _module, $(LG_KERNEL_MODULES), \
 $(eval include $(src)/$(_module)/$(_module).Kbuild))


#
# Define CFLAGS that apply to all the Loongson kernel modules.
#

ccflags-y += -I$(src)/
ccflags-y += -I$(src)/include
ccflags-y += -I$(src)/common/inc
ccflags-y += -I$(src)/loonggpu-bridge/
ccflags-y += -I$(src)/lgkcd/
ccflags-y += -Wall $(DEFINES) $(INCLUDES) -Wno-cast-qual -Wno-error -Wno-format-extra-args
ccflags-y += -D__KERNEL__ -DMODULE -DNVRM
ccflags-y += -DLG_VERSION_STRING=\"2024.01.16\"

ifneq ($(SYSSRCHOST1X),)
 ccflags-y += -I$(SYSSRCHOST1X)
endif

# Some Android kernels prohibit driver use of filesystem functions like
# filp_open() and kernel_read(). Disable the LG_FILESYSTEM_ACCESS_AVAILABLE
# functionality that uses those functions when building for Android.

PLATFORM_IS_ANDROID ?= 0

ifeq ($(PLATFORM_IS_ANDROID),1)
 ccflags-y += -DLG_FILESYSTEM_ACCESS_AVAILABLE=0
else
 ccflags-y += -DLG_FILESYSTEM_ACCESS_AVAILABLE=1
endif

ccflags-y += -Wno-unused-function

ifneq ($(LG_BUILD_TYPE),debug)
 ccflags-y += -Wuninitialized
endif

ccflags-y += -fno-strict-aliasing

ifeq ($(LG_BUILD_TYPE),debug)
 ccflags-y += -g
endif

ccflags-y += -ffreestanding

ifeq ($(ARCH),arm64)
 ccflags-y += -mgeneral-regs-only -march=armv8-a
 ccflags-y += $(call cc-option,-mno-outline-atomics,)
endif

ifeq ($(ARCH),x86_64)
 ccflags-y += -mno-red-zone -mcmodel=kernel
endif

ifeq ($(ARCH),powerpc)
 ccflags-y += -mlittle-endian -mno-strict-align -mno-altivec
endif

ccflags-y += -DLG_UVM_ENABLE
ccflags-y += $(call cc-option,-Werror=undef,)
ccflags-y += -DLG_SPECTRE_V2=$(LG_SPECTRE_V2)
ccflags-y += -DLG_KERNEL_INTERFACE_LAYER

#
# Detect SGI UV systems and apply system-specific optimizations.
#

ifneq ($(wildcard /proc/sgi_uv),)
 ccflags-y += -DLG_CONFIG_X86_UV
endif


#
# The conftest.sh script tests various aspects of the target kernel.
# The per-module Kbuild files included above should:
#
# - Append to the LG_CONFTEST_*_COMPILE_TESTS variables to indicate
# which conftests they require.
# - Append to the LG_OBJECTS_DEPEND_ON_CONFTEST variable any object files
# that depend on conftest.
#
# The conftest machinery below will run the requested tests and
# generate the appropriate header files.
#

CC ?= cc
LD ?= ld

LG_CONFTEST_SCRIPT := $(src)/conftest.sh
LG_CONFTEST_HEADER := $(obj)/conftest/headers.h

LG_CONFTEST_CMD := /bin/sh $(LG_CONFTEST_SCRIPT) \
 "$(CC)" $(ARCH) $(LG_KERNEL_SOURCES) $(LG_KERNEL_OUTPUT)

LG_CFLAGS_FROM_CONFTEST := $(shell $(LG_CONFTEST_CMD) build_cflags)

LG_CONFTEST_CFLAGS = $(LG_CFLAGS_FROM_CONFTEST) $(ccflags-y) -fno-pie

LG_CONFTEST_COMPILE_TEST_HEADERS := $(obj)/conftest/macros.h
LG_CONFTEST_COMPILE_TEST_HEADERS += $(obj)/conftest/functions.h
LG_CONFTEST_COMPILE_TEST_HEADERS += $(obj)/conftest/symbols.h
LG_CONFTEST_COMPILE_TEST_HEADERS += $(obj)/conftest/types.h
LG_CONFTEST_COMPILE_TEST_HEADERS += $(obj)/conftest/generic.h

LG_CONFTEST_HEADERS := $(obj)/conftest/patches.h
LG_CONFTEST_HEADERS += $(obj)/conftest/headers.h
LG_CONFTEST_HEADERS += $(LG_CONFTEST_COMPILE_TEST_HEADERS)


#
# Generate a header file for a single conftest compile test. Each compile test
# header depends on conftest.sh, as well as the generated conftest/headers.h
# file, which is included in the compile test preamble.
#

$(obj)/conftest/compile-tests/%.h: $(LG_CONFTEST_SCRIPT) $(LG_CONFTEST_HEADER)
	@mkdir -p $(obj)/conftest/compile-tests
	@echo " CONFTEST: $(notdir $*)"
	@$(LG_CONFTEST_CMD) compile_tests '$(LG_CONFTEST_CFLAGS)' \
	 $(notdir $*) > $@

#
# Concatenate a conftest/*.h header from its constituent compile test headers
#
# $(1): The name of the concatenated header
# $(2): The list of compile tests that make up the header
#

define LG_GENERATE_COMPILE_TEST_HEADER
 $(obj)/conftest/$(1).h: $(addprefix $(obj)/conftest/compile-tests/,$(addsuffix .h,$(2)))
	@mkdir -p $(obj)/conftest
	@# concatenate /dev/null to prevent cat from hanging when $$^ is empty
	@cat $$^ /dev/null > $$@
endef

#
# Generate the conftest compile test headers from the lists of compile tests
# provided by the module-specific Kbuild files.
#

LG_CONFTEST_FUNCTION_COMPILE_TESTS ?=
LG_CONFTEST_GENERIC_COMPILE_TESTS ?=
LG_CONFTEST_MACRO_COMPILE_TESTS ?=
LG_CONFTEST_SYMBOL_COMPILE_TESTS ?=
LG_CONFTEST_TYPE_COMPILE_TESTS ?=

$(eval $(call LG_GENERATE_COMPILE_TEST_HEADER,functions,$(LG_CONFTEST_FUNCTION_COMPILE_TESTS)))
$(eval $(call LG_GENERATE_COMPILE_TEST_HEADER,generic,$(LG_CONFTEST_GENERIC_COMPILE_TESTS)))
$(eval $(call LG_GENERATE_COMPILE_TEST_HEADER,macros,$(LG_CONFTEST_MACRO_COMPILE_TESTS)))
$(eval $(call LG_GENERATE_COMPILE_TEST_HEADER,symbols,$(LG_CONFTEST_SYMBOL_COMPILE_TESTS)))
$(eval $(call LG_GENERATE_COMPILE_TEST_HEADER,types,$(LG_CONFTEST_TYPE_COMPILE_TESTS)))

$(obj)/conftest/patches.h: $(LG_CONFTEST_SCRIPT)
	@mkdir -p $(obj)/conftest
	@$(LG_CONFTEST_CMD) patch_check > $@


# Each of these headers is checked for presence with a test #include; a
# corresponding #define will be generated in conftest/headers.h.
LG_HEADER_PRESENCE_TESTS = \
 asm/system.h \
 drm/drmP.h \
 drm/drm_aperture.h \
 drm/drm_auth.h \
 drm/drm_gem.h \
 drm/drm_crtc.h \
 drm/drm_color_mgmt.h \
 drm/drm_atomic.h \
 drm/drm_atomic_helper.h \
 drm/drm_atomic_state_helper.h \
 drm/drm_encoder.h \
 drm/drm_atomic_uapi.h \
 drm/drm_drv.h \
 drm/drm_fbdev_generic.h \
 drm/drm_framebuffer.h \
 drm/drm_fb_helper.h \
 drm/drm_connector.h \
 drm/drm_probe_helper.h \
 drm/drm_blend.h \
 drm/drm_fourcc.h \
 drm/drm_prime.h \
 drm/drm_plane.h \
 drm/drm_vblank.h \
 drm/drm_file.h \
 drm/drm_ioctl.h \
 drm/drm_device.h \
 drm/drm_mode_config.h \
 drm/drm_modeset_lock.h \
 drm/display/drm_dp_mst_helper.h \
 drm/ttm/ttm_bo.h \
 drm/drm_damage_helper.h \
 drm/ttm/ttm_device.h \
 drm/ttm/ttm_bo_api.h \
 drm/ttm/ttm_caching.h \
 drm/ttm/ttm_bo_driver.h \
 drm/drm_buddy.h \
 drm/ttm/ttm_range_manager.h \
 drm/ttm/ttm_resource.h \
 drm/drm_irq.h \
 drm/ttm/ttm_page_alloc.h \
 drm/ttm/ttm_module.h \
 dt-bindings/interconnect/tegra_icc_id.h \
 generated/autoconf.h \
 generated/compile.h \
 generated/utsrelease.h \
 linux/aperture.h \
 linux/efi.h \
 linux/kconfig.h \
 linux/platform/tegra/mc_utils.h \
 linux/printk.h \
 linux/ratelimit.h \
 linux/prio_tree.h \
 linux/log2.h \
 linux/of.h \
 linux/bug.h \
 linux/sched.h \
 linux/sched/mm.h \
 linux/sched/signal.h \
 linux/sched/task.h \
 linux/sched/task_stack.h \
 linux/reservation.h \
 linux/device/class.h \
 xen/ioemu.h \
 linux/fence.h \
 linux/dma-fence.h \
 linux/dma-resv.h \
 linux/pci-dma-compat.h \
 soc/tegra/chip-id.h \
 soc/tegra/fuse.h \
 soc/tegra/tegra_bpmp.h \
 linux/platform/tegra/dce/dce-client-ipc.h \
 linux/nvhost.h \
 linux/nvhost_t194.h \
 linux/host1x-next.h \
 asm/book3s/64/hash-64k.h \
 asm/set_memory.h \
 asm/prom.h \
 asm/powernv.h \
 asm/loongson.h \
 linux/atomic.h \
 asm/barrier.h \
 asm/opal-api.h \
 sound/hdaudio.h \
 asm/pgtable_types.h \
 asm/page.h \
 linux/stringhash.h \
 linux/dma-map-ops.h \
 rdma/peer_mem.h \
 sound/hda_codec.h \
 linux/dma-buf.h \
 linux/time.h \
 linux/platform_device.h \
 linux/mutex.h \
 linux/reset.h \
 linux/of_platform.h \
 linux/of_device.h \
 linux/of_gpio.h \
 linux/gpio.h \
 linux/gpio/consumer.h \
 linux/interconnect.h \
 linux/pm_runtime.h \
 linux/clk.h \
 linux/clk-provider.h \
 linux/ioasid.h \
 linux/stdarg.h \
 linux/iosys-map.h \
 asm/coco.h \
 linux/vfio_pci_core.h \
 linux/mdev.h \
 soc/tegra/bpmp-abi.h \
 soc/tegra/bpmp.h \
 linux/sync_file.h \
 linux/cc_platform.h \
 drm/drm_gem_ttm_helper.h \
 asm/cpufeature.h

# Filename to store the define for the header in $(1); this is only consumed by
# the rule below that concatenates all of these together.
LG_HEADER_PRESENCE_PART = $(addprefix $(obj)/conftest/header_presence/,$(addsuffix .part,$(1)))

# Define a rule to check the header $(1).
define LG_HEADER_PRESENCE_CHECK
 $$(call LG_HEADER_PRESENCE_PART,$(1)): $$(LG_CONFTEST_SCRIPT) $(obj)/conftest/uts_release
	@mkdir -p $$(dir $$@)
	@$$(LG_CONFTEST_CMD) test_kernel_header '$$(LG_CONFTEST_CFLAGS)' '$(1)' > $$@
endef

# Evaluate the rule above for each header in the list.
$(foreach header,$(LG_HEADER_PRESENCE_TESTS),$(eval $(call LG_HEADER_PRESENCE_CHECK,$(header))))

# Concatenate all of the parts into headers.h.
$(obj)/conftest/headers.h: $(call LG_HEADER_PRESENCE_PART,$(LG_HEADER_PRESENCE_TESTS))
	@cat $^ > $@

clean-dirs := $(obj)/conftest


# For any object files that depend on conftest, declare the dependency here.
$(addprefix $(obj)/,$(LG_OBJECTS_DEPEND_ON_CONFTEST)): | $(LG_CONFTEST_HEADERS)

# Sanity checks of the build environment and target system/kernel

BUILD_SANITY_CHECKS = \
 cc_sanity_check \
 cc_version_check \
 dom0_sanity_check \
 xen_sanity_check \
 preempt_rt_sanity_check \
 vgpu_kvm_sanity_check \
 module_symvers_sanity_check

.PHONY: $(BUILD_SANITY_CHECKS)

$(BUILD_SANITY_CHECKS):
	@$(LG_CONFTEST_CMD) $@ full_output

# Perform all sanity checks before generating the conftest headers

$(LG_CONFTEST_HEADERS): | $(BUILD_SANITY_CHECKS)

# Make the conftest headers depend on the kernel version string

$(obj)/conftest/uts_release: LG_GENERATE_UTS_RELEASE
	@mkdir -p $(dir $@)
	@LG_UTS_RELEASE="// Kernel version: `$(LG_CONFTEST_CMD) compile_tests '$(LG_CONFTEST_CFLAGS)' uts_release`"; \
	if ! [ -f "$@" ] || [ "$$LG_UTS_RELEASE" != "`cat $@`" ]; \
	then echo "$$LG_UTS_RELEASE" > $@; fi

.PHONY: LG_GENERATE_UTS_RELEASE

$(LG_CONFTEST_HEADERS): $(obj)/conftest/uts_release
