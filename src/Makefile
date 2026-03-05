#
# This Makefile was automatically generated; do not edit.
#

###########################################################################
# Makefile for Loongson Linux LOONGGPU driver kernel modules
###########################################################################

# This makefile is read twice: when a user or loongson-installer invokes
# 'make', this file is read.  It then invokes the Linux kernel's
# Kbuild.  Modern versions of Kbuild will then read the Kbuild file in
# this directory.  However, old versions of Kbuild will instead read
# this Makefile.  For backwards compatibility, when read by Kbuild
# (recognized by KERNELRELEASE not being empty), do nothing but
# include the Kbuild file in this directory.

ifneq ($(KERNELRELEASE),)
  include $(src)/Kbuild
else

  # Determine the location of the Linux kernel source tree, and of the
  # kernel's output tree.  Use this to invoke Kbuild, and pass the paths
  # to the source and output trees to Loongson's Kbuild file via
  # LG_KERNEL_{SOURCES,OUTPUT}.

  ifdef SYSSRC
    KERNEL_SOURCES := $(SYSSRC)
  else
    KERNEL_UNAME ?= $(shell uname -r)
    KERNEL_MODLIB := /lib/modules/$(KERNEL_UNAME)
    KERNEL_SOURCES := $(shell test -d $(KERNEL_MODLIB)/source && echo $(KERNEL_MODLIB)/source || echo $(KERNEL_MODLIB)/build)
  endif

  KERNEL_OUTPUT := $(KERNEL_SOURCES)
  KBUILD_PARAMS :=

  ifdef SYSOUT
    ifneq ($(SYSOUT), $(KERNEL_SOURCES))
      KERNEL_OUTPUT := $(SYSOUT)
      KBUILD_PARAMS := KBUILD_OUTPUT=$(KERNEL_OUTPUT)
    endif
  else
    KERNEL_UNAME ?= $(shell uname -r)
    KERNEL_MODLIB := /lib/modules/$(KERNEL_UNAME)
    ifeq ($(KERNEL_SOURCES), $(KERNEL_MODLIB)/source)
      KERNEL_OUTPUT := $(KERNEL_MODLIB)/build
      KBUILD_PARAMS := KBUILD_OUTPUT=$(KERNEL_OUTPUT)
    endif
  endif

  CC ?= cc
  LD ?= ld
  OBJDUMP ?= objdump

  ifndef ARCH
    ARCH := $(shell uname -m | sed -e 's/i.86/i386/' \
      -e 's/armv[0-7]\w\+/arm/' \
      -e 's/aarch64/arm64/' \
      -e 's/ppc64le/powerpc/' \
      -e 's/loongarch64/loongarch/' \
    )
  endif

  LG_KERNEL_MODULES ?= $(wildcard loonggpu loonggpu-bridge)
  LG_KERNEL_MODULES := $(filter-out $(LG_EXCLUDE_KERNEL_MODULES), \
                                    $(LG_KERNEL_MODULES))
  LG_VERBOSE ?=
  SPECTRE_V2_RETPOLINE ?= 0

  ifeq ($(LG_VERBOSE),1)
    KBUILD_PARAMS += V=1
  endif
  KBUILD_PARAMS += -C $(KERNEL_SOURCES) M=$(CURDIR)
  KBUILD_PARAMS += ARCH=$(ARCH)
  KBUILD_PARAMS += LG_KERNEL_SOURCES=$(KERNEL_SOURCES)
  KBUILD_PARAMS += LG_KERNEL_OUTPUT=$(KERNEL_OUTPUT)
  KBUILD_PARAMS += LG_KERNEL_MODULES="$(LG_KERNEL_MODULES)"
  KBUILD_PARAMS += INSTALL_MOD_DIR=kernel/drivers/gpu/drm/loonggpu/gpu
  KBUILD_PARAMS += LG_SPECTRE_V2=$(SPECTRE_V2_RETPOLINE)

  .PHONY: modules module clean clean_conftest modules_install
  modules clean modules_install:
	@$(MAKE) "LD=$(LD)" "CC=$(CC)" "OBJDUMP=$(OBJDUMP)" $(KBUILD_PARAMS) $@
	@if [ "$@" = "modules" ]; then \
	  for module in $(LG_KERNEL_MODULES); do \
	    if [ -x split-object-file.sh ]; then \
	      ./split-object-file.sh $$module.ko; \
	    fi; \
	  done; \
	fi

  # Compatibility target for scripts that may be directly calling the
  # "module" target from the old build system.

  module: modules

  # Check if the any of kernel module linker scripts exist. If they do, pass
  # them as linker options (via variable LG_MODULE_LD_SCRIPTS) while building
  # the kernel interface object files. These scripts do some processing on the
  # module symbols on which the Linux kernel's module resolution is dependent
  # and hence must be used whenever present.

  LD_SCRIPT ?= $(KERNEL_SOURCES)/scripts/module-common.lds      \
               $(KERNEL_SOURCES)/arch/$(ARCH)/kernel/module.lds \
               $(KERNEL_OUTPUT)/scripts/module.lds
  LG_MODULE_COMMON_SCRIPTS := $(foreach s, $(wildcard $(LD_SCRIPT)), -T $(s))

  # Kbuild's "clean" rule won't clean up the conftest headers on its own, and
  # clean-dirs doesn't appear to work as advertised.
  clean_conftest:
	$(RM) -r conftest
  clean: clean_conftest

endif # KERNELRELEASE
