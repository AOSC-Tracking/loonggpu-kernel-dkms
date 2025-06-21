#!/bin/sh

PATH="${PATH}:/bin:/sbin:/usr/bin"

# make sure we are in the directory containing this script
SCRIPTDIR=`dirname $0`
cd $SCRIPTDIR

CC="$1"
ARCH=$2
SOURCES=$3
HEADERS=$SOURCES/include
OUTPUT=$4
XEN_PRESENT=1
PREEMPT_RT_PRESENT=0

# VGX_BUILD parameter defined only for VGX builds (vGPU Host driver)
# VGX_KVM_BUILD parameter defined only vGPU builds on KVM hypervisor
# GRID_BUILD parameter defined only for GRID builds (GRID Guest driver)
# GRID_BUILD_CSP parameter defined only for GRID CSP builds (GRID Guest driver for CSPs)

test_xen() {
    #
    # Determine if the target kernel is a Xen kernel. It used to be
    # sufficient to check for CONFIG_XEN, but the introduction of
    # modular para-virtualization (CONFIG_PARAVIRT, etc.) and
    # Xen guest support, it is no longer possible to determine the
    # target environment at build time. Therefore, if both
    # CONFIG_XEN and CONFIG_PARAVIRT are present, text_xen() treats
    # the kernel as a stand-alone kernel.
    #
    if ! test_configuration_option CONFIG_XEN ||
         test_configuration_option CONFIG_PARAVIRT; then
        XEN_PRESENT=0
    fi
}

append_conftest() {
    #
    # Echo data from stdin: this is a transitional function to make it easier
    # to port conftests from drivers with parallel conftest generation to
    # older driver versions
    #

    while read LINE; do
        echo ${LINE}
    done
}

test_header_presence() {
    #
    # Determine if the given header file (which may or may not be
    # present) is provided by the target kernel.
    #
    # Input:
    #   $1: relative file path
    #
    # This routine creates an upper case, underscore version of each of the
    # relative file paths, and uses that as the token to either define or
    # undefine in a C header file. For example, linux/fence.h becomes
    # NV_LINUX_FENCE_H_PRESENT, and that is either defined or undefined, in the
    # output (which goes to stdout, just like the rest of this file).

    TEST_CFLAGS="-E -M $CFLAGS"

    file="$1"
    file_define=LG_`echo $file | tr '/.\-a-z' '___A-Z'`_PRESENT

    CODE="#include <$file>"

    if echo "$CODE" | $CC $TEST_CFLAGS - > /dev/null 2>&1; then
        echo "#define $file_define"
    else
        # If preprocessing failed, it could have been because the header
        # file under test is not present, or because it is present but
        # depends upon the inclusion of other header files. Attempting
        # preprocessing again with -MG will ignore a missing header file
        # but will still fail if the header file is present.
        if echo "$CODE" | $CC $TEST_CFLAGS -MG - > /dev/null 2>&1; then
            echo "#undef $file_define"
        else
            echo "#define $file_define"
        fi
    fi
}

build_cflags() {
    ISYSTEM=`$CC -print-file-name=include 2> /dev/null`
    BASE_CFLAGS="-O2 -D__KERNEL__ \
-DKBUILD_BASENAME=\"#conftest$$\" -DKBUILD_MODNAME=\"#conftest$$\" \
-nostdinc -isystem $ISYSTEM \
-Wno-implicit-function-declaration -Wno-strict-prototypes"

    if [ "$OUTPUT" != "$SOURCES" ]; then
        OUTPUT_CFLAGS="-I$OUTPUT/include2 -I$OUTPUT/include"
        if [ -f "$OUTPUT/include/generated/autoconf.h" ]; then
            AUTOCONF_FILE="$OUTPUT/include/generated/autoconf.h"
        else
            AUTOCONF_FILE="$OUTPUT/include/linux/autoconf.h"
        fi
    else
        if [ -f "$HEADERS/generated/autoconf.h" ]; then
            AUTOCONF_FILE="$HEADERS/generated/autoconf.h"
        else
            AUTOCONF_FILE="$HEADERS/linux/autoconf.h"
        fi
    fi

    test_xen

    if [ "$XEN_PRESENT" != "0" ]; then
        MACH_CFLAGS="-I$HEADERS/asm/mach-xen"
    fi

    KERNEL_ARCH="$ARCH"

    if [ "$ARCH" = "i386" -o "$ARCH" = "x86_64" ]; then
        if [ -d "$SOURCES/arch/x86" ]; then
            KERNEL_ARCH="x86"
        fi
    fi

    SOURCE_HEADERS="$HEADERS"
    SOURCE_ARCH_HEADERS="$SOURCES/arch/$KERNEL_ARCH/include"
    OUTPUT_HEADERS="$OUTPUT/include"
    OUTPUT_ARCH_HEADERS="$OUTPUT/arch/$KERNEL_ARCH/include"

    # Look for mach- directories on this arch, and add it to the list of
    # includes if that platform is enabled in the configuration file, which
    # may have a definition like this:
    #   #define CONFIG_ARCH_<MACHUPPERCASE> 1
    for _mach_dir in `ls -1d $SOURCES/arch/$KERNEL_ARCH/mach-* 2>/dev/null`; do
        _mach=`echo $_mach_dir | \
            sed -e "s,$SOURCES/arch/$KERNEL_ARCH/mach-,," | \
            tr 'a-z' 'A-Z'`
        grep "CONFIG_ARCH_$_mach \+1" $AUTOCONF_FILE > /dev/null 2>&1
        if [ $? -eq 0 ]; then
            MACH_CFLAGS="$MACH_CFLAGS -I$_mach_dir/include"
        fi
    done

    if [ "$ARCH" = "arm" ]; then
        MACH_CFLAGS="$MACH_CFLAGS -D__LINUX_ARM_ARCH__=7"
    fi

    # Add the mach-default includes (only found on x86/older kernels)
    MACH_CFLAGS="$MACH_CFLAGS -I$SOURCE_HEADERS/asm-$KERNEL_ARCH/mach-default"
    MACH_CFLAGS="$MACH_CFLAGS -I$SOURCE_ARCH_HEADERS/asm/mach-default"

    CFLAGS="$BASE_CFLAGS $MACH_CFLAGS $OUTPUT_CFLAGS -include $AUTOCONF_FILE"
    CFLAGS="$CFLAGS -I$SOURCE_HEADERS"
    CFLAGS="$CFLAGS -I$SOURCE_HEADERS/uapi"
    CFLAGS="$CFLAGS -I$SOURCE_HEADERS/xen"
    CFLAGS="$CFLAGS -I$OUTPUT_HEADERS/generated/uapi"
    CFLAGS="$CFLAGS -I$SOURCE_ARCH_HEADERS"
    CFLAGS="$CFLAGS -I$SOURCE_ARCH_HEADERS/uapi"
    CFLAGS="$CFLAGS -I$OUTPUT_ARCH_HEADERS/generated"
    CFLAGS="$CFLAGS -I$OUTPUT_ARCH_HEADERS/generated/uapi"

    if [ -n "$BUILD_PARAMS" ]; then
        CFLAGS="$CFLAGS -D$BUILD_PARAMS"
    fi

    # Check if gcc supports asm goto and set CC_HAVE_ASM_GOTO if it does.
    # Older kernels perform this check and set this flag in Kbuild, and since
    # conftest.sh runs outside of Kbuild it ends up building without this flag.
    # Starting with commit e9666d10a5677a494260d60d1fa0b73cc7646eb3 this test
    # is done within Kconfig, and the preprocessor flag is no longer needed.

    GCC_GOTO_SH="$SOURCES/build/gcc-goto.sh"

    if [ -f "$GCC_GOTO_SH" ]; then
        # Newer versions of gcc-goto.sh don't print anything on success, but
        # this is okay, since it's no longer necessary to set CC_HAVE_ASM_GOTO
        # based on the output of those versions of gcc-goto.sh.
        if [ `/bin/sh "$GCC_GOTO_SH" "$CC"` = "y" ]; then
            CFLAGS="$CFLAGS -DCC_HAVE_ASM_GOTO"
        fi
    fi

    #
    # If CONFIG_HAVE_FENTRY is enabled and gcc supports -mfentry flags then set
    # CC_USING_FENTRY and add -mfentry into cflags.
    #
    # linux/ftrace.h file indirectly gets included into the conftest source and
    # fails to get compiled, because conftest.sh runs outside of Kbuild it ends
    # up building without -mfentry and CC_USING_FENTRY flags.
    #
    grep "CONFIG_HAVE_FENTRY \+1" $AUTOCONF_FILE > /dev/null 2>&1
    if [ $? -eq 0 ]; then
        echo "" > conftest$$.c

        $CC -mfentry -c -x c conftest$$.c > /dev/null 2>&1
        rm -f conftest$$.c

        if [ -f conftest$$.o ]; then
            rm -f conftest$$.o

            CFLAGS="$CFLAGS -mfentry -DCC_USING_FENTRY"
        fi
    fi

    grep "CONFIG_AS_HAS_EXPLICIT_RELOCS \+1" $AUTOCONF_FILE > /dev/null 2>&1
    if [ $? -eq 0 ]; then
        echo "" > conftest$$.c

        $CC -mexplicit-relocs -c -x c conftest$$.c > /dev/null 2>&1
        rm -f conftest$$.c

        if [ -f conftest$$.o ]; then
            rm -f conftest$$.o
            CFLAGS="$CFLAGS -mexplicit-relocs"
        fi
    fi

    #
    # If CONFIG_CPU_LOONGSON64 is enabled then reset _LOONGARCH_ISA to
    # _LOONGARCH_ISA_LOONGARCH64.
    #
    grep "CONFIG_CPU_LOONGSON64 \+1" $AUTOCONF_FILE > /dev/null 2>&1
    if [ $? -eq 0 ]; then
        CFLAGS="$CFLAGS -U_LOONGARCH_ISA -D_LOONGARCH_ISA=_LOONGARCH_ISA_LOONGARCH64"
    fi
}

CONFTEST_PREAMBLE="#include \"conftest/headers.h\"
    #if defined(LG_LINUX_KCONFIG_H_PRESENT)
    #include <linux/kconfig.h>
    #endif
    #if defined(LG_GENERATED_AUTOCONF_H_PRESENT)
    #include <generated/autoconf.h>
    #else
    #include <linux/autoconf.h>
    #endif
    #if defined(CONFIG_XEN) && \
        defined(CONFIG_XEN_INTERFACE_VERSION) &&  !defined(__XEN_INTERFACE_VERSION__)
    #define __XEN_INTERFACE_VERSION__ CONFIG_XEN_INTERFACE_VERSION
    #endif
    #if defined(CONFIG_KASAN) && defined(CONFIG_ARM64)
    #if defined(CONFIG_KASAN_SW_TAGS)
    #define KASAN_SHADOW_SCALE_SHIFT 4
    #else
    #define KASAN_SHADOW_SCALE_SHIFT 3
    #endif
    #endif"

test_configuration_option() {
    #
    # Check to see if the given configuration option is defined
    #

    get_configuration_option $1 >/dev/null 2>&1

    return $?

}

set_configuration() {
    #
    # Set a specific configuration option.  This function is called to always
    # enable a configuration, in order to verify whether the test code for that
    # configuration is no longer required and the corresponding
    # conditionally-compiled code in the driver can be removed.
    #
    DEF="$1"

    if [ "$3" = "" ]
    then
        VAL=""
        CAT="$2"
    else
        VAL="$2"
        CAT="$3"
    fi

    echo "#define ${DEF} ${VAL}" | append_conftest "${CAT}"
}

unset_configuration() {
    #
    # Un-set a specific configuration option.  This function is called to
    # always disable a configuration, in order to verify whether the test
    # code for that configuration is no longer required and the corresponding
    # conditionally-compiled code in the driver can be removed.
    #
    DEF="$1"
    CAT="$2"

    echo "#undef ${DEF}" | append_conftest "${CAT}"
}

compile_check_conftest() {
    #
    # Compile the current conftest C file and check+output the result
    #
    CODE="$1"
    DEF="$2"
    VAL="$3"
    CAT="$4"

    echo "$CONFTEST_PREAMBLE
    $CODE" > conftest$$.c

    $CC $CFLAGS -c conftest$$.c > /dev/null 2>&1
    rm -f conftest$$.c

    if [ -f conftest$$.o ]; then
        rm -f conftest$$.o
        if [ "${CAT}" = "functions" ]; then
            #
            # The logic for "functions" compilation tests is inverted compared to
            # other compilation steps: if the function is present, the code
            # snippet will fail to compile because the function call won't match
            # the prototype. If the function is not present, the code snippet
            # will produce an object file with the function as an unresolved
            # symbol.
            #
            echo "#undef ${DEF}" | append_conftest "${CAT}"
        else
            echo "#define ${DEF} ${VAL}" | append_conftest "${CAT}"
        fi
        return
    else
        if [ "${CAT}" = "functions" ]; then
            echo "#define ${DEF} ${VAL}" | append_conftest "${CAT}"
        else
            echo "#undef ${DEF}" | append_conftest "${CAT}"
        fi
        return
    fi
}

export_symbol_present_conftest() {
    #
    # Check Module.symvers to see whether the given symbol is present.
    #

    SYMBOL="$1"
    TAB='	'

    if grep -e "${TAB}${SYMBOL}${TAB}.*${TAB}EXPORT_SYMBOL\(_GPL\)\?\s*\$" \
               "$OUTPUT/Module.symvers" >/dev/null 2>&1; then
        echo "#define LS_IS_EXPORT_SYMBOL_PRESENT_$SYMBOL 1" |
            append_conftest "symbols"
    else
        # May be a false negative if Module.symvers is absent or incomplete,
        # or if the Module.symvers format changes.
        echo "#define LS_IS_EXPORT_SYMBOL_PRESENT_$SYMBOL 0" |
            append_conftest "symbols"
    fi
}

export_symbol_gpl_conftest() {
    #
    # Check Module.symvers to see whether the given symbol is present and its
    # export type is GPL-only (including deprecated GPL-only symbols).
    #

    SYMBOL="$1"
    TAB='	'

    if grep -e "${TAB}${SYMBOL}${TAB}.*${TAB}EXPORT_\(UNUSED_\)*SYMBOL_GPL\s*\$" \
               "$OUTPUT/Module.symvers" >/dev/null 2>&1; then
        echo "#define LG_IS_EXPORT_SYMBOL_GPL_$SYMBOL 1" |
            append_conftest "symbols"
    else
        # May be a false negative if Module.symvers is absent or incomplete,
        # or if the Module.symvers format changes.
        echo "#define LG_IS_EXPORT_SYMBOL_GPL_$SYMBOL 0" |
            append_conftest "symbols"
    fi
}

get_configuration_option() {
    #
    # Print the value of given configuration option, if defined
    #
    RET=1
    OPTION=$1

    OLD_FILE="linux/autoconf.h"
    NEW_FILE="generated/autoconf.h"
    FILE=""

    if [ -f $HEADERS/$NEW_FILE -o -f $OUTPUT/include/$NEW_FILE ]; then
        FILE=$NEW_FILE
    elif [ -f $HEADERS/$OLD_FILE -o -f $OUTPUT/include/$OLD_FILE ]; then
        FILE=$OLD_FILE
    fi

    if [ -n "$FILE" ]; then
        #
        # We are looking at a configured source tree; verify
        # that its configuration includes the given option
        # via a compile check, and print the option's value.
        #

        if [ -f $HEADERS/$FILE ]; then
            INCLUDE_DIRECTORY=$HEADERS
        elif [ -f $OUTPUT/include/$FILE ]; then
            INCLUDE_DIRECTORY=$OUTPUT/include
        else
            return 1
        fi

        echo "#include <$FILE>
        #ifndef $OPTION
        #error $OPTION not defined!
        #endif

        $OPTION
        " > conftest$$.c

        $CC -E -P -I$INCLUDE_DIRECTORY -o conftest$$ conftest$$.c > /dev/null 2>&1

        if [ -e conftest$$ ]; then
            tr -d '\r\n\t ' < conftest$$
            RET=$?
        fi

        rm -f conftest$$.c conftest$$
    else
        CONFIG=$OUTPUT/.config
        if [ -f $CONFIG ] && grep "^$OPTION=" $CONFIG; then
            grep "^$OPTION=" $CONFIG | cut -f 2- -d "="
            RET=$?
        fi
    fi

    return $RET

}

check_for_ib_peer_memory_symbols() {
    kernel_dir="$1"
    module_symvers="${kernel_dir}/Module.symvers"

    sym_ib_register="ib_register_peer_memory_client"
    sym_ib_unregister="ib_unregister_peer_memory_client"
    tab='	'

    # Return 0 for true(no errors), 1 for false
    if [ ! -f "${module_symvers}" ]; then
        return 1
    fi

    if grep -e "${tab}${sym_ib_register}${tab}.*${tab}EXPORT_SYMBOL.*\$"    \
               "${module_symvers}" > /dev/null 2>&1 &&
       grep -e "${tab}${sym_ib_unregister}${tab}.*${tab}EXPORT_SYMBOL.*\$"  \
               "${module_symvers}" > /dev/null 2>&1; then
        return 0
    else
        return 1
    fi
}

compile_test() {
    case "$1" in
        drm_driver_prime_flag_present)
            #
            # Determine whether driver feature flag DRIVER_PRIME is present.
            #
            # The DRIVER_PRIME flag was added by commit 3248877ea179 (drm:
            # base prime/dma-buf support (v5)) in v3.4 (2011-11-25) and is
            # removed by commit 0424fdaf883a ("drm/prime: Actually remove
            # DRIVER_PRIME everywhere") in v5.4.
            #
            # DRIVER_PRIME define is changed to enum value by commit
            # 0e2a933b02c9 (drm: Switch DRIVER_ flags to an enum) in v5.1
            # (2019-01-29).
            #
            CODE="
            #if defined(LG_DRM_DRM_DRV_H_PRESENT)
            #include <drm/drm_drv.h>
            #endif

            unsigned int drm_driver_prime_flag_present_conftest(void) {
                return DRIVER_PRIME;
            }"

            compile_check_conftest "$CODE" "LG_DRM_DRIVER_PRIME_FLAG_PRESENT" "" "types"
        ;;

        pwm_add_table)
            #
            # Determine whether pwm_add_table function is exported in kernel.
            #
            export_symbol_gpl_conftest "pwm_add_table"
        ;;

        drm_driver_irq_shared_flag_present)
            #
            # Determine whether driver feature flag DRIVER_IRQ_SHARED is present.
            #
            # The DRIVER_IRQ_SHARED flag is removed by commit 1ff494813baf
            # ("drm/irq: Ditch DRIVER_IRQ_SHARED") in v5.1-rc1.
            #
            CODE="
            #if defined(LG_DRM_DRM_DRV_H_PRESENT)
            #include <drm/drm_drv.h>
            #endif

            unsigned int drm_driver_irq_shared_flag_present_conftest(void) {
                return DRIVER_IRQ_SHARED;
            }"

            compile_check_conftest "$CODE" "LG_DRM_DRIVER_IRQ_SHARED_FLAG_PRESENT" "" "types"
        ;;

        drm_sched_stop)
            #
            # Determine if the drm_sched_stop() function is present.
            #
            # Added by commit 222b5f044159 ("drm/sched: Refactor
            # ring mirror list handling") in v5.1-rc1.
            #
            CODE="
            #include <drm/gpu_scheduler.h>
            void conftest_drm_sched_stop(void) {
                drm_sched_stop();
            }"

            compile_check_conftest "$CODE" "LG_DRM_SCHED_STOP_PRESENT" "" "functions"
        ;;

        ttm_bo_populate)
            #
            # Determine if the ttm_bo_populate() function is present.
            #
            # commit id: fc5d96670eb2540d2572a14351e82ffe45d5ac11
            # commit msg: drm/ttm: Move swapped objects off the manager's LRU list
            # in v6.13-rc1
            #
            CODE="
            #include <drm/ttm/ttm_bo.h>
            typeof(ttm_bo_populate) conftest_ttm_bo_populate;
            int conftest_ttm_bo_populate(struct ttm_buffer_object *bo,
                                         struct ttm_operation_ctx *ctx) {
                return 0;
            }"

            compile_check_conftest "$CODE" "LG_TTM_BO_POPULATE" "" "type"
        ;;

        drm_sched_job_cleanup)
            #
            # Determine if the drm_sched_job_cleanup() function is present.
            #
            #
            CODE="
            #include <drm/gpu_scheduler.h>
            void conftest_drm_sched_job_cleanup(void) {
                drm_sched_job_cleanup();
            }"

            compile_check_conftest "$CODE" "LG_DRM_SCHED_JOB_CLEANUP_PRESENT" "" "functions"
        ;;

        pwm_apply_state)
            #
            # Determine if the pwm_apply_state() function is present.
            #
            # Added by commit 222b5f044159 ("drm/sched: Refactor
            # ring mirror list handling") in v5.1-rc1.
            #
            CODE="
            #include <linux/pwm.h>
            int conftest_pwm_apply_state(void) {
                return pwm_apply_state();
            }"

            compile_check_conftest "$CODE" "LG_PWM_APPLY_STATE" "" "functions"
        ;;

        drm_fb_helper_fill_info)
            #
            # Determine if the drm_fb_helper_fill_info() function is present.
            #
            # Added by commit 3df3116ab4b1 ("drm/fb-helper: Add
            # fill_info() functions") in v5.2-rc1.
            #
            CODE="
            #if defined(LG_DRM_DRM_FB_HELPER_H_PRESENT)
            #include <drm/drm_fb_helper.h>
            #endif

            void conftest_drm_fb_helper_fill_info(void) {
                drm_fb_helper_fill_info();
            }"

            compile_check_conftest "$CODE" "LG_DRM_FB_HELPER_FILL_INFO_PRESENT" "" "functions"
        ;;

        ttm_bo_device_init)
            #
            # Determine if the ttm_bo_device_init() function has
            # 'glob' parameter.
            #
            # ttm_bo_device_init() removed 'glob' parameter by
            # commit a64f784bb14a ("drm/ttm: initialize globals
            # during device init (v2)") in v5.0-rc1.
            #
            CODE="
            #include <drm/ttm/ttm_bo_driver.h>

            int conftest_ttm_bo_device_init_has_glob_arg(void) {
               return
                   ttm_bo_device_init(
                           NULL, /* struct ttm_bo_device *bdev */
                           NULL, /* struct ttm_bo_global *glob */
                           NULL, /* struct ttm_bo_driver *driver */
                           NULL, /* struct address_space *mapping */
                           0,    /* uint64_t file_page_offset */
                           0     /* need_dma32 */);
            }"

            compile_check_conftest "$CODE" "LG_TTM_BO_DEVICE_INIT_HAS_GLOB_ARG" "" "types"
        ;;

        ttm_eu_reserve_buffers)
            #
            # Determine if the function ttm_eu_reserve_buffers() has
            # 'del_lru' parameter.
            #
            # ttm_eu_reserve_buffers() removed 'del_lru' parameter by
            # commit 6e58ab7ac7fa ("drm/ttm: Make LRU removal optional v2")
            # in v5.3-rc1.
            #
            CODE="
            #include <drm/ttm/ttm_execbuf_util.h>
            int conftest_ttm_eu_reserve_buffers_has_del_lru_arg(void) {
                return
                    ttm_eu_reserve_buffers(
                            NULL, /* struct ww_acquire_ctx *ticket */
                            NULL, /* struct list_head *list */
                            0,    /* bool intr */
                            NULL, /* struct list_head *dups */
                            0     /* bool del_lru */);
            }"

            compile_check_conftest "$CODE" "LG_TTM_EU_RESERVE_BUFFERS_HAS_DEL_LRU_ARG" "" "types"
        ;;

        drm_syncobj_find_fence)
            #
            # Determine if the function drm_syncobj_find_fence() has
            # 'point' and 'flags' parameters.
            #
            # drm_syncobj_find_fence() added 'point' parameter by
            # commit 0a6730ea27b6 ("drm: expand drm_syncobj_find
            # fence to support timeline point v2") in v4.20-rc1.
            #
            # drm_syncobj_find_fence() added 'flags' parameter by
            # commit 649fdce23cdf ("drm: add flags to drm_syncobj_find_fence")
            # in v5.0-rc1.
            #
            CODE="
            #include <drm/drm_syncobj.h>
            int conftest_drm_syncobj_find_fence(void) {
                return
                    drm_syncobj_find_fence(
                            NULL, /* struct drm_file *file_private */
                            0,    /* u32 handle */
                            0,    /* u64 point */
                            0,    /* u64 flags */
                            NULL  /* struct dma_fence **fence */);
            }"

            compile_check_conftest "$CODE" "LG_DRM_SYNCOBJ_FIND_FENCE_HAS_FLAGS_ARG" "" "types"
        ;;

        ttm_bo_move_to_lru_tail_has_bulk_arg)
            #
            # Determine if the function ttm_bo_move_to_lru_tail() has
            # 'bulk' parameter.
            #
            # ttm_bo_move_to_lru_tail() added 'bulk' parameter by
            # commit 9a2779528edd ("drm/ttm: revise ttm_bo_move_to_lru_tail to
            # support bulk moves") in v4.20-rc1.
            #
            CODE="
            #include <drm/ttm/ttm_bo_api.h>
            void conftest_ttm_bo_move_to_lru_tail_has_bulk_arg(void) {
                (void) ttm_bo_move_to_lru_tail(NULL, NULL);
            }"

            compile_check_conftest "$CODE" "LG_TTM_BO_MOVE_TO_LRU_TAIL_HAS_BULK_ARG" "" "types"
        ;;

        loongson_screen_state)
           #
           # Determine if loongson_screen_state exist.
           # This function is added in loongson kernel v4.19.
           # commit id: 2cc57f6171f7264bf5e9f09cce9f5042e07598ad
           #
           CODE="
           #include <linux/cpufreq.h>
           typeof(loongson_screen_state) conftest_loongson_screen_state;
           void conftest_loongson_screen_state(int state) {
               return ;
           }"
           compile_check_conftest "$CODE" "LG_LOONGSON_SCREEN_STATE" "" "types"
        ;;

        pwm_request)
           #
           # Determine if pwm_request function exist.
           # commit msg: pwm: Delete deprecated functions pwm_request() and pwm_free()
           # commit id: 0af4d704ba8e5ee632b6e65015ffe4d229c1a9a9.
           # changed in v6.3-rc7
           #
           CODE="
           #include <linux/pwm.h>
           typeof(pwm_request) conftest_pwm_request;
           struct pwm_device *conftest_pwm_request(int pwm, const char *label) {
               return NULL;
           }"
           compile_check_conftest "$CODE" "LG_PWM_REQUEST" "" "types"
        ;;

        atomic_check)
           #
           # Determine if atomic_check function has drm_crtc_state arg.
           # commit msg: drm/atomic: Pass the full state to CRTC atomic_check
           # commit id: 29b77ad7b9ca8c87152a1a9e8188970fb2a93df4.
           # changed in v5.10-rc2
           #
           CODE="
           #include <drm/drm_modeset_helper_vtables.h>
           #include <drm/drm_crtc.h>
           static const struct drm_crtc_helper_funcs *funcs;
           typeof(*funcs->atomic_check) conftest_atomic_check;
           int conftest_atomic_check(struct drm_crtc *crtc,
                                      struct drm_crtc_state *state) {
               return 0;
           }"
           compile_check_conftest "$CODE" "LG_ATOMIC_CHECK_HAS_CRTC_STATE_ARG" "" "types"
        ;;

        mmu_notifier_ops_invalidate_range_start_has_range_arg)
            #
            # Determine if mmu_notifier_ops.invalidate_range_start takes (mn, range),
            # or just (mn) args. Acronym keys:
            #   mn: struct mmu_notifier *
            #   range: const struct mmu_notifier_range *
            #
            # The range arg was removed from mmu_notifier_ops.invalidate_range_start
            # by commit 5d6527a784f7 ("mm/mmu_notifier: use structure for
            # invalidate_range_start/end callback") in v5.0-rc1.
            #
            echo "$CONFTEST_PREAMBLE
            #include <linux/mmu_notifier.h>

            static const struct mmu_notifier_ops *ops;
            typeof(*ops->invalidate_range_start) conftest_mmu_notifier_invalidate_range_start_has_range_arg;

            int conftest_mmu_notifier_invalidate_range_start_has_range_arg(
                    struct mmu_notifier *mn, const struct mmu_notifier_range *range) {
                return 0;
            }" > conftest$$.c

            $CC $CFLAGS -c conftest$$.c > /dev/null 2>&1
            rm -f conftest$$.c

            if [ -f conftest$$.o ]; then
                rm -f conftest$$.o
                echo "#define LG_MMU_NOTIFIER_INVALIDATE_RANGE_START_HAS_RANGE_ARG" | append_conftest "types"
            else
                echo "#undef LG_MMU_NOTIFIER_INVALIDATE_RANGE_START_HAS_RANGE_ARG" | append_conftest "types"
            fi
        ;;

        mmu_notifier_put)
            #
            # Determine if mmu_notifier_put() is present.
            #
            # mmu_notifier_unregister_no_release() is removed
            # by commit c96245148c1e ("mm/mmu_notifiers: remove unregister_no_release") in v5.3-rc6.
            #
            CODE="
            #include <linux/mmu_notifier.h>
            void conftest_mmu_notifier_put(void) {
                mmu_notifier_put();
            }"

            compile_check_conftest "$CODE" "LG_MMU_NOTIFIER_PUT_PRESENT" "" "functions"
        ;;

        mmu_notifier_unregister_no_release)
            #
            # Determine if mmu_notifier_unregister_no_release() is present.
            #
            # mmu_notifier_unregister_no_release() is removed
            # by commit c96245148c1e ("mm/mmu_notifiers: remove unregister_no_release") in v5.3-rc6.
            #
            CODE="
            #include <linux/mmu_notifier.h>
            void conftest_mmu_notifier_unregister_no_release(void) {
                mmu_notifier_unregister_no_release();
            }"

            compile_check_conftest "$CODE" "LG_MMU_NOTIFIER_UNREGISTER_NO_RELEASE" "" "functions"
        ;;

        mmu_notifier_unregister)
            #
            # Determine if mmu_notifier_unregister() is present.
            #
            # mmu_notifier_unregister_no_release() is removed
            # by commit c96245148c1e ("mm/mmu_notifiers: remove unregister_no_release") in v5.3-rc6.
            #
            CODE="
            #include <linux/mmu_notifier.h>
            void conftest_mmu_notifier_unregister(void) {
                mmu_notifier_unregister();
            }"

            compile_check_conftest "$CODE" "LG_MMU_NOTIFIER_UNREGISTER" "" "functions"
        ;;


        drm_hdmi_avi_infoframe_from_display_mode)
            #
            # Determine if the function drm_hdmi_avi_infoframe_from_display_mode()
            # has 'connector' parameter.
            #
            # drm_hdmi_avi_infoframe_from_display_mode() added 'connector' parameter
            # by commit 13d0add333af ("drm/edid: Pass connector to AVI infoframe
            # functions") in v5.1-rc1.
            #
            CODE="
            #include <drm/drm_edid.h>

            typeof(drm_hdmi_avi_infoframe_from_display_mode) conftest_drm_hdmi_avi_infoframe_from_display_mode_has_connector_arg;
            int conftest_drm_hdmi_avi_infoframe_from_display_mode_has_connector_arg(
                    struct hdmi_avi_infoframe *frame,
                    struct drm_connector *connector,
                    const struct drm_display_mode *mode) {
                return 0;
            }"

            compile_check_conftest "$CODE" "LG_DRM_HDMI_AVI_INFOFRAME_FROM_DISPLAY_MODE_HAS_CONNECTOR_ARG" "" "types"
        ;;

        drm_fb_helper_single_add_all_connectors)
            #
            # Determine if the function drm_fb_helper_single_add_all_connectors exist
            #
            # drm/fb-helper: Remove drm_fb_helper_connector
            # commit e5852bee90d6cb7d9bd2c635c056e7746f137e06
            # in v5.2-rc4.
            #
            CODE="
            #include <drm/drm_fb_helper.h>

            typeof(drm_fb_helper_single_add_all_connectors) conftest_drm_fb_helper_single_add_all_connectors;
            int conftest_drm_fb_helper_single_add_all_connectors(struct drm_fb_helper *fb_helper) {
                return 0;
            }"

            compile_check_conftest "$CODE" "LG_DRM_FB_HELPER_SINGLE_ADD_ALL_CONNECTORS" "" "types"
        ;;

        drm_hdmi_avi_infoframe_from_display_mode_const_conn)
            #
            # Determine if the function drm_hdmi_avi_infoframe_from_display_mode()
            # has 'const struct connector' parameter.
            #
            # drm_hdmi_avi_infoframe_from_display_mode() added 'connector' parameter
            # by commit 192a3aa0e4e20e1087baa29183c5d64d48716fa9 ("drm/edid: Constify
            # connector argument to infoframe functions") in v5.8-rc3.
            #
            CODE="
            #include <drm/drm_edid.h>

            typeof(drm_hdmi_avi_infoframe_from_display_mode) conftest_drm_hdmi_avi_infoframe_from_display_mode_has_connector_arg;
            int conftest_drm_hdmi_avi_infoframe_from_display_mode_has_connector_arg(
                    struct hdmi_avi_infoframe *frame,
                    const struct drm_connector *connector,
                    const struct drm_display_mode *mode) {
                return 0;
            }"

            compile_check_conftest "$CODE" "LG_DRM_HDMI_AVI_INFOFRAME_FROM_DISPLAY_MODE_HAS_CONST_CONNECTOR_ARG" "" "types"
        ;;


        ttm_buffer_object_has_base)
            #
            # Determine if the ttm_buffer_object has an base member.
            #
            # Added by commit 8eb8833e7ed3 ("drm/ttm: add gem base object")
            # in v5.4-rc1.
            CODE="
            #include <drm/ttm/ttm_bo_api.h>
            int conftest_ttm_buffer_object_has_base(void) {
                return offsetof(struct ttm_buffer_object, base);
            }"

            compile_check_conftest "$CODE" "LG_TTM_BUFFER_OBJECT_HAS_BASE" "" "types"
        ;;

        mode_config_has_fb_modifiers)
            #
            # Determine if the mode_config has an fb_modifiers_not_supported member.
            #
            # Added by commit 2af104290da5e4858e8caefa068827d7392c6a09
            # ("drm: introduce fb_modifiers_not_supported flag in mode_config")
            # in vLinux 5.17-rc3.
            CODE="
            #include <drm/drm_mode_config.h>
            int conftest_mode_config_has_fb_modifiers(void) {
                return offsetof(struct drm_mode_config, fb_modifiers_not_supported);
            }"

            compile_check_conftest "$CODE" "LG_MODE_CONFIG_HAS_FB_MODIFIERS" "" "types"
        ;;

        dma_resv_replace_fences)
            #
            # Determine if the dma_resv_replace_fences() exist.
            #
            # Added by commit 548e7432dc2da475a18077b612e8d55b8ff51891
            # ("dma-buf: add dma_resv_replace_fences v2")
            # in v5.18-rc1
            CODE="
            #include <linux/dma-resv.h>
            typeof(dma_resv_replace_fences) conftest_dma_resv_replace_fences;
            void conftest_dma_resv_replace_fences(struct dma_resv *obj, uint64_t context,
                                                  struct dma_fence *fence,
                                                  enum dma_resv_usage usage) {
                return;
            }"

            compile_check_conftest "$CODE" "LG_DMA_RESV_REPLACE_FENCES" "" "types"
        ;;

        kobj_type_has_attr_group)
            #
            # Determine if the kobj_type has attribute_group member.
            #
            # Added by commit 980c05616e5d554c46176fe08b5601801f2f8192
            # ("driver core: make sysfs_dev_char_kobj static")
            # in v6.3-rc5.
            CODE="
            #include <linux/kobject.h>
            int conftest_kobj_type_has_attr_group(void) {
                return offsetof(struct kobj_type, default_groups);
            }"

            compile_check_conftest "$CODE" "LG_KOBJ_TYPE_HAS_ATTR_GROUP" "" "types"
        ;;

        ttm_placement_has_busy_placement)
            #
            # Determine if the ttm_placement has an busy_placement member.
            #
            # Added by commit a78a8da51b36c7a0c0c16233f91d60aac03a5a49 ("drm/ttm:
            # replace busy placement with flags v6")
            # in v6.8-rc2.
            CODE="
            #include <drm/ttm/ttm_placement.h>
            int conftest_ttm_placement_has_busy_placement(void) {
                return offsetof(struct ttm_placement, num_busy_placement);
            }"

            compile_check_conftest "$CODE" "LG_TTM_PLACEMENT_HAS_BUSY_PLACEMENT" "" "types"
        ;;

        drm_driver_has_gem_prime_export)
            #
            # Determine if the drm_driver has an gem_prime_export member.
            #
            # Added by commit 8eb8833e7ed3 ("drm/ttm: add gem base object")
            # in v5.4-rc1.
            CODE="
            #include <drm/drm_drv.h>
            int conftest_drm_driver_has_gem_prime_export(void) {
                return offsetof(struct drm_driver, gem_prime_export);
            }"

            compile_check_conftest "$CODE" "LG_DRM_DRIVER_HAS_GEM_PRIME_EXPORT" "" "types"
        ;;


        ttm_buffer_object_has_mem)
            #
            # Determine if the ttm_buffer_object has an mem member.
            #
            # Added by commit 8eb8833e7ed3 ("drm/ttm: add gem base object")
            # in v5.4-rc1.
            CODE="
            #include <drm/ttm/ttm_bo_api.h>
            int conftest_ttm_buffer_object_has_mem(void) {
                return offsetof(struct ttm_buffer_object, mem);
            }"

            compile_check_conftest "$CODE" "LG_TTM_BUFFER_OBJECT_HAS_MEM" "" "types"
        ;;

        drm_device_has_pdev)
            #
            # Determine if the drm_device has pdev member.
            #
            # Added by commit b347e04452ff6382ace8fba9c81f5bcb63be17a6 ("drm:
            # Remove pdev field from struct drm_device") in v5.13-rc1.
            CODE="
            #include <drm/drm_device.h>
            int conftest_drm_device_has_pdev(void) {
                return offsetof(struct drm_device, pdev);
            }"

            compile_check_conftest "$CODE" "LG_DDEV_HAS_PDEV" "" "types"
        ;;


        drm_driver_has_gem_prime_res_obj)
            #
            # Determine if .gem_prime_res_obj exists in drm_driver.
            #
            # Removed by commit 51c98747113e ("drm/prime: Ditch gem_prime_res_obj hook")
            # in v5.4-rc1.
            #
            CODE="
            #include <drm/drm_drv.h>
            int conftest_drm_driver_has_gem_prime_res_obj(void) {
                return offsetof(struct drm_driver, gem_prime_res_obj);
            }"

            compile_check_conftest "$CODE" "LG_DRM_DRIVER_HAS_GEM_PRIME_RES_OBJ" "" "types"
        ;;

        ttm_validate_buffer_has_num_shared)
            #
            # Determine if the ttm_validate_buffer struct has num_shared member.
            #
            # Added by commit a9f34c70fd16 ("drm/ttm: allow reserving more than
            # one shared slot v3") in  v5.0-rc1.
            #
            CODE="
            #include <drm/ttm/ttm_execbuf_util.h>
            int conftest_ttm_validate_buffer_has_num_shared(void) {
                return offsetof(struct ttm_validate_buffer, num_shared);
            }"

            compile_check_conftest "$CODE" "LG_TTM_VALIDATE_BUFFER_HAS_NUM_SHARED" "" "types"
        ;;

        drm_gem_dmabuf_kmap)
            #
            # Determine if the functions drm_gem_dmabuf_kmap/kunmap are present.
            #
            # Removed by commit 03189d5bf778 ("drm: Remove defunct dma_buf_kmap
            # stubs") v5.0-rc1.
            #
            CODE="
            #include <drm/drm_prime.h>
            typeof(drm_gem_dmabuf_kmap) conftest_drm_gem_dmabuf_kmap;
            void conftest_drm_gem_dmabuf_kmap(struct dma_buf *dma_buf, unsigned long page_num) {
                return NULL;
            }"

            compile_check_conftest "$CODE" "LG_DRM_GEM_DMABUF_KMAP_PRESENT" "" "types"
        ;;

        drm_buddy_free_list_has_flags)
            #
            # Determine if drm_buddy_free_list has flags arg.
            #
            # Changed by commit 96950929eb232038022abd961be46d492d7a6f0f
            # drm/buddy: Implement tracking clear page feature") in v6.9-rc6
            # porting in v6.10
            #
            CODE="
            #include <drm/drm_buddy.h>
            typeof(drm_buddy_free_list) conftest_drm_buddy_free_list;
            void conftest_drm_buddy_free_list(struct drm_buddy *mm,
                                              struct list_head *objects,
                                              unsigned int flags
                                              ) {
                return;
            }"
            compile_check_conftest "$CODE" "LG_DRM_BUDDY_FREE_LIST_HAS_FLAGS" "" "types"
        ;;

        zone_managed_pages)
            #
            # Determine if zone_managed_pages exist.
            #
            # Changed by commit 8b5989f3333717273d02ab87ba8781f72a6783ab
            # arm: implement the new page table range API") in v5.1
            #
            CODE="
            #include <linux/mmzone.h>
            typeof(zone_managed_pages) conftest_zone_managed_pages;
            unsigned long conftest_zone_managed_pages(struct zone *zone) {
                return 1;
            }"
            compile_check_conftest "$CODE" "LG_ZONE_MANAGED_PAGES" "" "types"
        ;;

        use_mm)
            #
            # Determine if use_mm exist.
            #
            # Changed by commit 02bf43c7b7f7a19aa59a75f5244f0a3408bace1a
            # fs.xattr.simple.rework.rbtree.rwlock.v6.2") in v6.2-rc1
            #
            CODE="
            #include <linux/mmu_context.h>
            typeof(use_mm) conftest_use_mm;
            void conftest_use_mm(struct mm_struct *mm) {
                return;
            }"
            compile_check_conftest "$CODE" "LG_USE_MM" "" "types"
        ;;

        vm_flags_set)
            #
            # Determine if vm_flags_set exist.
            #
            # Changed by commit 8b5989f3333717273d02ab87ba8781f72a6783ab
            # arm: implement the new page table range API") in v6.5
            #
            CODE="
            #include <linux/mm.h>
            typeof(vm_flags_set) conftest_vm_flags_set;
            void conftest_vm_flags_set(struct vm_area_struct *vma,
                                       vm_flags_t flags) {
                return;
            }"
            compile_check_conftest "$CODE" "LG_VM_FLAGS_SET" "" "types"
        ;;


        sysfs_emit)
            #
            # Determine if sysfs_emit exist.
            #
            # Changed by commit 8b5989f3333717273d02ab87ba8781f72a6783ab
            # arm: implement the new page table range API") in v5.1
            #
            CODE="
            #include <linux/sysfs.h>
            typeof(sysfs_emit) conftest_sysfs_emit;
            int conftest_sysfs_emit(char *buf, const char *fmt, ...) {
                return 0;
            }"
            compile_check_conftest "$CODE" "LG_SYSFS_EMIT" "" "types"
        ;;

        drm_buddy_block_trim_has_start)
            #
            # Determine if drm_buddy_block_trim has start arg.
            #
            # Changed by commit d507ae0dc83b7f43cdf6760b8f1a30aac4fc405a
            # drm/buddy: Add start address support to trim function") in v6.11-rc3
            # porting in v6.11
            #
            CODE="
            #include <drm/drm_buddy.h>
            typeof(drm_buddy_block_trim) conftest_drm_buddy_block_trim_has_start;
            int conftest_drm_buddy_block_trim_has_start(struct drm_buddy *mm,
                                                        u64 *start,
                                                        u64 new_size,
                                                        struct list_head *blocks
                                                        ) {
                return 0;
            }"
            compile_check_conftest "$CODE" "LG_DRM_BUDDY_BLOCK_TRIM_HAS_START" "" "types"
        ;;

        drm_sched_init_has_device_rq)
            #
            # Determine if drm_sched_init has num_rqs arg.
            #
            # Changed by commit 56e449603f0ac580700621a356d35d5716a62ce5
            # drm/sched: Convert the GPU scheduler to variable number of run-queues") in v6.7-rc1
            # porting in 6.7
            #
            CODE="
            #include <drm/gpu_scheduler.h>
            typeof(drm_sched_init) conftest_drm_sched_init_has_device_rq;
            int conftest_drm_sched_init_has_device_rq(struct drm_gpu_scheduler *sched,
                                              const struct drm_sched_backend_ops *ops,
                                              u32 num_rqs, uint32_t hw_submission,
                                              unsigned int hang_limit,
                                              long timeout, struct workqueue_struct *timeout_wq,
                                              atomic_t *score, const char *name, struct device *dev) {
                return 0;
            }"
            compile_check_conftest "$CODE" "LG_DRM_SCHED_INIT_HAS_DEVICE_RQ" "" "types"
        ;;

        drm_sched_init_has_device_rq_submit_wq)
            #
            # Determine if drm_sched_init has submit_wq arg.
            #
            # Changed by commit a6149f0393699308fb00149be913044977bceb56
            # drm/sched: Convert drm scheduler to use a work queue rather than kthread") in v6.7-rc1
            # porting in 6.8
            #
            CODE="
            #include <drm/gpu_scheduler.h>
            typeof(drm_sched_init) conftest_drm_sched_init_has_submit_wq;
            int conftest_drm_sched_init_has_submit_wq(struct drm_gpu_scheduler *sched,
                                              const struct drm_sched_backend_ops *ops,
                                              struct workqueue_struct *submit_wq,
                                              u32 num_rqs, u32 credit_limit,
                                              unsigned int hang_limit,
                                              long timeout,
                                              struct workqueue_struct *timeout_wq,
                                              atomic_t *score, const char *name, struct device *dev
                                              ) {
                return 0;
            }"
            compile_check_conftest "$CODE" "LG_DRM_SCHED_INIT_HAS_SUBMIT_WQ" "" "types"
        ;;

        drm_sched_init_has_struct_arg)
            #
            # Determine if drm_sched_init uses struct drm_sched_init_args arg.
            #
            # Changed by commit 796a9f55a8d1d85387b973df9a06cbf4bc2d6327
            # drm/sched: Use struct for drm_sched_init() params
            #
            CODE="
            #include <drm/gpu_scheduler.h>
            typeof(drm_sched_init) conftest_drm_sched_init_has_submit_wq;
            int conftest_drm_sched_init_has_submit_wq(struct drm_gpu_scheduler *sched,
                                                      const struct drm_sched_init_args *args
                                                      ) {
                return 0;
            }"
            compile_check_conftest "$CODE" "LG_DRM_SCHED_INIT_HAS_STRUCT_ARG" "" "types"
        ;;

        drm_sched_job_init_has_credits)
            #
            # Determine if drm_sched_job_init has credits arg.
            #
            # Changed by commit a78422e9dff366b3a46ae44caf6ec8ded9c9fc2f
            # drm/sched: implement dynamic job-flow control") in v6.7-rc1
            # porting in 6.8
            #
            CODE="
            #include <drm/gpu_scheduler.h>
            typeof(drm_sched_job_init) conftest_drm_sched_job_init_has_credits;
            int conftest_drm_sched_job_init_has_credits(struct drm_sched_job *job,
                                                   struct drm_sched_entity *entity,
                                                   u32 credits, void *owner
                                                   ) {
                return 0;
            }"
            compile_check_conftest "$CODE" "LG_DRM_SCHED_JOB_INIT_HAS_CREDITS" "" "types"
        ;;

        drm_crtc_state_has_async_flip)
            #
            # Determine if the drm_crtc_state has an async_flip member.
            #
            # Added by commit 4d85f45c73a2 ("drm/atomic: Rename
            # crtc_state->pageflip_flags to async_flip") in v5.4-rc1.
            #
            CODE="
            #include <drm/drm_crtc.h>
            int conftest_drm_crtc_state_has_async_flip(void) {
                return offsetof(struct drm_crtc_state, async_flip);
            }"

            compile_check_conftest "$CODE" "LG_DRM_CRTC_STATE_HAS_ASYNC_FLIP" "" "types"
        ;;

        drm_mode_config_funcs_has_output_poll_changed)
            #
            # Determine if the drm_mode_config_funcs has output_poll_changed member.
            #
            # Added by commit 446d0f4849b101bfc35c0d00835c3e3a4804616d ("
            # drm: Remove struct drm_mode_config_funcs.output_poll_changed
            # ") in v6.11-rc4
            #
            CODE="
            #include <drm/drm_mode_config.h>
            int conftest_drm_mode_config_funcs_has_output_poll_changed(void) {
                return offsetof(struct drm_mode_config_funcs, output_poll_changed);
            }"

            compile_check_conftest "$CODE" "LG_DRM_MODE_CFG_FUNC_HAS_OUTPUT_POLL_CHANGED" "" "types"
        ;;

        drm_driver_has_lastclose)
            #
            # Determine if the drm_driver has lastclose member.
            #
            # Added by commit b5757a5be2fac24f5c138e8ddb3b2c7be8ba1cb3 ("
            # drm: Remove struct drm_driver.lastclose
            # ") in v6.11-rc4
            #
            CODE="
            #include <drm/drm_drv.h>
            int conftest_drm_driver_has_lastclose(void) {
                return offsetof(struct drm_driver, lastclose);
            }"

            compile_check_conftest "$CODE" "LG_DRM_DRIVER_HAS_LASTCLOSE" "" "types"
        ;;

        drm_need_swiotlb)
            #
            # Determine if the funcion drm_need_swiotlb() is present.
            #
            # Added by commit 913b2cb727b7 ("drm: change func to
            # better detect wether swiotlb is needed") in v5.2-rc1.
            #
            CODE="
            #include <drm/drm_cache.h>
            bool conftest_drm_need_swiotlb(void) {
                return drm_need_swiotlb();
            }"

            compile_check_conftest "$CODE" "LG_DRM_NEED_SWIOTLB_PRESENT" "" "functions"
        ;;

        drm_driver_gem_prime_export_has_dev_arg)
            #
            # Determine if drm_driver.gem_prime_export takes (dev, obj, flags),
            # or just (obj, flags). Acronym keys:
            #   dev: struct drm_device *
            #   obj: struct drm_gem_object *
            #   flags: int
            #
            # The dev arg was removed from drm_driver.gem_prime_export
            # by commit e4fa8457b219 ("drm/prime: Align gem_prime_export with
            # obj_funcs.export") in v5.4-rc1.
            #
            echo "$CONFTEST_PREAMBLE
            #include <drm/drm_drv.h>

            static const struct drm_driver *driver;
            typeof(*driver->gem_prime_export) conftest_drm_driver_gem_prime_export_has_dev_arg;

            struct dma_buf *conftest_drm_driver_gem_prime_export_has_dev_arg(
                    struct drm_device *dev, struct drm_gem_object *obj, int flags) {
                return NULL;
            }" > conftest$$.c

            $CC $CFLAGS -c conftest$$.c > /dev/null 2>&1
            rm -f conftest$$.c

            if [ -f conftest$$.o ]; then
                rm -f conftest$$.o
                echo "#define LG_DRM_DRIVER_GEM_PRIME_EXPORT_HAS_DEV_ARG" | append_conftest "types"
            else
                echo "#undef LG_DRM_DRIVER_GEM_PRIME_EXPORT_HAS_DEV_ARG" | append_conftest "types"
            fi
        ;;

        drm_connector_for_each_possible_encoder)
            #
            # Determine if the funcion drm_connector_for_each_possible_encoder() has no 'i'.
            #
            # Changed by commit 62afb4ad425af2bc6ac6ff6d697825ae47c25211 ("drm/connector:
            # Allow max possible encoders to attach to a connector") in v5.4-rc1.
            #
            CODE="
            #include <drm/drm_connector.h>
            #if defined (LG_DRM_DRMP_H_PRESENT)
            #include <drm/drmP.h>
            #endif
            struct drm_encoder * conftest_drm_connector_for_each_possible_encoder(struct drm_connector *con,
										struct drm_encoder *enc) {
		int i;

                drm_connector_for_each_possible_encoder(con, enc, i)
			return enc;

		return NULL;
            }"

            compile_check_conftest "$CODE" "LG_DRM_CONNECTOR_FOR_EACH_POSSIBLE_ENCODER_HAS_I" "" "types"
        ;;

        drm_bridge_attach)
            #
            # Determine if the drm_bridge_attach() function has the flags argument.
            #
            # Changed by commit a25b988ff83f3ca0d8f5acf855fb1717c1c61a69 ("drm/bridge:
            # Extend bridge API to disable connector creation") in v5.6-rc3.
            #
            CODE="
            #include <drm/drm_bridge.h>
            #include <drm/drm_encoder.h>
            int conftest_drm_bridge_attach(struct drm_encoder *encoder, struct drm_bridge *bridge,
                                           struct drm_bridge *previous,
                                           enum drm_bridge_attach_flags flags) {
                return drm_bridge_attach(encoder, bridge, previous, flags);
            }"
            compile_check_conftest "$CODE" "LG_DRM_BRIDGE_ATTACH_HAS_FLAGS_ARG" "" "types"
        ;;

        drm_sched_priority)
            #
            # Determine if drm_sched_priority structure contains the DRM_SCHED_PRIORITY_COUNT.
            #
            # Changed by commit e2d732fdb7a9e421720a644580cd6a9400f97f60 ("drm/scheduler:
            # Scheduler priority fixes (v2)") in v5.9-rc1.
            #
            CODE="
            #include <drm/gpu_scheduler.h>
            enum drm_sched_priority conftest_drm_sched_priority(void) {
               return DRM_SCHED_PRIORITY_COUNT;
            }"
            compile_check_conftest "$CODE" "LG_DRM_SCHED_PRIORITY_COUNT" "" "types"
        ;;

        drm_sched_priority_has_min)
            #
            # Determine if drm_sched_priority structure contains the DRM_SCHED_PRIORITY_MIN.
            #
            # Changed by commit fe375c74806dbd30b00ec038a80a5b7bf4653ab7 ("drm/sched:
            # Rename priority MIN to LOW") in v6.7-rc3
            #
            CODE="
            #include <drm/gpu_scheduler.h>
            enum drm_sched_priority conftest_drm_sched_priority(void) {
               return DRM_SCHED_PRIORITY_MIN;
            }"
            compile_check_conftest "$CODE" "LG_DRM_SCHED_PRIORITY_MIN" "" "types"
        ;;


        drm_sched_priority_has_unset)
            #
            # Determine if drm_sched_priority structure contains the DRM_SCHED_PRIORITY_UNSET.
            #
            # Changed by commit e2d732fdb7a9e421720a644580cd6a9400f97f60 ("drm/scheduler:
            # Scheduler priority fixes (v2)") in v5.9-rc1.
            #
            CODE="
            #include <drm/gpu_scheduler.h>
            enum drm_sched_priority conftest_drm_sched_priority_has_unset(void) {
               return DRM_SCHED_PRIORITY_UNSET;
            }"
            compile_check_conftest "$CODE" "LG_DRM_SCHED_PRIORITY_COUNT_HAS_UNSET" "" "types"
        ;;

        drm_gem_object_put)
            #
            # Determine if drm_gem_object_put_unlocked() function exist.
            #
            # Changed by commit ab15d56e27be47b7feebffd6e8319ece01959fbf ("drm:
            # remove transient drm_gem_object_put_unlocked()") in v5.8-rc1.
            #
            CODE="
            #include <drm/drm_gem.h>
            void conftest_drm_gem_object_put(void) {
                drm_gem_object_put_unlocked();
            }"
            compile_check_conftest "$CODE" "LG_DRM_GEM_OBJECT_PUT_UNLOCKED" "" "functions"
        ;;

        drm_sched_entity_init)
            #
            # Determine if drm_sched_entity_init function contains the priority argument.
            #
            # Changed by commit b3ac17667f115e64c67ea6101fc814f47134b530 ("drm/scheduler:
            # rework entity creation") in v5.6.
            #
            CODE="
            #include <drm/gpu_scheduler.h>
            #include <linux/types.h>
            int conftest_drm_sched_entity_init(struct drm_sched_entity *entity,
                                               enum drm_sched_priority priority,
                                               struct drm_gpu_scheduler **sched_list,
                                               unsigned int num_sched_list,
                                               atomic_t *guilty) {
                return drm_sched_entity_init(entity, priority, sched_list, num_sched_list, guilty);
            }"
            compile_check_conftest "$CODE" "LG_DRM_SCHED_ENTITY_INIT_HAS_PRIORITY_ARG" "" "types"
        ;;

	drm_crtc_helper_funcs)
            #
            # Determine if drm_crtc_helper_funcs structure contains the get_scanout_position argument.
            #
            # Changed by commit f1e2b6371c12aec5e772e5fdedaa4455c20a787f ("drm:
            # Add get_scanout_position() to struct drm_crtc_helper_funcs") in v5.6-rc2
            #
            CODE="
            #include <drm/drm_modeset_helper_vtables.h>
            #include <linux/types.h>
            int conftest_drm_crtc_helper_funcs(struct drm_crtc_helper_funcs *crtc_helper) {
                return offsetof(struct drm_crtc_helper_funcs, get_scanout_position);
            }"
            compile_check_conftest "$CODE" "LG_DRM_CRTC_HELPER_FUNCS_HAS_GET_SCANOUT_POSITION" "" "types"
        ;;

	drm_driver_has_get_vblank_timestamp)
            #
            # Determine if drm_driver has get_vblank_timestamp.
            #
            # Changed by commit f397d66b31ab4f1380d3f31e2770e160a3e5a73b ("drm:
            # Clean-up VBLANK-related callbacks in struct drm_driver") in v5.6-rc1
            #
            CODE="
            #include <drm/drm_drv.h>
            int conftest_drm_driver_has_get_vblank_timestamp(void) {
                return offsetof(struct drm_driver, get_vblank_timestamp);
            }"
            compile_check_conftest "$CODE" "LG_DRM_DRIVER_HAS_GET_VBLANK_TIMESTAMP" "" "types"
        ;;

	mmu_notifier_ops_has_free_notifier)
            #
            # Determine if mmu_notifier_ops has free_notifier.
            #
            # Changed by commit 2c7933f53f6bff7656e3324ca1a04e478bdc57c1 ("mm/mmu_notifiers:
            # add a get/put scheme for the registration") in v5.3-rc5
            #
            CODE="
            #include <linux/mmu_notifier.h>
            int conftest_mmu_notifier_ops_has_free_notifier(void) {
                return offsetof(struct mmu_notifier_ops, free_notifier);
            }"
            compile_check_conftest "$CODE" "LG_MMU_NOTIFIER_OPS_HAS_FREE_NOTIFIER" "" "types"
        ;;

	i2c_new_client_device)
            #
            # Determine if i2c_new_client_device function exists.
            #
            # Changed by commit 390fd0475af565d2fc31de98fcc84f3c2922e008 ("i2c:
            # remove deprecated i2c_new_device API") in v5.7-rc1
            #
            CODE="
            #include <linux/i2c.h>
            struct i2c_client *conftest_i2c_new_client_device(void) {
                return i2c_new_client_device();
            }"
            compile_check_conftest "$CODE" "LG_I2C_NEW_CLIENT_DEVICE" "" "functions"
        ;;

        drm_fb_helper_init_has_max_conn_count)
            #
            # Determine if drm_fb_helper_init function has max_conn_count.
            #
            # Changed by commit 2dea2d1182179e7dded5352d3ed9f84ad3945b93 ("drm:
            # Remove unused arg from drm_fb_helper_init") in v5.6-rc5
            #
            CODE="
            #include <drm/drm_fb_helper.h>
            #include <drm/drm_device.h>
            int conftest_drm_fb_helper_init_has_max_conn_count(struct drm_device *dev,
                                                               struct drm_fb_helper *fb_helper,
                                                               int max_conn_count) {
                return drm_fb_helper_init(dev, fb_helper, max_conn_count);
            }"
            compile_check_conftest "$CODE" "LG_DRM_FB_HELP_INIT_HAS_MAX_CONN_COUNT" "" "types"
        ;;

        mm_struct_has_mmap_lock)
            #
            # Determine if mm_struct has mmap_lock.
            #
            # Changed by commit da1c55f1b272f4bd54671d459b39ea7b54944ef9 ("mmap locking API:
            # rename mmap_sem to mmap_lock") in v5.8-rc1
            #
            CODE="
            #include <linux/mm_types.h>
            int conftest_mm_struct_has_mmap_lock(void) {
                return offsetof (struct mm_struct, mmap_lock);
            }"
            compile_check_conftest "$CODE" "LG_MM_STRUCT_HAS_MMAP_LOCK" "" "types"
        ;;

        ttm_resource_has_num_pages)
            #
            # Determine if ttm_resource has num_pages.
            #
            # Changed by commit e3c92eb4a84fb0f00442e6b5cabf4f11b0eaaf41 ("drm/ttm:
            # rework on ttm_resource to use size_t type") in v6.1-rc3
            #
            CODE="
            #include <drm/ttm/ttm_resource.h>
            int conftest_ttm_resource_has_num_pages(void) {
                return offsetof (struct ttm_resource, num_pages);
            }"
            compile_check_conftest "$CODE" "LG_TTM_RES_HAS_NUM_PAGES" "" "types"
        ;;


        drm_prime_pages_to_sg)
            #
            # Determine if drm_prime_pages_to_sg function contains the drm_device argument.
            #
            # Changed by commit 707d561f77b5e2a6f90c9786bee44ee7a8dedc7e ("drm:
            # allow limiting the scatter list size") in v5.9-rc4
            #
            CODE="
            #include <drm/drm_prime.h>
            typeof(drm_prime_pages_to_sg) conftest_drm_prime_pages_to_sg;
            struct sg_table *conftest_drm_prime_pages_to_sg(struct drm_device *dev,
                                               struct page **pages,
                                               unsigned int nr_pages)
            {
                return NULL;
            }"
            compile_check_conftest "$CODE" "LG_DRM_PRIME_PAGES_TO_SG_HAS_DRM_DEVICE" "" "types"
        ;;

        ttm_fbdev_mmap)
            #
            # Determine if ttm_fbdev_mmap function exists.
            #
            # Changed by commit 12067e0e89aa296ce994299aef26eddd612cf3c4 ("drm/ttm:
            # rename ttm_fbdev_mmap") in v5.4-rc3
            #
            CODE="
            #include <drm/ttm/ttm_bo_api.h>
	    int conftest_ttm_fbdev_mmap(void) {
		return ttm_fbdev_mmap();
            }"
            compile_check_conftest "$CODE" "LG_TTM_FBDEV_MMAP" "" "functions"
        ;;

	ttm_resource_manager)
            #
            # Determine if ttm_resource_manager structure exists.
            #
            # Changed by commit b2458726b38cb69f3da3677dbdf53e47af0e8792 ("drm/ttm:
            # give resource functions their own [ch] files") in v5.9-rc1
            #
            CODE="
            #include <drm/ttm/ttm_resource.h>
            void conftest_ttm_resource_manager(struct ttm_resource_manager *man) {
                return;
            }"
            compile_check_conftest "$CODE" "LG_TTM_RESOURCE_MANAGER" "" "types"
        ;;

        ttm_bo_init_mm)
            #
            # Determine if the ttm_bo_init_mm function exists.
            #
            # Changed by commit 98399abd52b234b82457ef6c40c41543d806d3b7 ("drm/ttm:
            # purge old manager init path") in v5.9-rc1
            #
            CODE="
            #include <drm/ttm/ttm_bo_api.h>
            int conftest_ttm_bo_init_mm(void) {
                return ttm_bo_init_mm();
            }"
            compile_check_conftest "$CODE" "LG_TTM_BO_INIT_MM" "" "functions"
        ;;

        ttm_bo_clean_mm)
            #
            # Determine if the ttm_bo_clean_mm function exists.
            #
            # Changed by commit 0cf0a7984268c64e906b63a96df3e331ca61f989 ("drm/ttm:
            # make TTM responsible for cleaning system only") v5.9-rc1
            #
            CODE="
            #include <drm/ttm/ttm_bo_api.h>
            int conftest_ttm_bo_clean_mm(void) {
                return ttm_bo_clean_mm();
            }"
            compile_check_conftest "$CODE" "LG_TTM_BO_CLEAN_MM" "" "functions"
        ;;

        ttm_resource_alloc)
            #
            # Determine if the ttm_resource_alloc function exists.
            #
            # Changed by commit b2458726b38cb69f3da3677dbdf53e47af0e8792 ("drm/ttm:
            # give resource functions their own [ch] files") in v5.9-rc1
            #
            CODE="
            #include <drm/ttm/ttm_resource.h>
            int conftest_ttm_resource_alloc(void) {
                return ttm_resource_alloc();
            }"
            compile_check_conftest "$CODE" "LG_TTM_RESOURCE_ALLOC" "" "functions"
        ;;

        ttm_bo_pipeline_move)
            #
            # Determine if the ttm_bo_pipeline_move function exists.
            #
            # Changed by commit e46f468fef953dea30e7a7c69ad7e0370af26855 ("drm/ttm:
            # drop special pipeline accel cleanup function") in v5.9-rc6
            #
            CODE="
            #include <drm/ttm/ttm_bo_driver.h>
            int conftest_ttm_bo_pipeline_move(void) {
                return ttm_bo_pipeline_move();
            }"
            compile_check_conftest "$CODE" "LG_TTM_BO_PIPELINE_MOVE" "" "functions"
        ;;

        ttm_tt_bind)
            #
            # Determine if the ttm_tt_bind function exists.
            #
            # Changed by commit 9e9a153bdf2555a931fd37678a8e44d170a5d943 ("drm/ttm:
            # move ttm binding/unbinding out of ttm_tt paths") in v5.9-rc6
            #
            CODE="
            #include <drm/ttm/ttm_tt.h>
            int conftest_ttm_tt_bind(void) {
                return ttm_tt_bind();
            }"
            compile_check_conftest "$CODE" "LG_TTM_TT_BIND" "" "functions"
        ;;

        ttm_bo_mem_put)
            #
            # Determine if the ttm_bo_mem_put function exists.
            #
            # Changed by commit b2458726b38cb69f3da3677dbdf53e47af0e8792 ("drm/ttm:
            # give resource functions their own [ch] files") in v5.9-rc1
            #
            CODE="
            #include <drm/ttm/ttm_bo_driver.h>
            int conftest_ttm_bo_mem_put(void) {
                return ttm_bo_mem_put();
            }"
            compile_check_conftest "$CODE" "LG_TTM_BO_MEM_PUT" "" "functions"
        ;;

        ttm_bus_placement_has_size)
            #
            # Determine if ttm_bus_placement structure has size member.
            #
            # Changed by commit 54d04ea8cdbd143496e4f5cc9c0a9f86c0e55a2e ("drm/ttm:
            # merge offset and base in ttm_bus_placement") in v5.9-rc4
            #
            CODE="
            #include <drm/ttm/ttm_bo_api.h>
            int conftest_ttm_bus_placement_has_size(void) {
                return offsetof(struct ttm_bus_placement, size);
            }"
            compile_check_conftest "$CODE" "LG_TTM_BUS_PLACEMENT_HAS_SIZE" "" "types"
        ;;

        dma_buf_ops_has_map)
            #
            # Determine if dma_buf_ops structure has map member.
            #
            # Changed by commit 54d04ea8cdbd143496e4f5cc9c0a9f86c0e55a2e ("drm/ttm:
            # merge offset and base in ttm_bus_placement") in v5.9-rc4
            #
            CODE="
            #include <linux/dma-buf.h>
            int conftest_dma_buf_ops_has_map(void) {
                return offsetof(struct dma_buf_ops, map);
            }"
            compile_check_conftest "$CODE" "LG_DMA_BUF_OPS_HAS_MAP" "" "types"
        ;;


        get_user_pages_remote_has_tsk)
            #
            # Determine if get_user_pages_remote function has task argument.
            #
            # Changed by commit 64019a2e467a288a16b65ab55ddcbf58c1b00187 ("mm/gup:
            # remove task_struct pointer for all gup code") in v5.9-rc1
            #
            CODE="
            #include <linux/mm.h>
            void conftest_get_user_pages_remote_has_tsk(void) {
               return get_user_pages_remote(NULL, NULL, 0, 0, 0, NULL, NULL, NULL);
            }"
            compile_check_conftest "$CODE" "LG_GET_USER_PAGES_REMOTE_HAS_TSK" "" "types"
        ;;

        ttm_buffer_obj_has_offset)
            #
            # Determine if ttm_buffer_object structure has offset member.
            #
            # Changed by commit 6407d666c5353d15d9e5c7cc1b8d8ced40e425f0 ("drm/ttm:
            # do not keep GPU dependent addresses") in v5.8-rc3
            #
            CODE="
            #include <drm/ttm/ttm_bo_api.h>
            int conftest_ttm_buffer_obj_has_offset(void) {
                return offsetof(struct ttm_buffer_object, offset);
            }"
            compile_check_conftest "$CODE" "LG_TTM_BUFFER_OBJ_HAS_OFFSET" "" "types"
        ;;

        ttm_backend_func)
            #
            # Determine if ttm_backend_func structure exists.
            #
            # Changed by commit 04e89ff364dec3dc261243c2f0780635448bc466 ("drm/ttm:
            # drop the tt backend function paths") in v5.9-rc4
            #
            CODE="
            #include <drm/ttm/ttm_tt.h>
            void conftest_ttm_backend_func(void) {
               struct ttm_backend_func func;
            }"
            compile_check_conftest "$CODE" "LG_TTM_BACKEND_FUNC" "" "types"
        ;;

        ttm_tt_set_populated)
            #
            # Determine if ttm_tt_set_populated function exists.
            #
            # Changed by commit 7eec915138279d7a83ff8f219846bf7c8ae637c1 ("drm/ttm/tt:
            # add wrappers to set tt state") in v5.9-rc6
            #
            CODE="
            #include <drm/ttm/ttm_tt.h>
            void conftest_ttm_tt_set_populated(void) {
                ttm_tt_set_populated();
            }"
            compile_check_conftest "$CODE" "LG_TTM_TT_SET_POPULATED" "" "functions"
        ;;

        ttm_tt_destroy_common)
            #
            # Determine if ttm_tt_destroy_common function exists.
            #
            # Changed by commit d5f45d1e2f08685c34483719b39f91010d6222e8 ("drm/ttm/tt:
            # remove ttm_tt_destroy_common v2") in v5.14
            #
            CODE="
            #include <drm/ttm/ttm_tt.h>
            void conftest_ttm_tt_destroy_common(void) {
                ttm_tt_destroy_common();
            }"
            compile_check_conftest "$CODE" "LG_TTM_TT_DESTROY_COMMON" "" "functions"
        ;;

        swiotlb_nr_tbl)
            #
            # Determine if swiotlb_nr_tbl function exists.
            #
            # Changed by commit 2cbc2776efe4faed0e17c48ae076aa03a0fcc61f ("swiotlb:
            # remove swiotlb_nr_tbl") in v5.12-rc3
            #
            CODE="
            #include <linux/swiotlb.h>
            void conftest_swiotlb_nr_tbl(void) {
                swiotlb_nr_tbl();
            }"
            compile_check_conftest "$CODE" "LG_SWIOTLB_NR_TBL" "" "functions"
        ;;


        ttm_dma_tt_fini)
            #
            # Determine if ttm_dma_tt_fini function exists.
            #
            # Changed by commit e34b8feeaa4b65725b25f49c9b08a0f8707e8e86 ("drm/ttm:
            # merge ttm_dma_tt back into ttm_tt") in v5.10-rc1
            #
            CODE="
            #include <drm/ttm/ttm_tt.h>
            void conftest_ttm_dma_tt_fini(void) {
                ttm_dma_tt_fini();
            }"
            compile_check_conftest "$CODE" "LG_TTM_DMA_TT_FINI" "" "functions"
        ;;

        ttm_tt_populate_has_bdev)
            #
            # Determine if ttm_tt_populate function contains bdev argument.
            #
            # Changed by commit 0a667b500703db80eb30759bb67df671641dbc5b ("drm/ttm:
            # remove bdev from ttm_tt") in v5.9-rc4
            #
            CODE="
            #include <drm/ttm/ttm_bo_driver.h>
            int conftest_ttm_tt_populate_has_bdev(struct ttm_bo_driver *drv) {
               return drv->ttm_tt_populate(NULL, NULL, NULL);
            }"
            compile_check_conftest "$CODE" "LG_TTM_TT_POPULATE_HAS_BDEV" "" "types"
        ;;

        ttm_tt_has_state)
            #
            # Determine if ttm_tt structure has state member.
            #
            # Changed by commit 3a4ab168a5df5c9532763ac26cde5c2ad06ca1e5 ("drm/ttm:
            # split bound/populated flags") in v5.9-rc6
            #
            CODE="
            #include <drm/ttm/ttm_tt.h>
            int conftest_ttm_tt_has_state(void) {
               return offsetof(struct ttm_tt, state);
            }"
            compile_check_conftest "$CODE" "LG_TTM_TT_HAS_STATE" "" "types"
        ;;

        ttm_bo_driver_has_invalidate)
            #
            # Determine if ttm_bo_driver structure has invalidate member.
            #
            # Changed by commit 5e791166d377c539db0f889e7793204912c374da ("drm/ttm:
            # nuke invalidate_caches callback") in v5.5-rc6
            #
            CODE="
            #include <drm/ttm/ttm_bo_driver.h>
            int conftest_ttm_bo_driver_has_invalidate(void) {
               return offsetof(struct ttm_bo_driver, invalidate_caches);
            }"
            compile_check_conftest "$CODE" "LG_TTM_BO_DRIVER_HAS_INVALIDATE" "" "types"
        ;;

        ttm_bo_driver_has_ttm_tt_bind)
            #
            # Determine if ttm_bo_driver structure has ttm_tt_bind member.
            #
            # Changed by commit 86008a7553e6c57268e4a4f71e3a43e73b1b3ef1 ("drm/ttm:
            # add optional bind/unbind via driver") in v5.9-rc4
            #
            CODE="
            #include <drm/ttm/ttm_bo_driver.h>
            int conftest_ttm_bo_driver_has_ttm_tt_bind(void) {
               return offsetof(struct ttm_bo_driver, ttm_tt_bind);
            }"
            compile_check_conftest "$CODE" "LG_TTM_BO_DRIVER_HAS_TTM_TT_BIND" "" "types"
        ;;

	ttm_bo_device_has_glob)
            #
            # Determine if ttm_bo_device structure has glob member.
            #
            # Changed by commit 97588b5b9a6b330dc2e3fbf3dea987e37d30194e ("drm/ttm:
            # remove pointers to globals") in v5.4-rc5
            #
            CODE="
            #include <drm/ttm/ttm_bo_driver.h>
            int conftest_ttm_bo_device_has_glob(void) {
		return offsetof(struct ttm_bo_device, glob);
            }"
            compile_check_conftest "$CODE" "LG_TTM_BO_DEVICE_HAS_GLOB" "" "types"
        ;;

	ttm_bo_device_init_has_man)
            #
            # Determine if ttm_bo_device_init function has manager argument.
            #
            # Changed by commit 9d6f4484e81c0005f019c8e9b43629ead0d0d355 ("drm/ttm:
            # turn ttm_bo_device.vma_manager into a pointer") in v5.3
            #
            CODE="
            #include <drm/ttm/ttm_bo_driver.h>
            int conftest_ttm_bo_device_init_has_man(void) {
               return
                   ttm_bo_device_init(
                           NULL, /* struct ttm_bo_device *bdev */
                           NULL, /* struct ttm_bo_driver *driver */
                           NULL, /* struct address_space *mapping */
                           NULL, /* struct drm_vma_offset_manager *vma_manager */
                           0     /* need_dma32 */);

            }"
            compile_check_conftest "$CODE" "LG_TTM_BO_DEVICE_INIT_HAS_MAN" "" "types"
        ;;

        drm_device_open_count)
            #
            # Determine if drm_device structure has atomic open_count member.
            #
            # Changed by commit 7e13ad896484a0165a68197a2e64091ea28c9602 ("drm:
            # Avoid drm_global_mutex for simple inc/dec of dev->open_count") in v5.6-rc2
            #
            CODE="
            #include <drm/drm_device.h>
           int conftest_drm_device_open_count(struct drm_device *dev) {
                return atomic_read(&dev->open_count);
            }"
            compile_check_conftest "$CODE" "LG_DRM_DEVICE_HAS_ATOMIC_OPEN_COUNT" "" "types"
        ;;

        loongson_system_configuration_has_vgabios_addr)
            #
            # Determine if loongson_system_configuration structure has vgabios_addr member.
            #
            # Changed by commit e866b23acf640624e7f55e8b3eb1a2b8d7ae67f6 ("LoongArch:
            # Add boot and setup routines") in v5.19-rc1
            #
            CODE="
            #include <asm/mach-la64/boot_param.h>
            int conftest_loongson_system_configuration_has_vgabios_addr(void) {
		return offsetof(struct loongson_system_configuration, vgabios_addr);
            }"
            compile_check_conftest "$CODE" "LG_LOONGSON_SYS_CONF_HAS_VGABIOS_ADDR" "" "types"
        ;;

        ttm_resource_manager_init_has_bdev)
            #
            # Determine if ttm_resource_manager_init function has bdev argument.
            #
            # Changed by commit 3f268ef06f8cf3c481dbd5843d564f5170c6df54 ("drm/ttm:
            # add back a reference to the bdev to the res manager") in v5.15-rc1
            #
            CODE="
            #include <drm/ttm/ttm_resource.h>
            void conftest_ttm_resource_manager_init_has_bdev(void) {
                ttm_resource_manager_init(NULL, NULL, 0);
            }"
            compile_check_conftest "$CODE" "LG_TTM_RESOURCE_MANAGER_INIT_HAS_BDEV" "" "types"
        ;;

        ttm_resource_manager_evict_all)
            #
            # Determine if the ttm_resource_manager_evict_all function exists.
            #
            # Changed by commit 4ce032d64c2a30cf5b23c57b3328d5d2dab99a1f ("drm/ttm:
            # nuke ttm_bo_evict_mm and rename mgr function v3") in v5.9
            #
            CODE="
            #include <drm/ttm/ttm_resource.h>
            int conftest_ttm_resource_manager_evict_all(void) {
                return ttm_resource_manager_evict_all();
            }"
            compile_check_conftest "$CODE" "LG_TTM_RESOURCE_MANAGER_EVICT_ALL" "" "functions"
        ;;

        drm_sched_backend_ops_timedout_job_ret_sched_stat)
            #
            # Determine if the return type of drm_sched_backend_ops->timedout_job()
            # is sched_stat.
            #
            # commit a6a1f036c74e3d2a3a56b3140492c7c3ecb879f3 ("drm/scheduler:
            # Job timeout handler returns status (v3)") in v5.11-rc6.
            #
            CODE="
            #include <drm/gpu_scheduler.h>

            static const struct drm_sched_backend_ops *ops;
            typeof(*ops->timedout_job) conftest_drm_sched_backend_ops_timedout_job_ret_sched_stat;
            enum drm_gpu_sched_stat conftest_drm_sched_backend_ops_timedout_job_ret_sched_stat(
                    struct drm_sched_job *sched_job) {
                return DRM_GPU_SCHED_STAT_NONE;
            }"

            compile_check_conftest "$CODE" "LG_DRM_SCHED_BACKEND_OPS_TIMEDOUT_JOB_RET_SCHED_STAT" "" "types"
        ;;

        drm_sched_entity_push_job)
            #
            # Determine if drm_sched_entity_push_job function removes entity argument.
            #
            # Changed by commit 0e10e9a1db230ae98c8ccfeaf0734545421c3995 ("drm/sched:
            # drop entity parameter from drm_sched_push_job") in v5.15-rc1
            #
            CODE="
            #include <drm/gpu_scheduler.h>

            typeof(drm_sched_entity_push_job) conftest_drm_sched_entity_push_job;
            void conftest_drm_sched_entity_push_job(struct drm_sched_job *sched_job) {
                return ;
            }"
            compile_check_conftest "$CODE" "LG_DRM_SCHED_ENTITY_PUSH_JOB_REMOVES_ENTITY" "" "types"
        ;;

        ttm_bo_mem_space)
            #
            # Determine if ttm_bo_mem_space function has **mem member.
            #
            # Changed by commit bfa3357ef9abc9d56a2910222d2deeb9f15c91ff ("drm/ttm:
            # allocate resource object instead of embedding it v2") in v5.13-rc5
            #
            CODE="
            #include <drm/ttm/ttm_bo.h>

            typeof(ttm_bo_mem_space) conftest_ttm_bo_mem_space;
            int conftest_ttm_bo_mem_space(struct ttm_buffer_object *bo,
                                          struct ttm_placement *placement,
                                          struct ttm_resource **mem,
                                          struct ttm_operation_ctx *ctx) {
                return 0;
            }"
            compile_check_conftest "$CODE" "LG_TTM_BO_MEM_SPACE_MEM_ARG_DOUBLE_PTR" "" "types"
        ;;

        dma_resv_wait_timeout)
            #
            # Determine if the dma_resv_wait_timeout function exists.
            #
            # Changed by commit d3fae3b3daac09961ab871a25093b0ae404282d5 ("dma-buf:
            # drop the _rcu postfix on function names v3") in v5.13-rc5
            #
            CODE="
            #include <linux/dma-resv.h>
            long conftest_dma_resv_wait_timeout(void) {
                return dma_resv_wait_timeout();
            }"
            compile_check_conftest "$CODE" "LG_DMA_RESV_WAIT_TIMEOUT" "" "functions"
        ;;

        drm_sched_job_arm)
            #
            # Determine if the drm_sched_job_arm function exists.
            #
            # Changed by commit dbe48d030b285a1305a874bee523681709fba162 ("drm/sched:
            # Split drm_sched_job_init") in v5.15-rc1
            #
            CODE="
            #include <drm/gpu_scheduler.h>
            void conftest_drm_sched_job_arm(void) {
                drm_sched_job_arm();
            }"
            compile_check_conftest "$CODE" "LG_DRM_SCHED_JOB_ARM" "" "functions"
        ;;

        drm_sched_backend_ops_has_prepare)
            #
            # Determine if drm_sched_backend_ops structure has prepare member.
            #
            # Changed by commit a82f30b04c6aaefe62cbbfd297e1bb23435b6b3a ("drm/scheduler:
            # rename dependency callback into prepare_job") in v6.1-rc1
            #
            CODE="
            #include <drm/gpu_scheduler.h>
            int conftest_drm_sched_backend_ops_has_prepare(void) {
                return offsetof(struct drm_sched_backend_ops, prepare_job);
            }"
            compile_check_conftest "$CODE" "LG_DRM_SCHED_BACKEDN_OPS_HAS_PREPARE" "" "types"
        ;;

        ttm_buffer_object_has_resv)
            #
            # Determine if ttm_buffer_object structure has resv member.
            #
            # Changed by commit a82f30b04c6aaefe62cbbfd297e1bb23435b6b3a ("drm/scheduler:
            # rename dependency callback into prepare_job") in v6.1-rc1
            #
            CODE="
            #include <drm/ttm/ttm_bo_api.h>
            int conftest_ttm_buffer_object_has_resv(void) {
                return offsetof(struct ttm_buffer_object, resv);
            }"
            compile_check_conftest "$CODE" "LG_TBO_HAS_RESV" "" "types"
        ;;

        dma_resv_usage_bookkeep)
            #
            # Determine if enum dma_resv_usage has bookkeep member.
            #
            # Changed by commit 0cc848a75b742c3f9800e643cd2c03b9cfdc3d69 ("dma-buf:
            # add DMA_RESV_USAGE_BOOKKEEP v3") in v5.15-rc1
            #
            CODE="
            #include <linux/dma-resv.h>
            enum dma_resv_usage conftest_dma_resv_usage_bookkeep(void) {
                return DMA_RESV_USAGE_BOOKKEEP;
            }"
            compile_check_conftest "$CODE" "LG_DMA_RESV_USAGE_BOOKKEEP_PRESENT" "" "types"
        ;;

        drm_plane_atomic_check_has_atomic_state_arg)
            #
            # Determine if the drm_plane_helper_funcs->atomic_check()
            # has 'atomic_state' parameter.
            #
            # Changed by commit 7c11b99a8e58c0875c4d433c6aea10e9fb93beb1 ("drm/atomic:
            # Pass the full state to planes atomic_check") in v5.12-rc1
            #
            CODE="
            #include <drm/drm_modeset_helper_vtables.h>
            #include <drm/drm_plane.h>
            #include <drm/drm_atomic.h>
            static const struct drm_plane_helper_funcs *funcs;
            typeof(*funcs->atomic_check) conftest_drm_plane_atomic_check_has_atomic_state_arg;
            int conftest_drm_plane_atomic_check_has_atomic_state_arg(
                    struct drm_plane *plane,
                    struct drm_atomic_state *state) {
                return 0;
            }"
            compile_check_conftest "$CODE" "LG_ATOMIC_CHECK_HAS_DRM_ATOMIC_STATE" "" "types"
        ;;

        vga_client_register)
            #
            # Determine if the function vga_client_register() has
            # 'cookie' parameter.
            #
            # Changed by commit bf44e8cecc03c9c6197c0b65d54703746a62fb35 ("vgaarb:
            # don't pass a cookie to vga_client_register") in v5.14-rc3
            #
            CODE="
            #include <linux/vgaarb.h>
            void conftest_vga_client_register(void) {
                vga_client_register(NULL, NULL, NULL, NULL);
            }"
            compile_check_conftest "$CODE" "LG_VGA_CLIENT_REGISTER_HAS_COOKIE" "" "types"
        ;;

        ttm_bo_lock_delayed_workqueue)
            #
            # Determine if the ttm_bo_lock_delayed_workqueue function exists.
            #
            # Changed by commit cd3a8a596214e6a338a22104936c40e62bdea2b6 ("drm/ttm:
            # remove ttm_bo_(un)lock_delayed_workqueue") in v6.1-rc8
            #
            CODE="
            #include <drm/ttm/ttm_bo_api.h>
            int conftest_ttm_bo_lock_delayed_workqueue(void) {
                return ttm_bo_lock_delayed_workqueue();
            }"
            compile_check_conftest "$CODE" "LG_TTM_BO_DELAYED_WORKQUEUE" "" "functions"
        ;;

        drm_fb_helper_alloc_info)
            #
            # Determine if the drm_fb_helper_alloc_info function exists.
            #
            # Changed by commit 7fd50bc39d126d172b4db1f024d7b12484aed0fb ("drm/fb-helper:
            # Rename drm_fb_helper_alloc_fbi() to use _info postfix") in v6.1-rc4
            #
            CODE="
            #include <drm/drm_fb_helper.h>
            struct fb_info *conftest_drm_fb_helper_alloc_info(void) {
                return drm_fb_helper_alloc_info();
            }"
            compile_check_conftest "$CODE" "LG_DRM_FB_HELPER_ALLOC_INFO" "" "functions"
        ;;

        fb_info_has_apertures)
            #
            # Determine if the fb_info has apertures member.
            #
            # Added by commit 5b6373de4351debd9fa87f1c46442b2c28d8b18e ("drm/fbdev:
            # Remove aperture handling and FBINFO_MISC_FIRMWARE") in v6.2-rc4.
            CODE="
            #include <linux/fb.h>
            int conftest_fb_info_has_apertures(void) {
                return offsetof(struct fb_info, apertures);
            }"
            compile_check_conftest "$CODE" "LG_FB_INFO_HAS_APERTURES" "" "types"
        ;;

        drm_fb_helper_prepare)
            #
            # Determine if the drm_fb_helper_prepare has preferred_bpp argument.
            #
            # Added by commit 6c80a93be62d398e1854d95069340b2e60f96166 ("drm/fb-helper:
            # Initialize fb-helper's preferred BPP in prepare function") in v6.2-rc6
            CODE="
            #include <drm/drm_fb_helper.h>
            void conftest_drm_fb_helper_prepare(void) {
                drm_fb_helper_prepare(NULL, NULL, 0, NULL);
            }"
            compile_check_conftest "$CODE" "LG_DRM_FB_HELPER_PREPARE_HAS_BPP" "" "types"
        ;;

        pwm_free)
            #
            # Determine if pwm_free exist.
            #
            # Changed by commit 0af4d704ba8e5ee632b6e65015ffe4d229c1a9a9 ("pwm:
            # Delete deprecated functions pwm_request() and pwm_free()") in v6.3-rc7
            #
            CODE="
            #include <linux/pwm.h>
            void conftest_pwm_free(void) {
               pwm_free();
            }"
            compile_check_conftest "$CODE" "LG_PWM_FREE" "" "functions"
        ;;

        drm_mode_config_has_fb_base)
            #
            # Determine if the drm_mode_config has fb_base member.
            #
            # Added by commit 7c99616e3fe7f35fe25bf6f5797267da29b4751e ("drm:
            # Remove drm_mode_config::fb_base") in v6.1-rc2.
            CODE="
            #include <drm/drm_mode_config.h>
            int conftest_drm_mode_config_has_fb_base(void) {
                return offsetof(struct drm_mode_config, fb_base);
            }"

            compile_check_conftest "$CODE" "LG_DRM_MODE_CONFIG_HAS_FB_BASE" "" "types"
        ;;

        pci_set_dma_mask)
            #
            # Determine if the pci_set_dma_mask() function is present.
            #
            # by commit 7968778914e53788a01c2dee2692cab157de9ac0 ("PCI: Remove
            #  the deprecated "pci-dma-compat.h" API") in v5.18-rc1
            #
            CODE="
            #include <linux/pci-dma-compat.h>
            static inline int conftest_pci_set_dma_mask(struct pci_dev *dev, u64 mask) {
		return pci_set_dma_mask(dev, mask);
            }"
            compile_check_conftest "$CODE" "LG_PCI_SET_DMA_MASK" "" "functions"
        ;;

        ttm_device_has_lru_lock)
            #
            # Determine if the ttm_device->lru_lock exists
            #
            # by commit a1f091f8ef2b680a5184db065527612247cb4cae  ("drm/ttm:
            # switch to per device LRU lock")
            # in v5.12-rc5
            CODE="
            #include <drm/ttm/ttm_device.h>
            #include <linux/spinlock_types.h>
            int conftest_ttm_device_has_lru_lock(void) {
                return offsetof(struct ttm_device, lru_lock);
            }"
            compile_check_conftest "$CODE" "LG_TTM_DEVICE_HAS_LRU_LOCK" "" "types"
        ;;

        drm_driver_has_gem_free)
            #
            # Determine if the drm_driver->gem_free exists
            #
            # by commit d693def4fd1c23f1ca5aed1afb9993b3a2069ad2 ("drm:
            # Remove obsolete GEM and PRIME callbacks from struct drm_driver")
            # in v5.9-rc7
            CODE="
            #include <drm/drm_drv.h>
            int conftest_drm_driver_has_gem_free(void) {
                return offsetof(struct drm_driver, gem_free_object_unlocked);
            }"
            compile_check_conftest "$CODE" "LG_DRM_DRIVER_HAS_GEM_FREE" "" "types"
        ;;

	dma_resv_get_excl)
            #
            # Determine if dma_resv_get_excl() is present.
            #
            # dma_resv_get_excl is removed
            # by commit 2254e49cef7015d7697bd1617d19e620e2788ec5 ("dma-resv: Fix kerneldoc") in v5.13-rc5
            #
            CODE="
            #include <linux/dma-resv.h>
            typeof(dma_resv_get_excl) conftest_dma_resv_get_excl;
            struct dma_fence *conftest_dma_resv_get_excl(struct dma_resv *obj) {
                return NULL;
            }"
            compile_check_conftest "$CODE" "LG_DMA_RESV_GET_EXCL_PRESENT" "" "types"
        ;;

        drm_plane_helper_destroy)
            #
            # Determine if drm_plane_helper_destroy exists.
            #
            # Changed by commit 30c637151cfac8da3588f3773462e705a4ff2f59 ("drm/plane-helper:
            # Export individual helpers") in v5.19
            #
            CODE="
            #include <drm/drm_plane_helper.h>
            #include <drm/drm_plane.h>
            typeof(drm_plane_helper_destroy) conftest_drm_plane_helper_destroy;
            void conftest_drm_plane_helper_destroy(struct drm_plane *plane) {
            }"
            compile_check_conftest "$CODE" "LG_DRM_PLANE_HELPER_DESTROY" "" "type"
        ;;

        ttm_range_mgr_node)
            #
            # Determine if ttm_range_mgr_node structure exists.
            #
            # Changed by commit 3eb7d96e94150304011d214750b45766cf62d9c9 ("drm/ttm:
            # flip over the range manager to self allocated nodes") in v5.13-rc1
            #
            CODE="
            #include <drm/ttm/ttm_range_manager.h>
            struct ttm_range_mgr_node node;
            "
            compile_check_conftest "$CODE" "LG_TTM_RANGE_MGR_NODE" "" "types"
        ;;

        ttm_resource_manager_func_alloc_has_res_arg)
            #
            # Determine if ttm_resource_manager_func->alloc() has res member.
            #
            # Changed by commit cb1c81467af355829a4a9d8fa3f92ffab355d93c ("drm/ttm:
            # flip the switch for driver allocated resources v2") in v5.13-rc5
            #
            CODE="
            #include <drm/ttm/ttm_resource.h>
            struct ttm_resource_manager_func *func;
            typeof(*func->alloc) conftest_ttm_resource_manager_func_alloc_has_res_arg;
            int conftest_ttm_resource_manager_func_alloc_has_res_arg(
                                      struct ttm_resource_manager *man,
                                      struct ttm_buffer_object *bo,
                                      const struct ttm_place *place,
                                      struct ttm_resource **res) {
                return 0;
            }"
            compile_check_conftest "$CODE" "LG_MAN_FUNC_ALLOC_HAS_RES" "" "types"
        ;;

    bo_pin_count)
            #
            # Determine if ttm_buffer_object->pin_count exists.
            #
            # Changed by commit dbe48d030b285a1305a874bee523681709fba162 ("drm/
            # sched: Split drm_sched_job_init") in 5.15-rc1
            # porintg in 6.6
            #
            CODE="
            #include <drm/ttm/ttm_bo.h>
            int conftest_drm_ttm_bo_pin_count(void) {
                return offsetof(struct ttm_buffer_object, pin_count);
            }"
            compile_check_conftest "$CODE" "LG_DRM_TBO_PIN_COUNT" "" "types"
        ;;

        ttm_bo_mmap)
            #
            # Determine if struct ttm_bo_mmap exists.
            #
            # Changed by commit a2c0a97b784f837300f7b0869c82ab712c600952 ("drm/
            # ttm: Remove ttm_bo_mmap() and friends") in 5.13-rc4
            # proting in 6.6
            #
            CODE="
            #include <drm/ttm/ttm_bo_api.h>
            void conftest_ttm_bo_mmap(void) {
                ttm_bo_mmap();
            }"
            compile_check_conftest "$CODE" "LG_TTM_BO_MMAP" "" "functions"
        ;;

        get_user_pages_remote_has_vmas)
            #
            # Determine if get_user_pages_remote function has task argument.
            #
            # Changed by commit ca5e863233e8f6acd1792fd85d6bc2729a1b2c10 ("mm/
            # gup: remove vmas parameter from get_user_pages_remote()") in 6.4-rc6
            # porting in 6.6
            #
            CODE="
            #include <linux/mm.h>
            void conftest_get_user_pages_remote_has_vmas(void) {
                return get_user_pages_remote(NULL, 0, 0, 0, NULL, NULL, NULL);
            }"
            compile_check_conftest "$CODE" "LG_GET_USER_PAGES_REMOTE_HAS_VMAS" "" "types"
        ;;

        get_user_pages_has_vmas)
            #
            # Determine if get_user_pages function has vmas argument.
            #
            # Changed by commit 54d020692b342f7bd02d7f5795fb5c401caecfcc ("mm/
            # gup: remove unused vmas parameter from get_user_pages()") in 6.4-rc6
            # porting in 6.6
            #
            CODE="
            #include <linux/mm.h>
            void conftest_get_user_pages_has_vmas(void) {
                return get_user_pages(0, 0, 0, NULL, NULL);
            }"
            compile_check_conftest "$CODE" "LG_GET_USER_PAGES_HAS_VMAS" "" "types"
        ;;

        ttm_sg_tt_init_has_caching)
            #
            # Determine if ttm_sg_tt_init function has caching argument.
            #
            # Changed by commit 1b4ea4c5980ff3a64607166298269c30a9671d33 ("drm/
            # ttm: set the tt caching state at creation time") in 5.10-rc1
            # porting in 6.6
            #
            CODE="
            #include <drm/ttm/ttm_tt.h>
            typeof(ttm_sg_tt_init) conftest_ttm_sg_tt_init_has_caching;
            int conftest_ttm_sg_tt_init_has_caching(struct ttm_tt *ttm,
                                                    struct ttm_buffer_object *bo,
                                                    uint32_t page_flags,
                                                    enum ttm_caching caching) {
                return 0;
            }"
            compile_check_conftest "$CODE" "LG_TTM_SG_TT_INIT_HAS_CACHING" "" "types"
        ;;

        drm_prime_sg_to_dma_addr_array)
            #
            # Determine if function drm_prime_sg_to_dma_addr_array exists.
            #
            # Changed by commit c67e62790f5c156705fb162da840c6d89d0af6e0 ("drm/
            # prime: split array import functions v4") in 5.10-rc6
            # proting in 6.6
            #
            CODE="
            #include <drm/drm_prime.h>
            int conftest_drm_prime_sg_to_dma_addr_array(void) {
                return drm_prime_sg_to_dma_addr_array();
            }"
            compile_check_conftest "$CODE" "LG_DRM_PRIME_SG_TO_DMA_ADDR_ARRAY" "" "functions"
        ;;

        drm_sched_init_has_device)
            #
            # Determine if drm_sched_init has device arg.
            #
            # Changed by commit 8ab62eda177bc350f34fea4fcea23603b8184bfd ("drm/
            # sched: Add device pointer to drm_gpu_scheduler") in 5.17-rc6
            # proting in 6.6
            #
            CODE="
            #include <drm/gpu_scheduler.h>
            void conftest_drm_sched_init_has_device(void) {
                drm_sched_init(NULL, NULL, 0, 0, 0, NULL, NULL, NULL, NULL);
            }"
            compile_check_conftest "$CODE" "LG_DRM_SCHED_INIT_HAS_DEVICE" "" "types"
        ;;

        pci_map_page)
            #
            # Determine if pci_map_page() exists.
            #
            # Changed by commit 7968778914e53788a01c2dee2692cab157de9ac0 ("PCI:
            # Remove the deprecated "pci-dma-compat.h" API") in 5.18-rc1
            # proting in 6.6
            #
            CODE="
            #include <linux/pci-dma-compat.h>
            dma_addr_t conftest_pci_map_page(void) {
                return pci_map_page();
            }"
            compile_check_conftest "$CODE" "LG_PCI_MAP_PAGE" "" "functions"
        ;;

        ttm_bo_init_reserved_has_size)
            #
            # Determine if ttm_bo_init_reserved has size.
            #
            # Changed by commit 347987a2cf0d146484d1c586951ef10028bb1674 ("drm/
            # ttm: rename and cleanup ttm_bo_init") in 5.19-rc6
            # proting in 6.6
            #
            CODE="
            #include <drm/ttm/ttm_bo.h>
            void conftest_ttm_bo_init_reserved_has_size(void) {
                int ttm_bo_init_reserved(NULL, NULL, 0, 0, NULL, 0, NULL, NULL, NULL, NULL);
            }"
            compile_check_conftest "$CODE" "LG_TTM_BO_INIT_RESERVED_HAS_SIZE" "" "types"
        ;;

        ttm_bo_dma_acc_size)
            #
            # Determine if ttm_bo_dma_acc_size exists.
            #
            # Changed by commit f07069da6b4c5f937d6df2de6504394845513964 ("drm/
            # ttm: move memory accounting into vmwgfx v4") in 5.11
            # remove in 6.6
            #
            CODE="
            #include <drm/ttm/ttm_bo_api.h>
            size_t conftest_ttm_bo_dma_acc_size(void) {
                return ttm_bo_dma_acc_size();
            }"
            compile_check_conftest "$CODE" "LG_HAS_TTM_BO_DMA_ACC_SIZE" "" "functions"
        ;;

        vgacon_text_force)
            #
            # Determine if vgacon_text_force exists.
            #
            # Changed by commit 6a2d2ddf2c345e0149bfbffdddc4768a9ab0a741 ("drm:
            # Move nomodeset kernel parameter to the DRM subsystem") in 5.16-rc3
            # remove in 6.6
            #
            CODE="
            #include <linux/console.h>
            typeof(vgacon_text_force) conftest_vgacon_text_force;
            bool conftest_vgacon_text_force(void) {
                return true;
            }"
            compile_check_conftest "$CODE" "LG_VGACON_TEXT_FORCE" "" "types"
        ;;


        dma_resv_reserve_fences)
            #
            # Determine if dma_resv_reserve_fences exists.
            #
            # Changed by commit c8d4c18bfbc4ab467188dbe45cc8155759f49d9e ("dma-buf/
            # drivers: make reserving a shared slot mandatory v4") in 5.18-rc2
            # proting in 6.6
            #
            CODE="
            #include <linux/dma-resv.h>
            int conftest_dma_resv_reserve_fences(void) {
                return dma_resv_reserve_fences();
            }"
            compile_check_conftest "$CODE" "LG_HAS_DMA_RESV_RESERVE_FENCES" "" "functions"
        ;;

        dma_resv_get_list)
            #
            # Determine if dma_resv_get_list exists.
            #
            # Changed by commit fb5ce730f21434d8100942cf1dbe1acda255fbeb ("dma-buf:
            # rename and cleanup dma_resv_get_list v2") in v5.13-rc5
            # proting in 6.6
            #
            CODE="
            #include <linux/dma-resv.h>
            typeof(dma_resv_get_list) conftest_dma_resv_get_list;
            struct dma_resv_list *conftest_dma_resv_get_list(struct dma_resv *obj) {
                return NULL;
            }"
            compile_check_conftest "$CODE" "LG_DMA_RESV_GET_LIST" "" "types"
        ;;

        ttm_bo_evict_mm)
            #
            # Determine if ttm_bo_evict_mm exists.
            #
            # Changed by commit 4ce032d64c2a30cf5b23c57b3328d5d2dab99a1f ("drm/ttm:
            # nuke ttm_bo_evict_mm and rename mgr function v3") in v5.9
            # proting in 6.6
            #
            CODE="
            #include <drm/ttm/ttm_bo_api.h>
            int *conftest_ttm_bo_evict_mm(void) {
                return ttm_bo_evict_mm();
            }"
            compile_check_conftest "$CODE" "LG_TTM_BO_EVICT_MM" "" "functions"
        ;;

        dma_resv_add_fence)
            #
            # Determine if dma_resv_add_fence function exists.
            #
            # Changed by commit 047a1b877ed48098bed71fcfb1d4891e1b54441d ("dma-buf:
            # remove dma_resv workaround") in v5.18-rc2
            # proting in 6.6
            #
            CODE="
            #include <linux/dma-resv.h>
            void conftest_dma_resv_add_fence(void) {
                return dma_resv_add_fence();
            }"
            compile_check_conftest "$CODE" "LG_DMA_RESV_ADD_FENCE" "" "functions"
        ;;

        drm_sched_dependency_optimized)
            #
            # Determine if drm_sched_dependency_optimized function exists.
            #
            # Changed by commit 2cf9886e281678ae9ee57e24a656749071d543bb ("drm/scheduler:
            # remove drm_sched_dependency_optimized") in v6.1-rc4
            # proting in 6.6
            #
            CODE="
            #include <drm/gpu_scheduler.h>
            #include <linux/dma-fence.h>
            bool conftest_drm_sched_dependency_optimized(struct dma_fence *fence,
                                                         struct drm_sched_entity *entity) {
                return drm_sched_dependency_optimized(fence, entity);
            }"
            compile_check_conftest "$CODE" "LG_DRM_SCHED_DEPENDENCY_OPTIMIZED" "" "functions"
        ;;

        prime_handle_to_fd)
            #
            # Determine if prime_handle_to_fd function exists.
            #
            # Changed by commit 2cf9886e281678ae9ee57e24a656749071d543bb ("drm/scheduler:
            # remove drm_sched_dependency_optimized") in v6.1-rc4
            # proting in 6.6
            #
            CODE="
            #include <drm/drm_prime.h>
            int conftest_prime_handle_to_fd(void) {
                return drm_gem_prime_handle_to_fd();
            }"
            compile_check_conftest "$CODE" "LG_DRM_GEM_PRIME_HANDLE_TO_FD" "" "functions"
        ;;

        ttm_global)
            #
            # Determine if ttm_global structure exists.
            #
            # Changed by commit 8af8a109b34fa88b8b91f25d11485b37d37549c3 ("drm/ttm:
            # device naming cleanup") in v5.11-rc5
            # proting in 6.6
            #
            CODE="
            #include <drm/ttm/ttm_device.h>
            void conftest_ttm_global(struct ttm_global *glob) {
            }"
            compile_check_conftest "$CODE" "LG_TTM_GLOBAL" "" "types"
        ;;

        ttm_dma_tt)
            #
            # Determine if ttm_dma_tt structure exists.
            #
            # Changed by commit e34b8feeaa4b65725b25f49c9b08a0f8707e8e86 ("drm/ttm:
            # merge ttm_dma_tt back into ttm_tt") in v5.10-rc1
            # proting in 6.6
            #
            CODE="
            #include <drm/ttm/ttm_tt.h>
            typeof(ttm_dma_tt_init) conftest_ttm_dma_tt_init;
            int conftest_ttm_dma_tt_init(struct ttm_dma_tt *ttm_dma, struct ttm_buffer_object *bo,
                                         uint32_t page_flags) {
                return 0;
            }"
            compile_check_conftest "$CODE" "LG_TTM_DMA_TT" "" "types"
        ;;

        ttm_resource_free_double_ptr)
            #
            # Determine if ttm_resource_free->res is double ptr.
            #
            # Changed by commit bfa3357ef9abc9d56a2910222d2deeb9f15c91ff ("drm/ttm:
            # allocate resource object instead of embedding it v2") in v5.13-rc5
            # porting in 6.6
            #
            CODE="
            #include <drm/ttm/ttm_resource.h>
            typeof(ttm_resource_free) conftest_ttm_resource_free_double_ptr;
            void conftest_ttm_resource_free_double_ptr(struct ttm_buffer_object *bo,
                                                       struct ttm_resource **res) {
                return;
            }"
            compile_check_conftest "$CODE" "LG_TTM_RESOURCE_FREE_DOUBLE_PTR" "" "types"
        ;;

        drm_driver_syncobj_timeline_present)
            #
            # Determine whether syncobj timeline is available.
            #
            # The syncobj timeline is available by commit e4f87db1d8a8
            # ("drm/syncobj: add timeline signal ioctl for syncobj v5") in v5.1-rc3.
            #
            CODE="
            #include <drm/drm.h>

            unsigned int drm_syncobj_timeline_present_conftest(void) {
                return DRM_IOCTL_SYNCOBJ_TIMELINE_SIGNAL;
            }"

            compile_check_conftest "$CODE" "LG_DRM_DRIVER_SYNCOBJ_TIMELINE_PRESENT" "" "types"
        ;;

        timer_container_of)
            #
            # Determine whether from_timer is renamed to timer_container_of.
            #
            # Changed by commit 41cb08555c4164996d67c78b3bf1c658075b75f1
			# ("treewide, timers: Rename from_timer() to timer_container_of()")
            #
            CODE="
            #include <linux/timer.h>

            #ifndef timer_container_of
            #error no timer_container_of
            #endif"

            compile_check_conftest "$CODE" "LG_DRM_DRIVER_TIMER_CONTAINER_OF" "" "types"
        ;;

        uts_release)
            #
            # print the kernel's UTS_RELEASE string.
            #
            echo "#include <generated/utsrelease.h>
            UTS_RELEASE" > conftest$$.c

            $CC $CFLAGS -E -P conftest$$.c
            rm -f conftest$$.c
        ;;

        # When adding a new conftest entry, please use the correct format for
        # specifying the relevant upstream Linux kernel commit.  Please
        # avoid specifying -rc kernels, and only use SHAs that actually exist
        # in the upstream Linux kernel git repository.
        #
        # Added|Removed|etc by commit <short-sha> ("<commit message") in
        # <kernel-version>.

        *)
            # Unknown test name given
            echo "Error: unknown conftest '$1' requested" >&2
            exit 1
        ;;
    esac
}

case "$5" in
    cc_sanity_check)
        #
        # Check if the selected compiler can create object files
        # in the current environment.
        #
        VERBOSE=$6

        echo "int cc_sanity_check(void) {
            return 0;
        }" > conftest$$.c

        $CC -c conftest$$.c > /dev/null 2>&1
        rm -f conftest$$.c

        if [ ! -f conftest$$.o ]; then
            if [ "$VERBOSE" = "full_output" ]; then
                echo "";
            fi
            if [ "$CC" != "cc" ]; then
                echo "The C compiler '$CC' does not appear to be able to"
                echo "create object files.  Please make sure you have "
                echo "your Linux distribution's libc development package"
                echo "installed and that '$CC' is a valid C compiler";
                echo "name."
            else
                echo "The C compiler '$CC' does not appear to be able to"
                echo "create executables.  Please make sure you have "
                echo "your Linux distribution's gcc and libc development"
                echo "packages installed."
            fi
            if [ "$VERBOSE" = "full_output" ]; then
                echo "";
                echo "*** Failed CC sanity check. Bailing out! ***";
                echo "";
            fi
            exit 1
        else
            rm -f conftest$$.o
            exit 0
        fi
    ;;

    cc_version_check)
        #
        # Verify that the same compiler major and minor version is
        # used for the kernel and kernel module. A mismatch condition is
        # not considered fatal, so this conftest returns a success status
        # code, even if it fails. Failure of the test can be distinguished
        # by testing for empty (success) versus non-empty (failure) output.
        #
        # Some gcc version strings that have proven problematic for parsing
        # in the past:
        #
        #  gcc.real (GCC) 3.3 (Debian)
        #  gcc-Version 3.3 (Debian)
        #  gcc (GCC) 3.1.1 20020606 (Debian prerelease)
        #  version gcc 3.2.3
        #
        #  As of this writing, GCC uses a version number as x.y.z and below
        #  are the typical version strings seen with various distributions.
        #  gcc (GCC) 4.4.7 20120313 (Red Hat 4.4.7-23)
        #  gcc version 4.8.5 20150623 (Red Hat 4.8.5-39) (GCC)
        #  gcc (GCC) 8.3.1 20190507 (Red Hat 8.3.1-4)
        #  gcc (GCC) 10.2.1 20200723 (Red Hat 10.2.1-1)
        #  gcc (Ubuntu 9.3.0-17ubuntu1~20.04) 9.3.0
        #  gcc (Ubuntu 7.5.0-3ubuntu1~16.04) 7.5.0
        #  gcc (Debian 8.3.0-6) 8.3.0
        #  aarch64-linux-gcc.br_real (Buildroot 2020.08-14-ge5a2a90) 9.3.0, GNU ld (GNU Binutils) 2.33.1
        #
        #  In order to extract GCC version correctly for version strings
        #  like the last one above, we first check for x.y.z and if that
        #  fails, we fallback to x.y format.
        VERBOSE=$6

        kernel_compile_h=$OUTPUT/include/generated/compile.h

        if [ ! -f ${kernel_compile_h} ]; then
            # The kernel's compile.h file is not present, so there
            # isn't a convenient way to identify the compiler version
            # used to build the kernel.
            IGNORE_CC_MISMATCH=1
        fi

        if [ -n "$IGNORE_CC_MISMATCH" ]; then
            exit 0
        fi

        kernel_cc_string=`cat ${kernel_compile_h} | \
            grep LINUX_COMPILER | cut -f 2 -d '"'`

        kernel_cc_version=`echo ${kernel_cc_string} | grep -o '[0-9]\+\.[0-9]\+\.[0-9]\+' | head -n 1`
        if [ -z "${kernel_cc_version}" ]; then
            kernel_cc_version=`echo ${kernel_cc_string} | grep -o '[0-9]\+\.[0-9]\+' | head -n 1`
        fi
        kernel_cc_major=`echo ${kernel_cc_version} | cut -d '.' -f 1`
        kernel_cc_minor=`echo ${kernel_cc_version} | cut -d '.' -f 2`

        echo "
        #if (__GNUC__ != ${kernel_cc_major}) || (__GNUC_MINOR__ != ${kernel_cc_minor})
        #error \"cc version mismatch\"
        #endif
        " > conftest$$.c

        $CC $CFLAGS -c conftest$$.c > /dev/null 2>&1
        rm -f conftest$$.c

        if [ -f conftest$$.o ]; then
            rm -f conftest$$.o
            exit 0;
        else
            #
            # The gcc version check failed
            #

            if [ "$VERBOSE" = "full_output" ]; then
                echo "";
                echo "Warning: Compiler version check failed:";
                echo "";
                echo "The major and minor number of the compiler used to";
                echo "compile the kernel:";
                echo "";
                echo "${kernel_cc_string}";
                echo "";
                echo "does not match the compiler used here:";
                echo "";
                $CC --version
                echo "";
                echo "It is recommended to set the CC environment variable";
                echo "to the compiler that was used to compile the kernel.";
                echo ""
                echo "To skip the test and silence this warning message, set";
                echo "the IGNORE_CC_MISMATCH environment variable to \"1\".";
                echo "However, mixing compiler versions between the kernel";
                echo "and kernel modules can result in subtle bugs that are";
                echo "difficult to diagnose.";
                echo "";
                echo "*** Failed CC version check. ***";
                echo "";
            elif [ "$VERBOSE" = "just_msg" ]; then
                echo "Warning: The kernel was built with ${kernel_cc_string}, but the" \
                     "current compiler version is `$CC --version | head -n 1`.";
            fi
            exit 0;
        fi
    ;;

    patch_check)
        #
        # Check for any "official" patches that may have been applied and
        # construct a description table for reporting purposes.
        #
        PATCHES=""

        for PATCH in patch-*.h; do
            if [ -f $PATCH ]; then
                echo "#include \"$PATCH\""
                PATCHES="$PATCHES "`echo $PATCH | sed -s 's/patch-\(.*\)\.h/\1/'`
            fi
        done

        echo "static struct {
                const char *short_description;
                const char *description;
              } __nv_patches[] = {"
            for i in $PATCHES; do
                echo "{ \"$i\", LG_PATCH_${i}_DESCRIPTION },"
            done
        echo "{ NULL, NULL } };"

        exit 0
    ;;

    compile_tests)
        #
        # Run a series of compile tests to determine the set of interfaces
        # and features available in the target kernel.
        #
        shift 5

        CFLAGS=$1
        shift

        for i in $*; do compile_test $i; done

        exit 0
    ;;

    dom0_sanity_check)
        #
        # Determine whether running in DOM0.
        #
        VERBOSE=$6

        if [ -n "$VGX_BUILD" ]; then
            if [ -f /proc/xen/capabilities ]; then
                if [ "`cat /proc/xen/capabilities`" == "control_d" ]; then
                    exit 0
                fi
            else
                echo "The kernel is not running in DOM0.";
                echo "";
                if [ "$VERBOSE" = "full_output" ]; then
                    echo "*** Failed DOM0 sanity check. Bailing out! ***";
                    echo "";
                fi
            fi
            exit 1
        fi
    ;;
    test_configuration_option)
        #
        # Check to see if the given config option is set.
        #
        OPTION=$6

        test_configuration_option $OPTION
        exit $?
    ;;

    get_configuration_option)
        #
        # Get the value of the given config option.
        #
        OPTION=$6

        get_configuration_option $OPTION
        exit $?
    ;;


    guess_module_signing_hash)
        #
        # Determine the best cryptographic hash to use for module signing,
        # to the extent that is possible.
        #

        HASH=$(get_configuration_option CONFIG_MODULE_SIG_HASH)

        if [ $? -eq 0 ] && [ -n "$HASH" ]; then
            echo $HASH
            exit 0
        else
            for SHA in 512 384 256 224 1; do
                if test_configuration_option CONFIG_MODULE_SIG_SHA$SHA; then
                    echo sha$SHA
                    exit 0
                fi
            done
        fi
        exit 1
    ;;


    test_kernel_header)
        #
        # Check for the availability of the given kernel header
        #

        CFLAGS=$6

        test_header_presence "${7}"

        exit $?
    ;;


    build_cflags)
        #
        # Generate CFLAGS for use in the compile tests
        #

        build_cflags
        echo $CFLAGS
        exit 0
    ;;

    module_symvers_sanity_check)
        #
        # Check whether Module.symvers exists and contains at least one
        # EXPORT_SYMBOL* symbol from vmlinux
        #

        if [ -n "$IGNORE_MISSING_MODULE_SYMVERS" ]; then
            exit 0
        fi

        TAB='	'

        if [ -f "$OUTPUT/Module.symvers" ] && \
             grep -e "^[^${TAB}]*${TAB}[^${TAB}]*${TAB}\+vmlinux" \
                     "$OUTPUT/Module.symvers" >/dev/null 2>&1; then
            exit 0
        fi

        echo "The Module.symvers file is missing, or does not contain any"
        echo "symbols exported from the kernel. This could cause the LOONGGPU"
        echo "kernel modules to be built against a configuration that does"
        echo "not accurately reflect the actual target kernel."
        echo "The Module.symvers file check can be disabled by setting the"
        echo "environment variable IGNORE_MISSING_MODULE_SYMVERS to 1."

        exit 1
    ;;
esac

