count:
	@echo "conftests:$(words $(ALL_CONFTESTS))" \
		  "objects:$(words $(LG_OBJECTS_DEPEND_ON_CONFTEST))" \
		  "modules:$(words $(LG_KERNEL_MODULES))"

.PHONY: count

# Include the top-level makefile to get $(LG_KERNEL_MODULES)
include Makefile

# Set $(src) for the to-be-included loonggpu*.Kbuild files
src := $(CURDIR)

# Include loonggpu*.Kbuild and append the loonggpu*-y objects to ALL_OBJECTS
$(foreach _module, $(LG_KERNEL_MODULES),          \
     $(eval include $(_module)/$(_module).Kbuild) \
 )

# Concatenate all of the conftest lists; use $(sort ) to remove duplicates
ALL_CONFTESTS := $(sort $(LG_CONFTEST_FUNCTION_COMPILE_TESTS) \
                        $(LG_CONFTEST_GENERIC_COMPILE_TESTS)  \
                        $(LG_CONFTEST_MACRO_COMPILE_TESTS)    \
                        $(LG_CONFTEST_SYMBOL_COMPILE_TESTS)   \
                        $(LG_CONFTEST_TYPE_COMPILE_TESTS)     \
                  )
