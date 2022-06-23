TARGET  := a64_usb_host_drv
DRIVER  := usb
REQUIRES = arm_v8a
LIBS     = base a64_lx_emul

INC_DIR  = $(PRG_DIR)

SRC_C += lx_emul/a64/common_dummies.c
SRC_C += $(notdir $(wildcard $(PRG_DIR)/generated_dummies.c))

vpath lx_emul/a64/common_dummies.c $(REP_DIR)/src/lib

#
# Genode C-API backends
#

SRC_CC  += genode_c_api/usb.cc

vpath genode_c_api/usb.cc $(subst /genode_c_api,,$(call select_from_repositories,src/lib/genode_c_api))

BOARDS := pine_a64lts pinephone

DTS_EXTRACT(pine_a64lts)  := --select ehci0 --select ehci1
DTS_EXTRACT(pinephone)    := --select ehci1
