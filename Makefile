# NXP BMS TAAP 12V -- S32K116 BMS Firmware
# Ported from ARS_SW_BMS_F4_12V (STM32F401)

# Pin assignments live in src/board_pins.h; see docs/S32K116_V2_board_review.md.

# Toolchain
TOOLCHAIN = $(HOME)/gcc-arm-gnu-tool-stm32/bin/arm-none-eabi-
CC      = $(TOOLCHAIN)gcc
AS      = $(TOOLCHAIN)gcc
LD      = $(TOOLCHAIN)gcc
OBJCOPY = $(TOOLCHAIN)objcopy
OBJDUMP = $(TOOLCHAIN)objdump
SIZE    = $(TOOLCHAIN)size

# Project
TARGET  = nxp_bms
BUILD   = build

# Sources -- Phase 1: Skeleton + HAL
C_SRCS  = src/main.c \
          src/hal/hal_systime.c \
          src/hal/hal_uart.c \
          src/hal/hal_uart1.c \
          src/hal/hal_gpio.c \
          startup/startup.c \
          startup/system_S32K116.c \
          startup/syscalls.c

# Phase 2: I2C HAL + BQ7690x Driver
C_SRCS += src/hal/hal_i2c.c \
          src/bq7690x/bq7690x.c \
          src/bq7690x/bq7690x_measurements.c \
          src/bq7690x/bq7690x_status.c \
          src/bq7690x/bq7690x_protection.c \
          src/bq7690x/bq7690x_fet.c \
          src/bq7690x/bq7690x_balance.c \
          src/bq7690x/bq7690x_calibration.c \
          src/bq7690x/bq7690x_config.c \
          src/bq7690x/bq7690x_info.c \
          src/bq7690x/bq7690x_system.c \
          src/bq7690x/bq7690x_power_modes.c \
          src/bq7690x/bq7690x_power.c

# Phase 3: BQ Controller
C_SRCS += src/controller/bq_controller.c \
          src/controller/cell_open_wire.c \
          src/controller/wire_fault.c \
          src/controller/bq_link_supervisor.c

# Phase 4: CRC + Link Protocol
C_SRCS += src/hal/hal_crc.c \
          src/protocol/crc_calc.c \
          src/protocol/link_protocol.c

# Phase 5: Battery SOC/SOH
C_SRCS += src/controller/battery_soc_soh.c

# Phase 6: External Temperature ADC
C_SRCS += src/hal/hal_adc.c \
          src/controller/external_temp_adc.c \
          src/controller/ntc_bands.c \
          src/controller/ntc_fault.c

# Phase 7: System Health + Debug Report
C_SRCS += src/controller/bl_compat.c \
          src/controller/system_health.c \
          src/controller/debug_report.c \
          src/common/usart_util.c

# Phase 8: Triple UART Command
C_SRCS += src/protocol/tripple_uart_cmd.c

# Phase 9: Main Controller + SW Overcurrent Protection
C_SRCS += src/controller/main_controller.c \
          src/controller/sw_oc_protection.c

# Phase 10: Parameter System
C_SRCS += src/controller/parameter/parameter.c \
          src/controller/parameter/param_cmd_handler.c

ASM_SRCS = startup/startup_S32K116.S

# BOOTHOSTED=1 builds the image that runs behind the 16 KB bootloader, linked at
# 0x4000 and without a Flash Configuration Field. The flat image (default) is the
# one a probe programs directly at 0x0000. The two are not interchangeable: a flat
# image will not boot from the bootloader, and a hosted image will not boot alone.
ifeq ($(BOOTHOSTED),1)
LINKER  = linker/S32K116_17_app4000.ld
TARGET  = nxp_bms_app4000
BUILD   = build/app4000
# --nmagic turns off page alignment of segments. Without it ld emits a first LOAD
# segment padded back to a 64 KB-aligned base -- virtual address 0x00000000, size
# 0x40C0 -- for a .interrupts section that actually lives at 0x4000. Programming
# that ELF therefore writes zeros over the whole bootloader region, including its
# Flash Configuration Field and device binding. The .bin is unaffected, which is why
# over-the-air transfers were always correct and only SWD programming destroyed
# things.
LDFLAGS_EXTRA = -Wl,--nmagic -Wl,-z,max-page-size=4
else
LINKER  = linker/S32K116_17_flash.ld
endif

$(shell mkdir -p $(BUILD))

# Flags
CFLAGS  = -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -std=c99
CFLAGS += -Wall -Wextra -Og -g
CFLAGS += -DCPU_S32K116 -DSTART_FROM_FLASH
CFLAGS += -isystem include
CFLAGS += -Isrc -Isrc/hal -Isrc/bq7690x -Isrc/controller -Isrc/controller/parameter -Isrc/protocol -Isrc/common
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += $(EXTRA_CFLAGS)
# Emit a .d file beside every object listing the headers it used, so editing a header
# rebuilds the sources that include it.  Without this, `make` only ever noticed .c edits:
# on 2026-09-23 four edited headers left a mix of objects hours apart in one image, whose
# code differed from a clean build in ways that took a byte-level diff to find.  -MP adds
# phony targets for each header so a deleted or renamed one does not wedge the build.
CFLAGS  += -MMD -MP
ASFLAGS_DEP = -MMD -MP

ASFLAGS = -mcpu=cortex-m0plus -mthumb
ASFLAGS += -DCPU_S32K116 -DSTART_FROM_FLASH -D__START=main
ASFLAGS += -isystem include

LDFLAGS = -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft
LDFLAGS += -T $(LINKER)
LDFLAGS += -nostdlib --specs=nosys.specs --specs=nano.specs
LDFLAGS += -Wl,--gc-sections -Wl,-Map=$(BUILD)/$(TARGET).map $(LDFLAGS_EXTRA)
LDLIBS  = -lc -lgcc -lm

# Objects
C_OBJS   = $(patsubst %.c,$(BUILD)/%.o,$(C_SRCS))
ASM_OBJS = $(patsubst %.S,$(BUILD)/%.o,$(ASM_SRCS))
OBJS     = $(C_OBJS) $(ASM_OBJS)
DEPS     = $(OBJS:.o=.d)

# OpenOCD
OPENOCD     = $(HOME)/openocd-code/src/openocd
OPENOCD_SCRIPTS = $(HOME)/openocd-code/tcl
OPENOCD_CFG = openocd/s32k116.cfg

GDB     = $(TOOLCHAIN)gdb

.PHONY: flash_004_flat flash_004_hosted all clean flash flash_openocd unsecure openocd gdbtui \
       flash_004

all: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).bin $(BUILD)/$(TARGET).hex
	@$(SIZE) $(BUILD)/$(TARGET).elf

$(BUILD)/$(TARGET).elf: $(OBJS)
	@mkdir -p $(dir $@)
	$(LD) $(LDFLAGS) -o $@ $^ $(LDLIBS)

$(BUILD)/$(TARGET).bin: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

$(BUILD)/$(TARGET).hex: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c -o $@ $<

$(BUILD)/%.o: %.S
	@mkdir -p $(dir $@)
	$(AS) $(ASFLAGS) $(ASFLAGS_DEP) -c -o $@ $<

# Pull in the generated header dependencies.  The leading dash keeps the first build,
# where no .d files exist yet, from complaining.
-include $(DEPS)

clean:
	rm -rf $(BUILD)

flash flash_openocd: $(BUILD)/$(TARGET).elf
	$(OPENOCD) -s $(OPENOCD_SCRIPTS) -f $(OPENOCD_CFG) \
		-c "init" \
		-c "kinetis mdm halt" \
		-c "halt" \
		-c "kinetis disable_wdog" \
		-c "kinetis fopt 0x7f" \
		-c "program $(BUILD)/$(TARGET).elf verify" \
		-c "reset run" \
		-c "shutdown"
	@sleep 1
	@echo "Clearing DAP debug power request (required for VLPS)..."
	$(OPENOCD) -s $(OPENOCD_SCRIPTS) -f $(OPENOCD_CFG) \
		-c "init" \
		-c "s32k.dap dpreg 4 0x00000000" \
		-c "shutdown"

# Recovery: mass-erase via MDM-AP. Use when chip is in secured state
# (OpenOCD prints "Your Kinetis MCU is in secured state").
unsecure:
	$(OPENOCD) -s $(OPENOCD_SCRIPTS) -f $(OPENOCD_CFG) \
		-c "init" \
		-c "kinetis mdm mass_erase" \
		-c "shutdown"

openocd:
	$(OPENOCD) -s $(OPENOCD_SCRIPTS) -f $(OPENOCD_CFG)

gdbtui: $(BUILD)/$(TARGET).elf
	$(GDB) -tui -x openocd.gdb

# SWD over an MQTT proxy, for a board that is not on this desk. Used by the
# flash_004* and *_017 targets below; DAP_PORT selects the proxy instance.
#
# The firmware runs in VLPS, and over the high-latency MQTT link the core cannot
# be reliably caught coming out of reset (nRESET/SYSRESETREQ + vector-catch
# timing keep missing), so `kinetis mdm halt` / `reset halt` time out against a
# sleeping core. A mass erase blanks the flash, leaving the core halted and
# catchable, after which programming succeeds -- which is why every remote
# target here erases first. Local flash_openocd via MCU-Link does not need it;
# its low latency catches the core directly.
#
# If the probe's DAP stops responding ("CMD_INFO failed") after aborted halts,
# reboot it: publish "reboot" to <topic_prefix>/cmd over MQTT.
DAP_PORT ?= 4444
DAP_FLAGS = \
	-c "adapter driver cmsis-dap" \
	-c "cmsis-dap backend tcp" \
	-c "cmsis-dap tcp host 127.0.0.1" \
	-c "cmsis-dap tcp port $(DAP_PORT)" \
	-c "cmsis-dap tcp min_timeout 300" \
	-c "adapter speed 1000" \
	-c "transport select swd" \
	-f target/s32k.cfg \
	-c "reset_config srst_only srst_nogate"

# `kinetis fopt 0x7f` is load-bearing in every programming sequence below. It is
# what lets OpenOCD write the Flash Config Field (0x400-0x40F, including the FSEC
# security byte at 0x40C). An earlier version used `flash write_image erase`
# WITHOUT fopt: it failed with "Flash write error at 0x00000400", left FSEC blank
# (0xFF = secured-by-default), and put dap-003 into secured state (MDM-AP STATUS
# bit2=1) so the core could no longer be examined over SWD. Recovery was a
# mass erase through the MDM-AP, which `unsecure` still does. A correct flash
# leaves FCF @0x40C = 0xFE (unsecured). Do not drop fopt.

# One-command flash for the dap-004 wireless probe (port 4445).
# Does clean rebuild + a SINGLE-session mass-erase + program. Two reasons it
# differs from the plain local sequence:
#  1) clean rebuild -- belt and braces.  The original reason was that incremental
#     make missed header edits and silently flashed old code; that is fixed now
#     (-MMD -MP above), but a clean build before flashing hardware is cheap and
#     makes the [BUILD] stamp on the console trustworthy.
#  2) single session -- the target runs in VLPS, and a fresh `kinetis mdm halt`
#     times out against a sleeping core. Mass-erasing
#     first blanks the flash and leaves the core halted, so program/verify works
#     in the SAME session. fopt 0x7f keeps the FCF/FSEC correct (see note above).
# If the probe DAP wedges ("CMD_INFO failed"), publish "reboot" to
# wireless-cmsis-dap-004/cmd over MQTT, then re-run.
# --- programming dap-004 over SWD ----------------------------------------------
#
# There are two incompatible layouts and the wrong one is destructive, so there is
# no bare `flash_004` any more -- it names the layout or it does nothing.
#
#   flash_004_flat     a board with no bootloader. The flat image owns 0x0000 and
#                      provides the Flash Configuration Field.
#   flash_004_hosted   bootloader at 0x0000 plus this application at 0x4000, and the
#                      D-flash descriptor that makes the bootloader willing to start
#                      it. This is the production layout.
#
# The trap this replaces: the old `flash_004` built the flat image by default and
# mass-erased first, so a routine reflash of a bootloader board silently removed the
# bootloader and its device binding. `BOOTHOSTED=1 flash_004` was worse -- the mass
# erase cleared 0x0000 and only the application landed, leaving no vector table and
# a board that would not boot at all.

BOOTLOADER_DIR ?= ../../nxp_bms_bootloader
BOOTLOADER_ELF ?= $(BOOTLOADER_DIR)/build/bootloader-prot-bind/bootloader.elf

flash_004:
	@echo "flash_004 is ambiguous and the wrong choice is destructive. Pick one:"
	@echo "  make flash_004_flat      flat image, no bootloader (owns 0x0000)"
	@echo "  make flash_004_hosted    bootloader + application + descriptor"
	@false

flash_004_flat: DAP_PORT = 4445
flash_004_flat:
	$(MAKE) clean
	$(MAKE) $(BUILD)/$(TARGET).elf
	$(OPENOCD) -s $(OPENOCD_SCRIPTS) $(DAP_FLAGS) \
		-c "init" \
		-c "kinetis mdm mass_erase" \
		-c "halt" \
		-c "kinetis disable_wdog" \
		-c "kinetis fopt 0x7f" \
		-c "program $(BUILD)/$(TARGET).elf verify" \
		-c "reset run" \
		-c "s32k.dap dpreg 4 0x00000000" \
		-c "shutdown"

# Programming the application over SWD does not create the image descriptor the
# bootloader looks for, and the mass erase removes any that was there -- so without
# the descriptor the bootloader would refuse to start a perfectly good application.
# mkdesc.py builds one from the padded image, with the same CRC the over-the-air
# path would have recorded.
flash_004_hosted: DAP_PORT = 4445
flash_004_hosted:
	@test -f $(BOOTLOADER_ELF) || { \
		echo "no bootloader at $(BOOTLOADER_ELF)"; \
		echo "build it: make -C $(BOOTLOADER_DIR) BL_PROTECT=1 BL_BIND_UID=1"; \
		false; }
	$(MAKE) BOOTHOSTED=1 build/app4000/nxp_bms_app4000.bin
	python3 $(BOOTLOADER_DIR)/host/mkdesc.py \
		build/app4000/nxp_bms_app4000.bin build/app4000/desc.bin
	$(OPENOCD) -s $(OPENOCD_SCRIPTS) $(DAP_FLAGS) \
		-c "init" \
		-c "kinetis mdm mass_erase" \
		-c "halt" \
		-c "kinetis disable_wdog" \
		-c "kinetis fopt 0x7f" \
		-c "program $(BOOTLOADER_ELF) verify" \
		-c "flash write_image erase build/app4000/nxp_bms_app4000.bin 0x4000 bin" \
		-c "verify_image build/app4000/nxp_bms_app4000.bin 0x4000 bin" \
		-c "flash write_image build/app4000/desc.bin 0x10000000 bin" \
		-c "reset run" \
		-c "s32k.dap dpreg 4 0x00000000" \
		-c "shutdown"

# Recovery for a board whose SWD access has stalled.
#
# Symptom: a flash target dies right after "SWD DPIDR" with
# "Connecting DP: stalled AP operation, issuing ABORT". The DP answers but the AP will
# not complete, which happens when the core is wedged -- a blank or half-programmed
# chip with no valid vector table, or a fault loop with interrupts masked. A flash that
# mass-erases and then fails partway leaves exactly that state, so this is the remedy
# after an interrupted flash, not an exotic case.
#
# connect_assert_srst holds the target in reset while OpenOCD attaches, so examination
# succeeds without needing the core to be running. The mass erase then goes through the
# MDM-AP, which does not depend on the core at all. Afterwards the chip is blank and an
# ordinary `make flash_004_hosted` works.
recover_004: DAP_PORT = 4445
recover_004:
	$(OPENOCD) -s $(OPENOCD_SCRIPTS) $(DAP_FLAGS) \
		-c "reset_config srst_only srst_nogate connect_assert_srst" \
		-c "init" \
		-c "kinetis mdm mass_erase" \
		-c "shutdown"
	@echo "Chip mass-erased and blank. Now: make flash_004_hosted (or flash_004_flat)."

# --- dap-017: the V2 board ------------------------------------------------------
# Flat image, no bootloader -- bring-up wants as few moving parts as possible.
# Console is on the single UART (LPUART1, PTC9/PTC8) -- see board_pins.h, and note
# the line belongs to binary protocol by default, so uart_017 shows binary frames unless
# the console has been lent with BPCMD_SET_UART_MODE (0x18).
#
# Targets:
#   make proxy_017        start the MQTT<->CMSIS-DAP proxy (idempotent)
#   make proxy_017_stop   stop it
#   make flash_017        proxy + build + mass-erase + program + run
#   make uart_017         capture UART_017_SECS of the UART through the probe
#   make flash_monitor_017  flash_017 then uart_017
#
# dap-017 now has its own `[devices.dap-017]` entry in the proxy's devices.toml
# (added 2026-10-30) with a pinned `$internal-*` ClientID, so no credentials live
# in this repo and a bare `openocd-mqtt-proxy dap-017` can no longer fall back to
# the [defaults] identity and kick another board's proxy off the broker.
PROXY_DIR      ?= $(HOME)/workspace/cmsis_dap_remote_flashing/openocd-mqtt-proxy
PROXY_BIN       = $(PROXY_DIR)/target/release/openocd-mqtt-proxy
SERIAL_BIN      = $(PROXY_DIR)/target/release/serial-monitor
DAP017_DEVICE   = dap-017
DAP017_PORT     = 4471
# NOT under $(BUILD), which a clean removes: that would orphan a running proxy and
# then fail the next start with "Address already in use".
DAP017_PIDFILE  = .proxy_017.pid
UART_017_SECS  ?= 20

.PHONY: proxy_017 proxy_017_stop uart_017 flash_monitor_017

proxy_017:
	@# One shell for the whole thing. Each make recipe line gets its own shell, so an
	@# `exit 0` in a separate line ends only that line -- which is how an earlier version
	@# printed "port already served" and then started a second proxy anyway, giving the
	@# AddrInUse failure and a ClientID fight on the broker. Observed 2026-11-01.
	@set -e; \
	if [ -f $(DAP017_PIDFILE) ] && kill -0 $$(cat $(DAP017_PIDFILE)) 2>/dev/null; then \
		echo "proxy_017: already running (pid $$(cat $(DAP017_PIDFILE)))"; \
	elif ss -ltn 2>/dev/null | grep -q '127.0.0.1:$(DAP017_PORT)'; then \
		echo "proxy_017: port $(DAP017_PORT) already served by another process -- not starting"; \
		echo "proxy_017: find it with: ss -ltnp | grep $(DAP017_PORT)"; \
	else \
		test -x $(PROXY_BIN) || { echo "proxy_017: no binary at $(PROXY_BIN)"; exit 1; }; \
		cd $(PROXY_DIR) && { nohup $(PROXY_BIN) $(DAP017_DEVICE) \
			> $(CURDIR)/proxy_017.log 2>&1 & echo $$! > $(CURDIR)/$(DAP017_PIDFILE); }; \
		up=0; \
		for i in $$(seq 1 20); do \
			if ss -ltn 2>/dev/null | grep -q '127.0.0.1:$(DAP017_PORT)'; then up=1; break; fi; \
			sleep 1; \
		done; \
		if [ $$up -eq 1 ]; then \
			echo "proxy_017: up on 127.0.0.1:$(DAP017_PORT) (pid $$(cat $(CURDIR)/$(DAP017_PIDFILE)))"; \
		else \
			echo "proxy_017: did not come up -- see proxy_017.log"; exit 1; \
		fi; \
	fi

# Kill by recorded pid, never by pattern: a `pkill -f openocd-mqtt-proxy` would
# also match this recipe's own shell and any sibling board's proxy.
proxy_017_stop:
	@if [ -f $(DAP017_PIDFILE) ] && kill -0 $$(cat $(DAP017_PIDFILE)) 2>/dev/null; then \
		kill $$(cat $(DAP017_PIDFILE)) && echo "proxy_017: stopped $$(cat $(DAP017_PIDFILE))"; \
		rm -f $(DAP017_PIDFILE); \
	else \
		echo "proxy_017: not running (no live pid in $(DAP017_PIDFILE))"; \
		rm -f $(DAP017_PIDFILE); \
	fi

# Read the probe's UART bridge. Needs no proxy -- it is a different MQTT topic,
# so this can run while a flash session is in progress. Credentials come from the
# dap-017-serial entry in devices.toml.
uart_017:
	@mkdir -p $(BUILD)
	@test -x $(SERIAL_BIN) || { echo "uart_017: no binary at $(SERIAL_BIN)"; exit 1; }
	@echo "uart_017: capturing $(UART_017_SECS)s -> $(BUILD)/uart_017.log"
	-@cd $(PROXY_DIR) && timeout $(UART_017_SECS) $(SERIAL_BIN) $(DAP017_DEVICE)-serial \
		2>&1 | tee $(CURDIR)/$(BUILD)/uart_017.log
	@echo "uart_017: $$(wc -c < $(BUILD)/uart_017.log) bytes captured"

# Why step 1 retries, and why it is a loop rather than a single shot.
#
# Measured on dap-017, 2026-10-30, flashing over a board that was RUNNING the
# application rather than wedged:
#
#   * `CMD_INFO failed` is MQTT contention, not the target. The binary protocol broadcast
#     is ~720 B/s through the same probe, and it starves the DAP request/response
#     topics. Silencing it with BPCMD_TOGGLE_DEBUG fixed CMD_INFO outright.
#   * `CMD_CONNECT failed` is the application asleep in VLPS gating SWD. A bare
#     DAP connect succeeded 0 of 3 times idle. Poking the UART to hold the core in
#     VLPW (its RX edge clears SLEEPDEEP for 5 s) raised that to 1 of 3 -- luck,
#     not a mechanism, so do not rely on it.
#   * What does work: retry the ERASE until one connect lands. It took 4 attempts
#     over ~45 s. A mass erase goes through the MDM-AP and does not need the core,
#     and once the flash is blank the core is halted and catchable, so step 2 then
#     succeeds first time, deterministically.
#
# So the asymmetry is deliberate: step 1 is cheap and unreliable, so it loops;
# step 2 is expensive and reliable once step 1 has landed, so it runs once.
#
# Step 2 needs minutes, not seconds -- 87 KB over MQTT. Do not wrap it in a short
# timeout; an earlier attempt died at 110 s and looked like a connect failure when
# it was just the clock running out.
ERASE_TRIES ?= 8

define erase_retry
	@landed=0; \
	for i in $$(seq 1 $(ERASE_TRIES)); do \
	  echo "  mass erase attempt $$i/$(ERASE_TRIES)..."; \
	  if $(OPENOCD) -s $(OPENOCD_SCRIPTS) $(DAP_FLAGS) \
	      -c "reset_config srst_only srst_nogate connect_assert_srst" \
	      -c "gdb_port disabled" -c "tcl_port disabled" -c "telnet_port disabled" \
	      -c "init" -c "kinetis mdm mass_erase" -c "shutdown" 2>&1 \
	      | grep -qE "mass erase succeeded|Chip is unsecured"; then \
	    echo "  erase landed on attempt $$i"; landed=1; break; \
	  fi; \
	done; \
	if [ $$landed -eq 0 ]; then \
	  echo "erase never landed in $(ERASE_TRIES) tries."; \
	  echo "Quiet the binary protocol stream first:  python3 host/bpcmd.py cmd BPCMD_TOGGLE_DEBUG 0"; \
	  echo "If CMD_INFO is failing, reboot the probe: publish 'reboot' to wireless-cmsis-dap-017/cmd"; \
	  exit 1; \
	fi
endef

# Why step 2 retries too.
#
# $(erase_retry) covers a probe that will not CONNECT. It does nothing for an SWD
# link that drops PART WAY THROUGH the transfer, which is a different failure with
# a much worse outcome: the erase has already succeeded, so a half-written image
# is now in flash and the board does not come up. Seen on dap-017 2026-11-02 as
#
#   Error: Failed to write memory at 0x40020010
#   ** Verify Failed **
#
# and once before as a failure at 0x14000200. Programming after a landed connect
# is otherwise deterministic, so a plain retry clears it -- but it has to be a
# retry, because leaving a partial image behind is not an acceptable exit.
#
# Takes the ELF to program as $(1).
PROGRAM_TRIES ?= 4

define program_retry
	@landed=0; \
	for i in $$(seq 1 $(PROGRAM_TRIES)); do \
	  echo "  program attempt $$i/$(PROGRAM_TRIES)..."; \
	  if $(OPENOCD) -s $(OPENOCD_SCRIPTS) $(DAP_FLAGS) \
	      -c "gdb_port disabled" -c "tcl_port disabled" -c "telnet_port disabled" \
	      -c "init" -c "halt" \
	      -c "kinetis disable_wdog" -c "kinetis fopt 0x7f" \
	      -c "program $(1) verify" \
	      -c "reset run" -c "s32k.dap dpreg 4 0x00000000" -c "shutdown" 2>&1 \
	      | tee /dev/stderr | grep -q "Verified OK"; then \
	    echo "  programmed and verified on attempt $$i"; landed=1; break; \
	  fi; \
	  echo "  attempt $$i did not verify -- flash now holds a PARTIAL image, retrying"; \
	done; \
	if [ $$landed -eq 0 ]; then \
	  echo "PROGRAMMING NEVER VERIFIED in $(PROGRAM_TRIES) tries."; \
	  echo "The board is left with a PARTIAL IMAGE and will not run. Do not walk away."; \
	  exit 1; \
	fi
endef

# An MCU reset, which is the only way to clear a latched WF_SERVICE: the latch and
# the parameter cache are both RAM-only and re-seeded from defaults at boot. Also
# restores PARAM_WF_GRACE_MS to its default after a run that shortened it.
# Cheaper than a reflash and does not touch flash at all.
.PHONY: reset_017
reset_017: DAP_PORT = $(DAP017_PORT)
reset_017: proxy_017
	@echo "reset_017: MCU reset -- clears latched WF_SERVICE and the parameter cache"
	$(OPENOCD) -s $(OPENOCD_SCRIPTS) $(DAP_FLAGS) \
		-c "gdb_port disabled" -c "tcl_port disabled" -c "telnet_port disabled" \
		-c "init" -c "reset run" -c "s32k.dap dpreg 4 0x00000000" -c "shutdown"

flash_monitor_017: flash_017
	@$(MAKE) --no-print-directory uart_017
# --- hardware probe (src/tools/hw_probe.c) --------------------------------------
# A standalone discovery firmware: no BMS logic, no BQ configuration, read-only
# flash opcodes, and no pin driven as an output except the LPSPI0 signals.  It gets
# its own source list and its own build dir, overridden on the recursive make
# command line so they beat the assignments in this file, which keeps probe objects
# from ever being linked into a real image.
PROBE_SRCS = src/tools/hw_probe.c \
             src/hal/hal_uart.c \
             src/hal/hal_uart1.c \
             src/hal/hal_adc.c \
             src/hal/hal_systime.c \
             startup/startup.c \
             startup/system_S32K116.c \
             startup/syscalls.c

.PHONY: hw_probe
hw_probe:
	$(MAKE) BUILD=build/probe TARGET=hw_probe \
		C_SRCS="$(PROBE_SRCS)" build/probe/hw_probe.elf
	$(SIZE) build/probe/hw_probe.elf

# Flash the probe to the V2 board.  Same OpenOCD sequence as flash_017 -- the mdm
# halt / disable_wdog / fopt order is load-bearing, see the note above flash_004.
flash_017_probe: DAP_PORT = 4471
flash_017_probe: hw_probe
	$(OPENOCD) -s $(OPENOCD_SCRIPTS) $(DAP_FLAGS) \
		-c "gdb_port disabled" -c "tcl_port disabled" -c "telnet_port disabled" \
		-c "init" \
		-c "kinetis mdm halt" \
		-c "halt" \
		-c "kinetis disable_wdog" \
		-c "kinetis fopt 0x7f" \
		-c "program build/probe/hw_probe.elf verify" \
		-c "reset run" \
		-c "s32k.dap dpreg 4 0x00000000" \
		-c "shutdown"

# `kinetis mdm mass_erase`, not `kinetis mdm halt`.
#
# The halt form is what this target used until 2026-10-30 and it cannot be relied
# on over the wireless link: the application runs in VLPS, which gates SWD, so the
# core is only catchable in a brief wake window and `mdm halt` usually times out.
# Observed that day on dap-017 -- DPIDR read fine every attempt, the core examined
# once in six, and no memory read ever succeeded, which OpenOCD reports as
# "probably locked-up in RESET/WDOG loop".
#
# A mass erase blanks the flash, which leaves the core halted and catchable, and
# then programming succeeds. This is the same single-session sequence as
# flash_004_flat, and the reasoning is the note above DAP_PORT.
#
# `kinetis fopt 0x7f` is load-bearing -- dropping it leaves FSEC blank (0xFF =
# secured) and bricks the board out of SWD reach. See the note above flash_004.
#
# Two sessions, because one cannot cover both states the board might be in.
#
# Step 1 erases with SRST asserted. `connect_assert_srst` holds the target in reset
# while OpenOCD attaches, so examination does not need a running core -- and the
# mass erase goes through the MDM-AP, which does not depend on the core at all.
# Without it, `init` dies right after "SWD DPIDR" with "stalled AP operation,
# issuing ABORT" whenever the core is wedged or asleep in VLPS, and then nothing
# downstream runs. Measured on dap-017 on 2026-10-30: a single-session erase
# failed at init on every attempt; with SRST asserted it went through first time.
# This is the same remedy as recover_004, folded in rather than left as a separate
# target you have to remember -- flash_017 flashes a flat image and mass-erases
# anyway, so there is nothing to preserve and no reason to attempt the fragile path
# first.
#
# Step 2 programs a now-blank chip, which leaves the core halted and catchable.
flash_017: DAP_PORT = $(DAP017_PORT)
flash_017: proxy_017
	$(MAKE) $(BUILD)/$(TARGET).elf
	@echo "flash_017: step 1/2 -- mass erase, SRST asserted, retried"
	$(erase_retry)
	@echo "flash_017: step 2/2 -- program and run"
	$(OPENOCD) -s $(OPENOCD_SCRIPTS) $(DAP_FLAGS) \
		-c "gdb_port disabled" -c "tcl_port disabled" -c "telnet_port disabled" \
		-c "init" \
		-c "halt" \
		-c "kinetis disable_wdog" \
		-c "kinetis fopt 0x7f" \
		-c "program $(BUILD)/$(TARGET).elf verify" \
		-c "reset run" \
		-c "s32k.dap dpreg 4 0x00000000" \
		-c "shutdown"

# --- host tests ---------------------------------------------------------------
# The open-wire classifier is deliberately a pure function over a sample struct,
# with no I2C, timers or globals, so it can be compiled for the host and checked
# against the per-tap signatures recorded in the bench journal. That matters
# because the open-wire tests themselves need hands on the tap harness: this is
# the only way the decision logic gets verified without a board.
# $(CC) is the ARM cross-compiler in this Makefile, so the host test needs its
# own compiler or it links against newlib and fails on _exit/_sbrk/_write.
HOSTCC ?= cc

.PHONY: test
test:
	@mkdir -p build/test
	$(HOSTCC) -std=c99 -Wall -Wextra -O1 -g -o build/test/test_cell_open_wire \
		tests/test_cell_open_wire.c src/controller/cell_open_wire.c
	@./build/test/test_cell_open_wire
	$(HOSTCC) -std=c99 -Wall -Wextra -O1 -g -o build/test/test_ntc_bands \
		tests/test_ntc_bands.c src/controller/ntc_bands.c
	@./build/test/test_ntc_bands
