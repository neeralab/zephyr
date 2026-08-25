/*
 * Copyright (c) 2026 Neera
 * SPDX-License-Identifier: Apache-2.0
 *
 * XIP-safe XSPI NOR program/erase leaf. Relocated to ITCM via
 * zephyr_code_relocate(); must not call LOG, k_*, or HAL_GetTick.
 */

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <soc.h>
#include <cmsis_core.h>
#include <zephyr/sys/util.h>
#include <zephyr/toolchain.h>

#include "spi_nor.h"
#include "flash_stm32_xspi_xip.h"

/*
 * k_us_to_cyc_ceil32() may dispatch to a non-relocated helper when the
 * runtime frequency path is enabled. Timeouts use the compile-time Hz only.
 */
BUILD_ASSERT(!IS_ENABLED(CONFIG_TIMER_READS_ITS_FREQUENCY_AT_RUNTIME),
	     "FLASH_STM32_XSPI_XIP_SAFE requires fixed SYS_CLOCK_HW_CYCLES_PER_SEC");
BUILD_ASSERT(!IS_ENABLED(CONFIG_SYSTEM_CLOCK_HW_CYCLES_PER_SEC_RUNTIME_UPDATE),
	     "FLASH_STM32_XSPI_XIP_SAFE requires fixed SYS_CLOCK_HW_CYCLES_PER_SEC");

/* Private HAL functional-mode values (same as stm32*_hal_xspi.c). */
#define XSPI_FMODE_INDIRECT_WRITE ((uint32_t)0x00000000U)
#define XSPI_FMODE_AUTO_POLLING   ((uint32_t)XSPI_CR_FMODE_1)
#define XSPI_FMODE_MEMORY_MAPPED  ((uint32_t)XSPI_CR_FMODE)

#define XSPI_IE_ALL                                                                                \
	(XSPI_CR_TOIE | XSPI_CR_SMIE | XSPI_CR_FTIE | XSPI_CR_TCIE | XSPI_CR_TEIE)

#define XSPI_WIP_RETRY_COUNT 3U

static bool xspi_dwt_ready(void)
{
	return (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U;
}

static void xspi_dwt_enable(void)
{
	/* Inline only — must not call into NOR-resident helpers under XIP-off. */
#if defined(DCB)
	DCB->DEMCR |= DCB_DEMCR_TRCENA_Msk;
#elif defined(CoreDebug)
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
#endif
#if defined(CONFIG_CPU_CORTEX_M7)
	/*
	 * Unlock DWT LAR when present. Bit definitions match Armv7-M DWT LSR
	 * (Present=bit0, Access=bit1); avoid depending on CMSIS LAR helpers.
	 */
	if ((DWT->LSR & 0x1U) != 0U) {
		if ((DWT->LSR & 0x2U) != 0U) {
			DWT->LAR = 0xC5ACCE55U;
		}
	}
#endif
	if (!xspi_dwt_ready()) {
		DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	}
}

static bool xspi_wait_flag(XSPI_TypeDef *x, uint32_t flag, bool set, uint32_t cycles)
{
	uint32_t start = DWT->CYCCNT;
	uint32_t last = start;
	uint32_t idle_spins = 0U;
	/* ~4 empty loops per cycle at 600 MHz if DWT is dead. */
	uint32_t spin_limit = cycles * 4U;

	while (true) {
		bool now = (x->SR & flag) != 0U;

		if (now == set) {
			return true;
		}

		if (xspi_dwt_ready()) {
			uint32_t elapsed = DWT->CYCCNT - start;

			if (elapsed > cycles) {
				return false;
			}
			if (DWT->CYCCNT != last) {
				last = DWT->CYCCNT;
				idle_spins = 0U;
			} else {
				idle_spins++;
			}
		} else {
			idle_spins++;
		}

		if (idle_spins > spin_limit) {
			return false;
		}
	}
}

static void xspi_cr_clear_modes(XSPI_TypeDef *x)
{
	CLEAR_BIT(x->CR, XSPI_CR_FMODE | XSPI_CR_APMS | XSPI_CR_PMM);
}

/*
 * Abort any indirect/auto-poll transfer and leave the peripheral idle.
 * Must run before restoring memory-mapped mode.
 */
static void xspi_force_ready(XSPI_TypeDef *x, uint32_t cycles)
{
	CLEAR_BIT(x->CR, XSPI_IE_ALL);

	if ((x->CR & XSPI_CR_DMAEN) != 0U) {
		CLEAR_BIT(x->CR, XSPI_CR_DMAEN);
	}

	if (((x->SR & XSPI_SR_BUSY) != 0U) ||
	    ((x->CR & (XSPI_CR_FMODE | XSPI_CR_APMS)) != 0U)) {
		SET_BIT(x->CR, XSPI_CR_ABORT);
		(void)xspi_wait_flag(x, XSPI_SR_TCF, true, cycles);
		x->FCR = XSPI_FCR_CTCF | XSPI_FCR_CSMF | XSPI_FCR_CTEF;
		(void)xspi_wait_flag(x, XSPI_SR_BUSY, false, cycles);
	}

	xspi_cr_clear_modes(x);
	CLEAR_BIT(x->CR, XSPI_CR_ABORT);
	(void)xspi_wait_flag(x, XSPI_SR_BUSY, false, cycles);
}

static void xspi_config_cmd(XSPI_TypeDef *x, const XSPI_RegularCmdTypeDef *cmd)
{
	__IO uint32_t *ccr = &x->CCR;
	__IO uint32_t *tcr = &x->TCR;
	__IO uint32_t *ir = &x->IR;
	__IO uint32_t *abr = &x->ABR;

	/*
	 * Writing IR (or AR, below) is the trigger — it starts the bus
	 * transaction under whatever FMODE is *already* in CR. Callers must
	 * set FMODE (and PSMAR/PSMKR/PIR for auto-polling) before calling
	 * this, matching HAL_XSPI_Command/HAL_XSPI_AutoPolling ordering.
	 */

	*ccr = cmd->DQSMode;

	if (cmd->AlternateBytesMode != HAL_XSPI_ALT_BYTES_NONE) {
		*abr = cmd->AlternateBytes;
		MODIFY_REG((*ccr), (XSPI_CCR_ABMODE | XSPI_CCR_ABDTR | XSPI_CCR_ABSIZE),
			   (cmd->AlternateBytesMode | cmd->AlternateBytesDTRMode |
			    cmd->AlternateBytesWidth));
	}

	MODIFY_REG((*tcr), XSPI_TCR_DCYC, cmd->DummyCycles);

	if (cmd->DataMode != HAL_XSPI_DATA_NONE) {
		x->DLR = cmd->DataLength - 1U;
		if (cmd->DataDTRMode == HAL_XSPI_DATA_DTR_ENABLE) {
			CLEAR_BIT(x->TCR, XSPI_TCR_SSHIFT);
		}
	}

	if (cmd->InstructionMode != HAL_XSPI_INSTRUCTION_NONE) {
		if (cmd->AddressMode != HAL_XSPI_ADDRESS_NONE) {
			if (cmd->DataMode != HAL_XSPI_DATA_NONE) {
				MODIFY_REG((*ccr),
					   (XSPI_CCR_IMODE | XSPI_CCR_IDTR | XSPI_CCR_ISIZE |
					    XSPI_CCR_ADMODE | XSPI_CCR_ADDTR | XSPI_CCR_ADSIZE |
					    XSPI_CCR_DMODE | XSPI_CCR_DDTR),
					   (cmd->InstructionMode | cmd->InstructionDTRMode |
					    cmd->InstructionWidth | cmd->AddressMode |
					    cmd->AddressDTRMode | cmd->AddressWidth |
					    cmd->DataMode | cmd->DataDTRMode));
			} else {
				MODIFY_REG((*ccr),
					   (XSPI_CCR_IMODE | XSPI_CCR_IDTR | XSPI_CCR_ISIZE |
					    XSPI_CCR_ADMODE | XSPI_CCR_ADDTR | XSPI_CCR_ADSIZE),
					   (cmd->InstructionMode | cmd->InstructionDTRMode |
					    cmd->InstructionWidth | cmd->AddressMode |
					    cmd->AddressDTRMode | cmd->AddressWidth));
			}
			*ir = cmd->Instruction;
			x->AR = cmd->Address;
		} else {
			if (cmd->DataMode != HAL_XSPI_DATA_NONE) {
				MODIFY_REG((*ccr),
					   (XSPI_CCR_IMODE | XSPI_CCR_IDTR | XSPI_CCR_ISIZE |
					    XSPI_CCR_DMODE | XSPI_CCR_DDTR),
					   (cmd->InstructionMode | cmd->InstructionDTRMode |
					    cmd->InstructionWidth | cmd->DataMode |
					    cmd->DataDTRMode));
			} else {
				MODIFY_REG((*ccr), (XSPI_CCR_IMODE | XSPI_CCR_IDTR | XSPI_CCR_ISIZE),
					   (cmd->InstructionMode | cmd->InstructionDTRMode |
					    cmd->InstructionWidth));
			}
			*ir = cmd->Instruction;
		}
	} else if (cmd->AddressMode != HAL_XSPI_ADDRESS_NONE) {
		if (cmd->DataMode != HAL_XSPI_DATA_NONE) {
			MODIFY_REG((*ccr),
				   (XSPI_CCR_ADMODE | XSPI_CCR_ADDTR | XSPI_CCR_ADSIZE |
				    XSPI_CCR_DMODE | XSPI_CCR_DDTR),
				   (cmd->AddressMode | cmd->AddressDTRMode | cmd->AddressWidth |
				    cmd->DataMode | cmd->DataDTRMode));
		} else {
			MODIFY_REG((*ccr), (XSPI_CCR_ADMODE | XSPI_CCR_ADDTR | XSPI_CCR_ADSIZE),
				   (cmd->AddressMode | cmd->AddressDTRMode | cmd->AddressWidth));
		}
		x->AR = cmd->Address;
	}
}

static int xspi_abort_memmap(XSPI_TypeDef *x, uint32_t cycles)
{
	xspi_force_ready(x, cycles);

	if ((x->SR & XSPI_SR_BUSY) != 0U) {
		return -EIO;
	}

	return 0;
}

static int xspi_send_nodata(XSPI_TypeDef *x, const XSPI_RegularCmdTypeDef *cmd, uint32_t cycles)
{
	if (!xspi_wait_flag(x, XSPI_SR_BUSY, false, cycles)) {
		xspi_force_ready(x, cycles);
		return -EIO;
	}

	xspi_cr_clear_modes(x);
	xspi_config_cmd(x, cmd);

	if (!xspi_wait_flag(x, XSPI_SR_BUSY, false, cycles)) {
		xspi_force_ready(x, cycles);
		return -EIO;
	}
	x->FCR = XSPI_FCR_CTCF;
	return 0;
}

static int xspi_auto_poll(XSPI_TypeDef *x, const XSPI_RegularCmdTypeDef *cmd_rdsr,
			  uint32_t match, uint32_t mask, uint32_t cycles)
{
	if (!xspi_wait_flag(x, XSPI_SR_BUSY, false, cycles)) {
		xspi_force_ready(x, cycles);
		return -EIO;
	}

	/*
	 * FMODE/PSMAR/PSMKR/PIR must be live *before* xspi_config_cmd() writes
	 * IR/AR (the trigger) — else the RDSR would launch under whatever
	 * FMODE was previously set (indirect write) and auto-polling would
	 * never actually engage.
	 */
	x->PSMAR = match;
	x->PSMKR = mask;
	x->PIR = SPI_NOR_AUTO_POLLING_INTERVAL;
	MODIFY_REG(x->CR, (XSPI_CR_PMM | XSPI_CR_APMS | XSPI_CR_FMODE),
		   (HAL_XSPI_MATCH_MODE_AND | HAL_XSPI_AUTOMATIC_STOP_ENABLE |
		    XSPI_FMODE_AUTO_POLLING));

	xspi_config_cmd(x, cmd_rdsr);

	if (!xspi_wait_flag(x, XSPI_SR_SMF, true, cycles)) {
		xspi_force_ready(x, cycles);
		return -EIO;
	}
	x->FCR = XSPI_FCR_CSMF;
	xspi_cr_clear_modes(x);
	return 0;
}

static int xspi_transmit(XSPI_TypeDef *x, const XSPI_RegularCmdTypeDef *cmd_pp,
			 const uint8_t *data, size_t len, uint32_t cycles)
{
	__IO uint8_t *dr = (__IO uint8_t *)&x->DR;
	size_t left = len;

	/*
	 * Never do `XSPI_RegularCmdTypeDef cmd = *cmd_pp` here: GCC emits a
	 * libc memcpy() veneer into NOR (.text @ 0x9xxxxxxx). Memmap is already
	 * aborted in this window, so that call hangs/faults — typical DFU stall
	 * at 0–1%. Caller already set Address/DataLength on a stack cmd.
	 */
	if (!xspi_wait_flag(x, XSPI_SR_BUSY, false, cycles)) {
		xspi_force_ready(x, cycles);
		return -EIO;
	}

	xspi_cr_clear_modes(x);
	xspi_config_cmd(x, cmd_pp);

	while (left > 0U) {
		if (!xspi_wait_flag(x, XSPI_SR_FTF, true, cycles)) {
			xspi_force_ready(x, cycles);
			return -EIO;
		}
		*dr = *data++;
		left--;
	}

	if (!xspi_wait_flag(x, XSPI_SR_TCF, true, cycles)) {
		xspi_force_ready(x, cycles);
		return -EIO;
	}
	x->FCR = XSPI_FCR_CTCF;
	xspi_cr_clear_modes(x);
	return 0;
}

static int xspi_wait_wip(XSPI_TypeDef *x, const XSPI_RegularCmdTypeDef *cmd_rdsr, uint32_t cycles)
{
	for (uint32_t attempt = 0U; attempt < XSPI_WIP_RETRY_COUNT; attempt++) {
		int ret = xspi_auto_poll(x, cmd_rdsr, SPI_NOR_MEM_RDY_MATCH,
					 SPI_NOR_MEM_RDY_MASK, cycles);

		if (ret == 0) {
			return 0;
		}
		xspi_force_ready(x, cycles);
	}

	return -EIO;
}

static void xspi_restore_mmap(XSPI_HandleTypeDef *hxspi,
			      const struct flash_stm32_xspi_mmap_regs *mmap,
			      uint32_t cycles)
{
	XSPI_TypeDef *x = hxspi->Instance;

	xspi_force_ready(x, cycles);
	(void)xspi_wait_flag(x, XSPI_SR_BUSY, false, cycles);

	x->CCR = mmap->ccr;
	x->TCR = mmap->tcr;
	x->IR = mmap->ir;
	x->ABR = mmap->abr;
	x->WCCR = mmap->wccr;
	x->WTCR = mmap->wtcr;
	x->WIR = mmap->wir;
	x->WABR = mmap->wabr;

	CLEAR_BIT(x->CR, XSPI_CR_TCEN);
	MODIFY_REG(x->CR, XSPI_CR_FMODE, XSPI_FMODE_MEMORY_MAPPED);
	hxspi->State = HAL_XSPI_STATE_BUSY_MEM_MAPPED;

	__DSB();
	__ISB();
}

static int xspi_xip_window(XSPI_HandleTypeDef *hxspi,
			   const struct flash_stm32_xspi_mmap_regs *mmap,
			   const XSPI_RegularCmdTypeDef *cmd_wren,
			   const XSPI_RegularCmdTypeDef *cmd_rdsr,
			   const XSPI_RegularCmdTypeDef *cmd_op,
			   const uint8_t *data, size_t len,
			   const struct flash_stm32_xspi_xip_timeouts *to)
{
	XSPI_TypeDef *x = hxspi->Instance;
	int ret;

	xspi_dwt_enable();

	ret = xspi_abort_memmap(x, to->abort_cycles);
	if (ret != 0) {
		goto out_restore;
	}

	hxspi->State = HAL_XSPI_STATE_READY;

	ret = xspi_send_nodata(x, cmd_wren, to->cmd_cycles);
	if (ret != 0) {
		goto out_restore;
	}

	ret = xspi_auto_poll(x, cmd_rdsr, SPI_NOR_WREN_MATCH, SPI_NOR_WREN_MASK, to->poll_cycles);
	if (ret != 0) {
		goto out_restore;
	}

	if (data != NULL && len > 0U) {
		ret = xspi_transmit(x, cmd_op, data, len, to->xfer_cycles);
	} else {
		ret = xspi_send_nodata(x, cmd_op, to->cmd_cycles);
	}
	if (ret != 0) {
		goto out_restore;
	}

	ret = xspi_wait_wip(x, cmd_rdsr, to->wip_cycles);

out_restore:
	xspi_restore_mmap(hxspi, mmap, to->abort_cycles);
	return ret;
}

int flash_stm32_xspi_xip_program(XSPI_HandleTypeDef *hxspi,
				 const struct flash_stm32_xspi_mmap_regs *mmap,
				 const XSPI_RegularCmdTypeDef *cmd_wren,
				 const XSPI_RegularCmdTypeDef *cmd_rdsr,
				 const XSPI_RegularCmdTypeDef *cmd_pp,
				 const uint8_t *data, size_t len,
				 const struct flash_stm32_xspi_xip_timeouts *to)
{
	if (hxspi == NULL || mmap == NULL || cmd_wren == NULL || cmd_rdsr == NULL ||
	    cmd_pp == NULL || data == NULL || len == 0U || to == NULL) {
		return -EINVAL;
	}

	return xspi_xip_window(hxspi, mmap, cmd_wren, cmd_rdsr, cmd_pp, data, len, to);
}

int flash_stm32_xspi_xip_erase(XSPI_HandleTypeDef *hxspi,
			       const struct flash_stm32_xspi_mmap_regs *mmap,
			       const XSPI_RegularCmdTypeDef *cmd_wren,
			       const XSPI_RegularCmdTypeDef *cmd_rdsr,
			       const XSPI_RegularCmdTypeDef *cmd_erase,
			       const struct flash_stm32_xspi_xip_timeouts *to)
{
	if (hxspi == NULL || mmap == NULL || cmd_wren == NULL || cmd_rdsr == NULL ||
	    cmd_erase == NULL || to == NULL) {
		return -EINVAL;
	}

	return xspi_xip_window(hxspi, mmap, cmd_wren, cmd_rdsr, cmd_erase, NULL, 0U, to);
}
