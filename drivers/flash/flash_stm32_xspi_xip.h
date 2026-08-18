/*
 * Copyright (c) 2026 Neera
 * SPDX-License-Identifier: Apache-2.0
 *
 * XIP-safe MMIO leaf for STM32 XSPI NOR (program/erase while AHB mmap is off).
 * Call only under irq_lock(); code must run from ITCM/RAM, never from the NOR.
 */

#ifndef ZEPHYR_DRIVERS_FLASH_STM32_XSPI_XIP_H_
#define ZEPHYR_DRIVERS_FLASH_STM32_XSPI_XIP_H_

#include <stddef.h>
#include <stdint.h>

#include <soc.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Saved memory-mapped CCR pair (read + write cfg) restored after the window. */
struct flash_stm32_xspi_mmap_regs {
	uint32_t ccr;
	uint32_t tcr;
	uint32_t ir;
	uint32_t abr;
	uint32_t wccr;
	uint32_t wtcr;
	uint32_t wir;
	uint32_t wabr;
};

/** DWT cycle budgets for each phase (no HAL_GetTick under irq_lock). */
struct flash_stm32_xspi_xip_timeouts {
	uint32_t abort_cycles;
	uint32_t cmd_cycles;
	uint32_t poll_cycles;
	uint32_t xfer_cycles;
	uint32_t wip_cycles;
};

/**
 * Abort memmap, WREN+WEL, page-program @data/@len, WIP poll, restore mmap.
 * @data must be in RAM. All command structs must be on the stack (RAM).
 */
int flash_stm32_xspi_xip_program(XSPI_HandleTypeDef *hxspi,
				 const struct flash_stm32_xspi_mmap_regs *mmap,
				 const XSPI_RegularCmdTypeDef *cmd_wren,
				 const XSPI_RegularCmdTypeDef *cmd_rdsr,
				 const XSPI_RegularCmdTypeDef *cmd_pp,
				 const uint8_t *data, size_t len,
				 const struct flash_stm32_xspi_xip_timeouts *to);

/**
 * Abort memmap, WREN+WEL, instruction-only erase, WIP poll, restore mmap.
 */
int flash_stm32_xspi_xip_erase(XSPI_HandleTypeDef *hxspi,
			       const struct flash_stm32_xspi_mmap_regs *mmap,
			       const XSPI_RegularCmdTypeDef *cmd_wren,
			       const XSPI_RegularCmdTypeDef *cmd_rdsr,
			       const XSPI_RegularCmdTypeDef *cmd_erase,
			       const struct flash_stm32_xspi_xip_timeouts *to);

/** Snapshot current mmap command registers (call while memmap still active). */
static inline void flash_stm32_xspi_mmap_save(XSPI_HandleTypeDef *hxspi,
					      struct flash_stm32_xspi_mmap_regs *out)
{
	XSPI_TypeDef *x = hxspi->Instance;

	out->ccr = x->CCR;
	out->tcr = x->TCR;
	out->ir = x->IR;
	out->abr = x->ABR;
	out->wccr = x->WCCR;
	out->wtcr = x->WTCR;
	out->wir = x->WIR;
	out->wabr = x->WABR;
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_FLASH_STM32_XSPI_XIP_H_ */
