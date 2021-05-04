/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

//#define CONFIG_STM32_HAVE_IP_DMA_V2

// DMA1 Channel/Stream Selections
//--------------------------------------------//---------------------------//----------------
// redundant, already defined in /platforms/nuttx/Nuttx/nuttx/arch/arm/src/stm32/hardware/stm32_dma_v2.h
//#define DMAMAP_SPI2_RX             DMAMAP_SPI2_RX //STM32_DMA_MAP(DMA1,DMA_STREAM3,DMA_CHAN0)		Baro
//#define DMAMAP_SPI2_TX             DMAMAP_SPI2_TX //STM32_DMA_MAP(DMA1,DMA_STREAM3,DMA_CHAN0)		Baro
//#define DMAMAP_USART2_RX           STM32_DMA_MAP(DMA1,DMA_STREAM5,DMA_CHAN4)//		Flow
//#define DMAMAP_UART5_RX            STM32_DMA_MAP(DMA1,DMA_STREAM0,DMA_CHAN4)//		Wifi


//  DMA2 Channel/Stream Selections
//--------------------------------------------//---------------------------//----------------
#define DMAMAP_USART6_RX		DMAMAP_USART6_RX_1 // DMA2, Stream 1, Channel 5				GPS
//#define DMACHAN_SPI1_RX		DMAMAP_SPI1_RX_1   // DMA2, Stream 0, Channel 3			MPU
//#define DMAMAP_SPI1_TX		DMAMAP_SPI1_TX_1   // DMA2, Stream 0, Channel 3			MPU
