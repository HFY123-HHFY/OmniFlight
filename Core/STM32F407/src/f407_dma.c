#include "f407_dma.h"
#include "f407_gpio.h"  /* F407_RCC */

/* DMA2 时钟在 AHB1 总线 bit22 */
#define F407_RCC_AHB1_DMA2          (1UL << 22U)

/*
 * 使能 DMA2 控制器时钟。
 */
void F407_DMA_EnableClock(void)
{
    F407_RCC->AHB1ENR |= F407_RCC_AHB1_DMA2;
}

/*
 * 初始化 DMA 流：
 * 1) 若流已使能则先关闭
 * 2) 清除所有中断标志
 * 3) 配置 CR 寄存器
 * 4) FIFO 默认关闭（直接模式）
 */
void F407_DMA_StreamInit(uint32_t streamBase, uint8_t channel,
                         uint32_t configFlags)
{
    F407_DMA_StreamRegs_t *stream = (F407_DMA_StreamRegs_t *)streamBase;

    if (streamBase == 0U)
    {
        return;
    }

    /* 若已使能，先关闭（有界等待硬件确认，防流异常永卡） */
    if ((stream->CR & F407_DMA_CR_EN) != 0U)
    {
        uint32_t poll = 0U;

        stream->CR &= ~F407_DMA_CR_EN;
        while (((stream->CR & F407_DMA_CR_EN) != 0U) && (poll < 10000UL))
        {
            poll++;
        }
    }

    /* 清除该流对应的中断标志（通过 streamNum 反推需要 HIFCR 的哪个位域，但函数不持有 streamNum，
     * 所以这里只做寄存器级清零，上层调用 ClearStreamFlags 补充） */
    (void)channel;

    /* 写入配置：CHSEL + 通用标志 */
    stream->CR = ((uint32_t)channel << F407_DMA_CR_CHSEL_POS) | configFlags;

    /* FIFO 默认关闭（直接模式） */
    stream->FCR = 0U;
}

/*
 * 配置 DMA 传输地址和长度。
 */
void F407_DMA_StreamSetTransfer(uint32_t streamBase,
                                uint32_t periphAddr,
                                uint32_t memAddr,
                                uint16_t count)
{
    F407_DMA_StreamRegs_t *stream = (F407_DMA_StreamRegs_t *)streamBase;

    if (streamBase == 0U)
    {
        return;
    }

    stream->NDTR = (uint32_t)count;
    stream->PAR  = periphAddr;
    stream->M0AR = memAddr;
}

/*
 * 使能 DMA 流。
 */
void F407_DMA_StreamEnable(uint32_t streamBase)
{
    F407_DMA_StreamRegs_t *stream = (F407_DMA_StreamRegs_t *)streamBase;

    if (streamBase == 0U)
    {
        return;
    }

    stream->CR |= F407_DMA_CR_EN;
}

/*
 * 禁用 DMA 流，轮询等待硬件确认关闭。
 */
void F407_DMA_StreamDisable(uint32_t streamBase)
{
    F407_DMA_StreamRegs_t *stream = (F407_DMA_StreamRegs_t *)streamBase;

    if (streamBase == 0U)
    {
        return;
    }

    if ((stream->CR & F407_DMA_CR_EN) != 0U)
    {
        uint32_t poll = 0U;

        stream->CR &= ~F407_DMA_CR_EN;
        while (((stream->CR & F407_DMA_CR_EN) != 0U) && (poll < 10000UL))
        {
            poll++;
        }
    }
}

/*
 * 查询 DMA 流是否已使能。
 */
uint8_t F407_DMA_StreamIsEnabled(uint32_t streamBase)
{
    F407_DMA_StreamRegs_t *stream = (F407_DMA_StreamRegs_t *)streamBase;

    if (streamBase == 0U)
    {
        return 0U;
    }

    return ((stream->CR & F407_DMA_CR_EN) != 0U) ? 1U : 0U;
}

/*
 * 清除指定 Stream 的所有中断标志。
 * Stream 0~3 → LIFCR, Stream 4~7 → HIFCR。
 */
void F407_DMA_ClearStreamFlags(uint8_t streamNum)
{
    uint32_t mask = F407_DMA_GetClearMask(streamNum);

    if (streamNum <= 3U)
    {
        F407_DMA2->LIFCR = mask;
    }
    else
    {
        F407_DMA2->HIFCR = mask;
    }
}

/*
 * 轮询等待指定 Stream 传输完成（TCIF 置位）。
 *
 * 有界轮询：正常一帧 ~67µs 内完成，上限给足余量；
 * 超时（TIM 停发/流异常）直接返回，由上层下一帧重新配置恢复，
 * 绝不在最高优先级任务里永卡（曾导致全系统饿死）。
 */
void F407_DMA_WaitForComplete(uint8_t streamNum)
{
    uint32_t tcif = F407_DMA_GetTCIF(streamNum);
    volatile uint32_t *isr = (streamNum <= 3U) ? &F407_DMA2->LISR : &F407_DMA2->HISR;
    uint32_t poll = 0U;

    while (((*isr & tcif) == 0U) && (poll < 200000UL))
    {
        poll++;
    }
}
