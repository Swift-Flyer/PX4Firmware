# include <drivers/drv_rc.h>
# include <systemlib/ppm_decode.h>

#include <board_config.h>

#include "stm32.h"
#include "stm32_gpio.h"
#include "stm32_tim.h"

# define HRT_TIMER_BASE		STM32_TIM3_BASE
# define HRT_TIMER_POWER_REG	STM32_RCC_APB1ENR
# define HRT_TIMER_POWER_BIT   RCC_APB1ENR_TIM3EN
# define HRT_TIMER_VECTOR	STM32_IRQ_TIM3
# define HRT_TIMER_CLOCK	STM32_APB1_TIM3_CLKIN

#define REG(_reg)	(*(volatile uint32_t *)(HRT_TIMER_BASE + _reg))

#define rCR1     	REG(STM32_GTIM_CR1_OFFSET)
#define rCR2     	REG(STM32_GTIM_CR2_OFFSET)
#define rSMCR    	REG(STM32_GTIM_SMCR_OFFSET)
#define rDIER    	REG(STM32_GTIM_DIER_OFFSET)
#define rSR      	REG(STM32_GTIM_SR_OFFSET)
#define rEGR     	REG(STM32_GTIM_EGR_OFFSET)
#define rCCMR1   	REG(STM32_GTIM_CCMR1_OFFSET)
#define rCCMR2   	REG(STM32_GTIM_CCMR2_OFFSET)
#define rCCER    	REG(STM32_GTIM_CCER_OFFSET)
#define rCNT     	REG(STM32_GTIM_CNT_OFFSET)
#define rPSC     	REG(STM32_GTIM_PSC_OFFSET)
#define rARR     	REG(STM32_GTIM_ARR_OFFSET)
#define rCCR1    	REG(STM32_GTIM_CCR1_OFFSET)
#define rCCR2    	REG(STM32_GTIM_CCR2_OFFSET)
#define rCCR3    	REG(STM32_GTIM_CCR3_OFFSET)
#define rCCR4    	REG(STM32_GTIM_CCR4_OFFSET)
#define rDCR     	REG(STM32_GTIM_DCR_OFFSET)
#define rDMAR    	REG(STM32_GTIM_DMAR_OFFSET)

# define rCCR_HRT	rCCR2			/* compare register for HRT */
# define DIER_HRT	GTIM_DIER_CC2IE		/* interrupt enable for HRT */
# define SR_INT_HRT	GTIM_SR_CC2IF		/* interrupt status for HRT */

#  define rCCR_PPM	rCCR2			/* capture register for PPM */
#  define DIER_PPM	GTIM_DIER_CC2IE		/* capture interrupt (non-DMA mode) */
#  define SR_INT_PPM	GTIM_SR_CC2IF		/* capture interrupt (non-DMA mode) */
#  define SR_OVF_PPM	GTIM_SR_CC2OF		/* capture overflow (non-DMA mode) */
#  define CCMR1_PPM	0x100//2			/* not on TI1/TI2 */
#  define CCMR2_PPM	0			/* on TI3, not on TI4 */
#  define CCER_PPM	(GTIM_CCER_CC2E | GTIM_CCER_CC2P | GTIM_CCER_CC2NP) /* CC2, both edges */
#  define CCER_PPM_FLIP	GTIM_CCER_CC2P

#define GPIO_PPM_IN            (GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN7)

static int		rc_tim_isr(int irq, void *context);

void rc_init(void)
{
	irq_attach(HRT_TIMER_VECTOR, rc_tim_isr);

	/* clock/power on our timer */
	modifyreg32(HRT_TIMER_POWER_REG, 0, HRT_TIMER_POWER_BIT);

	/* disable and configure the timer */
	rCR1 = 0;
	rCR2 = 0;
	rSMCR = 0;
	rDIER = DIER_HRT | DIER_PPM;
	rCCER = 0;		/* unlock CCMR* registers */
	rCCMR1 = CCMR1_PPM;
	rCCMR2 = CCMR2_PPM;
	rCCER = CCER_PPM;
	rDCR = 0;

	/* configure the timer to free-run at 1MHz */
	rPSC = (HRT_TIMER_CLOCK / 1000000) - 1;	/* this really only works for whole-MHz clocks */

	/* run the full span of the counter */
	rARR = 0xffff;

	/* set an initial capture a little ways off */
	rCCR_HRT = 1000;

	/* generate an update event; reloads the counter, all registers */
	rEGR = GTIM_EGR_UG;

	/* enable the timer */
	rCR1 = GTIM_CR1_CEN;

	/* enable interrupts */
	up_enable_irq(HRT_TIMER_VECTOR);
	
	ppm_input_init(0xffff);
	stm32_configgpio(GPIO_PPM_IN);
}



static int
rc_tim_isr(int irq, void *context)
{
	uint32_t status;

	/* copy interrupt status */
	status = rSR;

	/* ack the interrupts we just read */
	rSR = ~status;

#if 1
	/* was this a PPM edge? */
	if (status & (SR_INT_PPM | SR_OVF_PPM)) {
		/* if required, flip edge sensitivity */
		ppm_input_decode(status & SR_OVF_PPM, rCCR_PPM);
	}
#endif
	return OK;
}