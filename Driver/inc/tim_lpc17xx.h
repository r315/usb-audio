#ifndef __tim_lpc17xx_h__
#define __tim_lpc17xx_h__

#include <stdint.h>

/* External match function control*/
#define EMC_0   4
#define EMC_1   6
#define EMC_2   8
#define EMC_3   10

#define EMR_OFF 0  // Do Nothing
#define EMR_CLR 1  // Clear the corresponding External Match bit/output to 0 (MATn.m pin is LOW if pinned out).
#define EMR_SET 2  // Set the corresponding External Match bit/output to 1 (MATn.m pin is HIGH if pinned out).
#define EMR_TGL 3  // Toggle the corresponding External Match bit/output.

// TIM_IR bits
#define TIM_IR_MR0      (1 << 0)
#define TIM_IR_MR1      (1 << 1)
#define TIM_IR_MR2      (1 << 2)
#define TIM_IR_MR3      (1 << 3)
#define TIM_IR_CR0      (1 << 4)
#define TIM_IR_CR1      (1 << 5)

// TIM_TCR Bits
#define TIM_TCR_EN      (1 << 0)
#define TIM_TCR_RST     (1 << 1)

// TIMx MCR bits
#define TIM_MCR_MR0I    (1 << 0)      // Enable interrupt on match
#define TIM_MCR_MR0R    (1 << 1)      // Reset Timer0 on match
#define TIM_MCR_MR0S    (1 << 2)      // Stop Timer0 on match
#define TIM_MCR_MR1I    (1 << 3)
#define TIM_MCR_MR1R    (1 << 4)
#define TIM_MCR_MR1S    (1 << 5)
#define TIM_MCR_MR2I    (1 << 6)
#define TIM_MCR_MR2R    (1 << 7)
#define TIM_MCR_MR2S    (1 << 8)
#define TIM_MCR_MR3I    (1 << 9)
#define TIM_MCR_MR3R    (1 << 10)
#define TIM_MCR_MR3S    (1 << 11)

// TIM_CCR Bits
#define TIM_CCR_CAP0RE  (1 << 0)    // Capture on rising edge
#define TIM_CCR_CAP0FE  (1 << 1)    // Capture on falling edge
#define TIM_MCR_CAP0I   (1 << 2)    // Interrupt on capture
#define TIM_CCR_CAP1RE  (1 << 3)
#define TIM_CCR_CAP1FE  (1 << 4)
#define TIM_MCR_CAP1I   (1 << 5)

// TIM_EMR Bits
#define TIM_EMR_EM0     (1 << 0)
#define TIM_EMR_EM1     (1 << 1)
#define TIM_EMR_EM2     (1 << 2)
#define TIM_EMR_EM3     (1 << 3)


#endif
