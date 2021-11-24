#ifndef STM32F103_LEGACY_DEFS_H
#define STM32F103_LEGACY_DEFS_H

#if defined(STM32F10X_MD) || defined(STM32F2XX)

#include <application.h>

#define FASTLED_NAMESPACE_BEGIN namespace NSFastLED {
#define FASTLED_NAMESPACE_END }
#define FASTLED_USING_NAMESPACE using namespace NSFastLED;

// reusing/abusing cli/sei defs for due
#define cli()  __disable_irq(); __disable_fault_irq();
#define sei() __enable_irq(); __enable_fault_irq();

#elif defined (__STM32F1__)

#include "cm3_regs.h"

#define cli() nvic_globalirq_disable()
#define sei() nvic_globalirq_enable()

#endif

// pgmspace definitions
#define PROGMEM
#define pgm_read_dword(addr) (*(const unsigned long *)(addr))
#define pgm_read_dword_near(addr) pgm_read_dword(addr)

#endif