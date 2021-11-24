#ifndef __FASTPIN_ARM_STM32_H
#define __FASTPIN_ARM_STM32_H

FASTLED_NAMESPACE_BEGIN

#if defined(FASTLED_FORCE_SOFTWARE_PINS)
#warning "Software pin support forced, pin access will be sloightly slower."
#define NO_HARDWARE_PIN_SUPPORT
#undef HAS_HARDWARE_PIN_SUPPORT

#else

/// Template definition for STM32 style ARM pins, providing direct access to the various GPIO registers.  Note that this
/// uses the full port GPIO registers.  In theory, in some way, bit-band register access -should- be faster, however I have found
/// that something about the way gcc does register allocation results in the bit-band code being slower.  It will need more fine tuning.
/// The registers are data output, set output, clear output, toggle output, input, and direction

template<uint8_t PIN, uint8_t _BIT, uint32_t _MASK, typename _GPIO> class _ARMPIN {

public:
    typedef volatile uint32_t * port_ptr_t;
    typedef uint32_t port_t;

    #if 0
    inline static void setOutput() {
        if(_BIT<8) {
            _CRL::r() = (_CRL::r() & (0xF << (_BIT*4)) | (0x1 << (_BIT*4));
        } else {
            _CRH::r() = (_CRH::r() & (0xF << ((_BIT-8)*4))) | (0x1 << ((_BIT-8)*4));
        }
    }
    inline static void setInput() { /* TODO */ } // TODO: preform MUX config { _PDDR::r() &= ~_MASK; }
    #endif

    inline static void setOutput() { pinMode(PIN, OUTPUT); } // TODO: perform MUX config { _PDDR::r() |= _MASK; }
    inline static void setInput() { pinMode(PIN, INPUT); } // TODO: preform MUX config { _PDDR::r() &= ~_MASK; }

#if defined(STM32F2XX)
    inline static void hi() __attribute__ ((always_inline)) { _GPIO::r()->BSRRL = _MASK; }
    inline static void lo() __attribute__ ((always_inline)) { _GPIO::r()->BSRRH = _MASK; }
#else
    inline static void hi() __attribute__ ((always_inline)) { _GPIO::r()->BSRR = _MASK; }
    inline static void lo() __attribute__ ((always_inline)) { _GPIO::r()->BRR = _MASK; }
    // inline static void lo() __attribute__ ((always_inline)) { _GPIO::r()->BSRR = (_MASK<<16); }
#endif
    inline static void set(register port_t val) __attribute__ ((always_inline)) { _GPIO::r()->ODR = val; }

    inline static void strobe() __attribute__ ((always_inline)) { toggle(); toggle(); }

    inline static void toggle() __attribute__ ((always_inline)) { if(_GPIO::r()->ODR & _MASK) { lo(); } else { hi(); } }

    inline static void hi(register port_ptr_t port) __attribute__ ((always_inline)) { hi(); }
    inline static void lo(register port_ptr_t port) __attribute__ ((always_inline)) { lo(); }
    inline static void fastset(register port_ptr_t port, register port_t val) __attribute__ ((always_inline)) { *port = val; }

    inline static port_t hival() __attribute__ ((always_inline)) { return _GPIO::r()->ODR | _MASK; }
    inline static port_t loval() __attribute__ ((always_inline)) { return _GPIO::r()->ODR & ~_MASK; }
    inline static port_ptr_t port() __attribute__ ((always_inline)) { return &_GPIO::r()->ODR; }

#if defined(STM32F2XX)
    inline static port_ptr_t sport() __attribute__ ((always_inline)) { return &_GPIO::r()->BSRRL; }
    inline static port_ptr_t cport() __attribute__ ((always_inline)) { return &_GPIO::r()->BSRRH; }
#else
    inline static port_ptr_t sport() __attribute__ ((always_inline)) { return &_GPIO::r()->BSRR; }
    inline static port_ptr_t cport() __attribute__ ((always_inline)) { return &_GPIO::r()->BRR; }
#endif

    inline static port_t mask() __attribute__ ((always_inline)) { return _MASK; }
};


#define _R(T) struct __gen_struct_ ## T
#define _FL_DEFPIN(PIN, BIT, L) template<> class FastPin<PIN> : public _ARMPIN<PIN, BIT, 1 << BIT, _R(GPIO ## L)> {};

#if defined(STM32F10X_MD)
#define _RD32(T) struct __gen_struct_ ## T { static __attribute__((always_inline)) inline volatile GPIO_TypeDef * r() { return T; } };
#define _FL_IO(L,C) _RD32(GPIO ## L);  _FL_DEFINE_PORT3(L, C, _R(GPIO ## L));

#elif defined(__STM32F1__)
#define _RD32(T) struct __gen_struct_ ## T { static __attribute__((always_inline)) inline gpio_reg_map* r() { return T->regs; } };
#define _FL_IO(L,C) _RD32(GPIO ## L);  _FL_DEFINE_PORT3(L, C, _R(GPIO ## L));

#elif defined(STM32F2XX)
#define _RD32(T) struct __gen_struct_ ## T { static __attribute__((always_inline)) inline volatile GPIO_TypeDef * r() { return T; } };
#define _FL_IO(L,C) _RD32(GPIO ## L);

#elif defined(STM32F1xx)
#define _RD32(T) struct __gen_struct_ ## T { static __attribute__((always_inline)) inline volatile GPIO_TypeDef * r() { return T; } };
#define _FL_IO(L,C) _RD32(GPIO ## L);  _FL_DEFINE_PORT3(L, C, _R(GPIO ## L));
#else
#error "Platform not supported"
#endif


#ifdef GPIOA
_FL_IO(A,0);
#endif
#ifdef GPIOB
_FL_IO(B,1);
#endif
#ifdef GPIOC
_FL_IO(C,2);
#endif
#ifdef GPIOD
_FL_IO(D,3);
#endif
#ifdef GPIOE
_FL_IO(E,4);
#endif
#ifdef GPIOF
_FL_IO(F,5);
#endif
#ifdef GPIOG
_FL_IO(G,6);
#endif


// STM32duino Core support for STM32F103
#if defined (STM32F103x6) || defined (STM32F103xB) || defined (STM32F103xE) || defined (STM32F103xG)
#include "variants/pins/stm32f103_pins.h"

// Legacy support for other arduino cores
#elif defined(__STM32F1__) || defined(SPARK) || defined(STM32F2XX)
#include "variants/pins/stm32f103_legacy_pins.h"

#else
 #error "Board not implemented"
#endif

#endif // FASTLED_FORCE_SOFTWARE_PINS

FASTLED_NAMESPACE_END

#endif // __INC_FASTPIN_ARM_STM32
