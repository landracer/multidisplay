/*
 * cplusplus.h
 *
 * Minimal C++ runtime support for AVR (new/delete operators, guard functions).
 * On ESP32 and other platforms with a full C++ runtime, these are not needed.
 *
 * Reference: http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&t=59453
 */

#ifndef CPLUSPLUS_H_
#define CPLUSPLUS_H_

#include <stdlib.h>

#if defined(__AVR__)

void * operator new(size_t size);
void operator delete(void * ptr);

__extension__ typedef int __guard __attribute__((mode (__DI__)));

extern "C" int __cxa_guard_acquire(__guard *);
extern "C" void __cxa_guard_release (__guard *);
extern "C" void __cxa_guard_abort (__guard *);

/* Needed for pure virtual functions on AVR */
extern "C" void __cxa_pure_virtual(void);

#endif /* __AVR__ */

#endif /* CPLUSPLUS_H_ */
