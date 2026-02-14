/*
 * cplusplus.cpp
 *
 * Minimal C++ runtime support for AVR targets.
 * ESP32 and other platforms with a full C++ runtime do not need these.
 */

#include <cplusplus.h>

#if defined(__AVR__)

 void * operator new(size_t size)
 {
   return malloc(size);
 }

 void operator delete(void * ptr)
 {
   if (ptr != NULL)
	   free(ptr);
 }

 int __cxa_guard_acquire(__guard *g) {return !*(char *)(g);};
 void __cxa_guard_release (__guard *g) {*(char *)g = 1;};
 void __cxa_guard_abort (__guard *) {};

 void __cxa_pure_virtual(void) {};

#endif /* __AVR__ */
