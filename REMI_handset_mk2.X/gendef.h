/****************************************************************************************
 *
 * FileName:   gendef.h
 *
 * File contains general definitions common to all PIC18 embedded applications.
 *
 * =======================================================================================
 */
#ifndef GENDEF_H
#define GENDEF_H


//============================== Build Options ===========================================

#define FIXED_POINT_FORMAT_18_14_BITS
//#define FIXED_POINT_FORMAT_20_12_BITS

//=======================================================================================

#define LITTLE_ENDIAN   1   // True for PIC18 and XC8 compiler

typedef signed char         int8;
typedef unsigned char       uint8,  uchar;

typedef signed short        int16;
typedef unsigned short      uint16, ushort;

typedef signed long         int32;
typedef unsigned long       uint32, ulong;

typedef signed long long    int64;
typedef unsigned long long  uint64;

typedef signed long         fixed_t;   // 32-bit fixed point

#ifndef bool
typedef unsigned char       bool;
#endif

typedef void (* pfnvoid)(void);     // pointer to void function

#ifndef NULL
#define NULL ((void *) 0)
#endif

#ifndef FALSE
#define FALSE  0
#define TRUE   (!FALSE)
#endif

#ifndef PRIVATE
#define PRIVATE  static    // for module private functions
#endif

#define until(exp)  while(!(exp))   // Usage:  do { ... } until (exp);
#define NOTHING

#define TEST_BIT(u, bit)   ((u) & (1<<(bit)))
#define SET_BIT(u, bit)    (u) |= (1<<(bit))
#define CLEAR_BIT(u, bit)  (u) &= ~(1<<(bit))

#define SWAP(w)     ((((w) & 0xFF) << 8) | (((w) >> 8) & 0xFF))

#if LITTLE_ENDIAN
#define LSB_MSB(w)  (w)        // LSB is already first
#else
#define LSB_MSB(w)  SWAP(w)    // Swap bytes to put LSB first
#endif

#define HI_BYTE(w)  (((w) >> 8) & 0xFF)   // Extract high-order byte from unsigned word
#define LO_BYTE(w)  ((w) & 0xFF)          // Extract low-order byte from unsigned word

#define LESSER_OF(arg1,arg2)  ((arg1)<=(arg2)?(arg1):(arg2))
#define ARRAY_SIZE(a)  (sizeof(a)/sizeof(a[0]))
#define MIN(x,y)       ((x > y)? y: x)

#if defined FIXED_POINT_FORMAT_18_14_BITS
// Macros for manipulating 32-bit (18:14) fixed-point numbers, type fixed_t.
// Integer part:    18 bits, signed, range +/-128K
// Fractional part: 14 bits, precision: +/-0.0001 (approx.)
//
#define IntToFixedPt(i)     (i << 14)                    // convert int to fixed-pt
#define FloatToFixed(r)     (fixed_t)(r * 16384)         // convert float (r) to fixed-pt
#define FixedToFloat(z)     ((float)z / 16384)           // convert fixed-pt (z) to float
#define IntegerPart(z)      (z >> 14)                    // get integer part of fixed-pt
#define FractionPart(z,n)   ((z & 0x3FFF) >> (14 - n))   // get n MS bits of fractional part
#define MultiplyFixed(v,w)  ((v * w) >> 14)              // product of two fixed-pt numbers^
#define LongMultiplyFixed(v,w)  (((int64)v * w) >> 14)   // product of two numbers > 4 "
//
// ^ Use MultiplyFixed(v,w) where (v * w) <= 4.0, otherwise use LongMultiplyFixed(v,w)
// " Disable IRQ while LongMultiplyFixed(v,w) executes, if an ISR uses the product!
//
#elif defined FIXED_POINT_FORMAT_20_12_BITS
// Macros for manipulating 32-bit (20:12) fixed-point numbers, type fixed_t.
// Integer part:    20 bits, signed, range +/-512K
// Fractional part: 12 bits, precision: +/-0.0005 (approx.)
//
#define IntToFixedPt(i)     (i << 12)                    // convert int to fixed-pt
#define FloatToFixed(r)     (fixed_t)(r * 4096)          // convert float (r) to fixed-pt
#define FixedToFloat(z)     ((float)z / 4096)            // convert fixed-pt (z) to float
#define IntegerPart(z)      (z >> 12)                    // get integer part of fixed-pt
#define FractionPart(z,n)   ((z & 0x0FFF) >> (12 - n))   // get n MS bits of fractional part
#define MultiplyFixed(v,w)  ((v * w) >> 12)              // product of two fixed-pt numbers^
#define LongMultiplyFixed(v,w)  (((int64)v * w) >> 12)   // product of two numbers > 4
//
// ^ Use MultiplyFixed(v,w) where (v * w) <= 4.0, otherwise use LongMultiplyFixed(v,w)
//   Disable IRQ while LongMultiplyFixed(v,w) executes, if an ISR uses the product!
//
#else 
#error "Fixed-point format undefined!"
#endif  // FIXED_POINT_FORMAT


/***** Commonly used symbolic constants *****/

#define ERROR     (-1)
#define SUCCESS     0
#define LOW         0
#define HI          1
#define DISABLE     0
#define ENABLE      1


#endif // GENDEF_H
