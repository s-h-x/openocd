#ifndef SC_MACRO_H_
#define SC_MACRO_H_

#include "static_assert.h"

#define LOW_BITS_MASK(n) (~(~0 << (n)))
#define MAKE_TYPE_FIELD(TYPE, bits, first_bit, last_bit)     ((((TYPE)(bits)) & LOW_BITS_MASK((last_bit) + 1u - (first_bit))) << (first_bit))
#define ARRAY_LEN(arr) (sizeof (arr) / sizeof (arr)[0])
#define NUM_BITS_TO_SIZE(num_bits) ( ( (size_t)(num_bits) + (CHAR_BIT - 1) ) / CHAR_BIT )
#define ARRAY_LEN(arr) (sizeof (arr) / sizeof (arr)[0])

#endif  // SC_MACRO_H_
