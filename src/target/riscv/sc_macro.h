#ifndef SC_MACRO_H_
#define SC_MACRO_H_

#include "static_assert.h"

/// Bit mask value with low 'n' bits set
#define LOW_BITS_MASK(n) (~(~0 << (n)))

/// @return expression of TYPE with 'first_bit':'last_bit' bits set
#define MAKE_TYPE_FIELD(TYPE, bits, first_bit, last_bit)     ((((TYPE)(bits)) & LOW_BITS_MASK((last_bit) + 1u - (first_bit))) << (first_bit))

/// Number of array 'arr' elements
#define ARRAY_LEN(arr) (sizeof (arr) / sizeof (arr)[0])

/** Number of bytes need for 'num_bits' bits
@param [in] num_bits number of bits
@return number of bytes for 'num_bits' \f$\lceil {\frac{num\_bits}{CHAR\_BIT}} \rceil\f$.
*/
#define NUM_BITS_TO_SIZE(num_bits) ( ( (size_t)(num_bits) + (CHAR_BIT - 1) ) / CHAR_BIT )

#endif  // SC_MACRO_H_
