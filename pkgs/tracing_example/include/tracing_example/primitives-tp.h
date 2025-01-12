#undef LTTNG_UST_TRACEPOINT_PROVIDER
#define LTTNG_UST_TRACEPOINT_PROVIDER tracing_examples

#undef LTTNG_UST_TRACEPOINT_INCLUDE
#define LTTNG_UST_TRACEPOINT_INCLUDE "tracing_example/primitives-tp.h"

#if !defined(_PRIMITIVES_TP_H) || defined(LTTNG_UST_TRACEPOINT_HEADER_MULTI_READ)
#define _PRIMITIVES_TP_H

#include <lttng/tracepoint.h>

#include <stdint.h>
#include "tracing_example/custom_struct.h"

/**
 * @brief
 * Tracepoint event showcasing integer
 */
LTTNG_UST_TRACEPOINT_EVENT(
    tracing_examples,
    tracepoint_int,
    LTTNG_UST_TP_ARGS(
        int, integer_arg,
        int64_t, int64_arg,
        uint8_t, uint8_arg
    ),
    LTTNG_UST_TP_FIELDS(
        lttng_ust_field_integer(int, integer_field, integer_arg)
        lttng_ust_field_integer(int64_t, int64_field, int64_arg)
        lttng_ust_field_integer(uint8_t, uint8_field, uint8_arg)
    )
)

/**
 * @brief
 * Tracepoint event showcasing floats
 */
LTTNG_UST_TRACEPOINT_EVENT(
    tracing_examples,
    tracepoint_float,
    LTTNG_UST_TP_ARGS(
        float, float_arg,
        double, double_arg
    ),
    LTTNG_UST_TP_FIELDS(
        lttng_ust_field_float(float, float_field, float_arg)
        lttng_ust_field_float(double, double_field, double_arg)
    )
)

/**
 * @brief
 * Tracepoint event showcasing strings
 */
LTTNG_UST_TRACEPOINT_EVENT(
    tracing_examples,
    tracepoint_string,
    LTTNG_UST_TP_ARGS(
        char*, string_arg
    ),
    LTTNG_UST_TP_FIELDS(
        lttng_ust_field_string(string_field, string_arg)
    )
)

/**
 * @brief
 * Tracepoint event showcasing static arrays
 * Display in decimal:
 * - lttng_ust_field_array: store in host byte order
 * - lttng_ust_field_array_network: store in network byte order (big-endian)
 * 
 * Display in hex:
 * - lttng_ust_field_array_hex: store in host byte order
 * - lttng_ust_field_array_network_hex: store in network byte order (big-endian)
 */
#define TRACEPOINT_ARRAY_COUNT 10
LTTNG_UST_TRACEPOINT_EVENT(
    tracing_examples,
    tracepoint_array,
    LTTNG_UST_TP_ARGS(
        int32_t*, int_arr_arg
    ),
    LTTNG_UST_TP_FIELDS(
        lttng_ust_field_array(int32_t, int_arr_field, int_arr_arg, TRACEPOINT_ARRAY_COUNT)
        lttng_ust_field_array_network(int32_t, int_arr_field_network, int_arr_arg, TRACEPOINT_ARRAY_COUNT)
        lttng_ust_field_array_hex(int32_t, int_arr_field_hex, int_arr_arg, TRACEPOINT_ARRAY_COUNT)
        lttng_ust_field_array_network_hex(int32_t, int_arr_field_network_hex, int_arr_arg, TRACEPOINT_ARRAY_COUNT)
    )
)


/**
 * @brief
 * Tracepoint event showcasing static array of char, non-null terminated
 */
#define TRACEPOINT_CHAR_ARRAY_COUNT 10
LTTNG_UST_TRACEPOINT_EVENT(
    tracing_examples,
    tracepoint_char_array,
    LTTNG_UST_TP_ARGS(
        char*, char_arr_arg
    ),
    LTTNG_UST_TP_FIELDS(
        lttng_ust_field_array(char, char_arr_field, char_arr_arg, TRACEPOINT_CHAR_ARRAY_COUNT)
    )
)

/**
 * @brief
 * Tracepoint event showcasing dynamic array (sequence) of integer, text
 * Same suffixes as lttng_ust_array*  applies here.
 * Display in decimal:
 * - lttng_ust_field_sequence: store in host byte order
 * - lttng_ust_field_sequence_network: store in network byte order (big-endian)
 * 
 * Display in hex:
 * - lttng_ust_field_sequence_hex: store in host byte order
 * - lttng_ust_field_sequence_network_hex: store in network byte order (big-endian)
 */
#define TRACEPOINT_CHAR_ARRAY_COUNT 10
LTTNG_UST_TRACEPOINT_EVENT(
    tracing_examples,
    tracepoint_sequence,
    LTTNG_UST_TP_ARGS(
        int32_t*, int_seq_arg,
        size_t, int_seq_cnt,
        char*, char_seq_arg,
        size_t, char_seq_cnt
    ),
    LTTNG_UST_TP_FIELDS(
        lttng_ust_field_sequence(int32_t, int_seq_field, int_seq_arg, size_t, int_seq_cnt)
        lttng_ust_field_sequence_hex(int32_t, int_seq_field_hex, int_seq_arg, size_t, int_seq_cnt)
        lttng_ust_field_sequence_network(int32_t, int_seq_field_network, int_seq_arg, size_t, int_seq_cnt)
        lttng_ust_field_sequence_network_hex(int32_t, int_seq_field_network_hex, int_seq_arg, size_t, int_seq_cnt)
        lttng_ust_field_sequence_text(char, char_seq_field, char_seq_arg, size_t, char_seq_cnt)
    )
)

/**
 * @brief
 * Tracepoint event showcasing passing custom structs
 */
LTTNG_UST_TRACEPOINT_EVENT(
    tracing_examples,
    tracepoint_custom_struct,
    LTTNG_UST_TP_ARGS(
        struct ExampleStruct, example_struct_arg
    ),
    LTTNG_UST_TP_FIELDS(
        lttng_ust_field_float(float, float_field, example_struct_arg.field1)
        lttng_ust_field_integer(int, int_field, example_struct_arg.field2)
    )
)


#endif /* _TP_H */

#include <lttng/tracepoint-event.h>