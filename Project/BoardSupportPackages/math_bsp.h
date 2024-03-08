#pragma once 
#include "main.h"

#define MathBsp_Abs(x) ((x) < 0 ? -(x) : (x))
#define MathBsp_Min(x, y) ((x) < (y) ? (x) : (y))
#define MathBsp_Max(x, y) ((x) > (y) ? (x) : (y))
#define MathBsp_Clamp(x, min, max) MathBsp_Min(MathBsp_Max((x), (min)), (max))


const uint16_t* MathBsp_Crc16Table(void);
uint16_t MathBsp_Crc16(void* pdata, uint32_t byte_len);

#define MATHBSP_TYPEBASED_FUNC_REFERENCE_LIST(reference, func)      \
    reference(double,   func##_double)                              \
    reference(float,    func##_float)                               \
    reference(uint64_t, func##_uint64)                              \
    reference(int64_t,  func##_int64)                               \
    reference(uint32_t, func##_uint32)                              \
    reference(int32_t,  func##_int32)                               \
    reference(uint16_t, func##_uint16)                              \
    reference(int16_t,  func##_int16)                               \
    reference(uint8_t,  func##_uint8)                               \
    reference(int8_t,   func##_int8)

/*
*   归并排序：支持基本类型
*   type* MathBsp_MergeSort_type(type* source, type* buff, uint32_t len); 其中type为基本类型
*   source: 待排序数组
*   buff:   临时缓冲区
*   len:    数组长度
*   返回值: 排序后的数组(指向source)
*/
#define MATHBSP_MERGESORT_REFERENCE(type, func)  type* func(type* source, type* buff, uint32_t len);
MATHBSP_TYPEBASED_FUNC_REFERENCE_LIST(MATHBSP_MERGESORT_REFERENCE, MathBsp_MergeSort)
/*
*   快速排序：支持基本类型
*   type* MathBsp_QuickSort_type(type* source, uint32_t len); 其中type为基本类型
*   source: 待排序数组
*   len:    数组长度
*   返回值: 排序后的数组(指向source)
*/
#define MATHBSP_QUICKSORT_REFERENCE(type, func)  type* func(type* source, uint32_t len);
MATHBSP_TYPEBASED_FUNC_REFERENCE_LIST(MATHBSP_QUICKSORT_REFERENCE, MathBsp_QuickSort)


