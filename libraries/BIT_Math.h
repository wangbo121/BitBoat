/*
 *@File     : BIT_Math.h
 *@Author   : wangbo
 *@Date     : Jun 3, 2018
 *@Copyright: 2018 Beijing Institute of Technology. All rights reserved.
 *@Warning  : 本内容仅限于北京理工大学复杂工业控制实验室内部传阅-禁止外泄以及用于其他商业目的
 */


#ifndef LIBRARIES_BIT_MATH_H_
#define LIBRARIES_BIT_MATH_H_

#include <math.h>

/*
 * Constrain a value to be within the range: low and high
 */
template <typename T>
T constrain_value(const T amt, const T low, const T high);

//inline float constrain_float(const float amt, const float low, const float high)
//{
//    return constrain_value(amt, low, high);
//}
//
//inline int16_t constrain_int16(const int16_t amt, const int16_t low, const int16_t high)
//{
//    return constrain_value(amt, low, high);
//}
//
//inline int32_t constrain_int32(const int32_t amt, const int32_t low, const int32_t high)
//{
//    return constrain_value(amt, low, high);
//}



#endif /* LIBRARIES_BIT_MATH_H_ */
