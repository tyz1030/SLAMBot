//
// Created by zhsh on 11/27/18.
//

#ifndef BOTLAB_F18_MATH_H
#define BOTLAB_F18_MATH_H

#include <cmath>

template<typename T>
T normalPdf(T x, T mean, T standardDeviation) {
    // from https://stackoverflow.com/a/10848293
    static const T inv_sqrt_2pi = 0.3989422804014327;
    T a = (x - mean) / standardDeviation;

    return inv_sqrt_2pi / standardDeviation * std::exp(-T(0.5) * a * a);
}

#endif //BOTLAB_F18_MATH_H
