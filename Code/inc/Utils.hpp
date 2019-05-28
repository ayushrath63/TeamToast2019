#ifndef __UTILS_HPP__
#define __UTILS_HPP__

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

#endif //__UTILS_HPP__