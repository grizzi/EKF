//
// Created by giuseppe on 05/06/18.
//

#ifndef SIMULATOR_UTILS_H
#define SIMULATOR_UTILS_H

template<typename T>
T clamp(T min, T value, T max)
{
    if(value > max)
        return max;

    if(value < min)
        return min;

    return value;

}
#endif //SIMULATOR_UTILS_H
