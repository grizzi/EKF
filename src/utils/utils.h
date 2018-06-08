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

class NormalDistribution
{
    std::default_random_engine generator_;
    std::normal_distribution<double> dist_;
    double sample_;

    NormalDistribution(double &mean, double &var)
    {
        dist_ = std::normal_distribution<double>(mean, var);
    }

    double draw(){ return dist_(generator_)};

};
#endif //SIMULATOR_UTILS_H
