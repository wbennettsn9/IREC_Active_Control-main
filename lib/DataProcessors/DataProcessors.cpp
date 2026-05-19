#include "DataProcessors.h"
#define _USE_MATH_DEFINES
#include <cmath>


StaticDriftCompensator::StaticDriftCompensator(unsigned long init_time, unsigned int frequency_cutoff){
    time_at_last_change = init_time;
    // Here I convert frequency cutoff to decay parameter d
    // https://tomroelandts.com/articles/low-pass-single-pole-iir-filter
    decay_parameter = exp(-2*M_PI*frequency_cutoff);
}

bool StaticDriftCompensator::check_motion(Vec3 derivative, unsigned long timestamp){
    if (fabs(derivative.x) < threshold  && fabs(derivative.y) < threshold && fabs(derivative.z) < threshold){
            return true;
    }
    time_at_last_change = timestamp;
    return false;
}

Vec3 StaticDriftCompensator::filter(Vec3 input, unsigned long timestamp){
    unsigned long dt = timestamp - time_at_last_change;
    if(this->check_motion(input, timestamp) && dt > 1000){
        // The filter was originally made for descrete regular timesteps, this converts it into fractional steps
        // This however distorts the actual cutoff frequency a bit
        float alpha = decay_parameter*(float)(dt) / 1000000;
        // if we assume that the setpoint is zero, then the remaining signal represents accumulated drift
        low_pass_drift = low_pass_drift + (input - low_pass_drift) * alpha;
    }
    // we take away th drift from the input
    return input - low_pass_drift;
}