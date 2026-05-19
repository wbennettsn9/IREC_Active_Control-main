#include <iostream>
#include <random>
#include <cmath>
#include "DataProcessors.h"

/*
This file allows you to run SITL simulations
Simply switch to env:naitive_sitl and press build. output is sim.csv
*/

struct SimConfig
{
    float drift_rate = 0.02f;
    float noise_std = 0.01f;
    float motion_amplitude = 0.2f;
    float motion_frequency = 2.0f;
    bool inject_motion = false;

    float dt_sec = 0.01f;
    unsigned long dt_us = 10000;
    float duration_sec = 20.0f;

    float cutoff_hz = 1.0f;
};

int main()
{
    SimConfig cfg;
    StaticDriftCompensator lowpass(0, cfg.cutoff_hz);
    std::default_random_engine rng(42); // deterministic seed
    std::normal_distribution<float> noise_dist(0.0f, cfg.noise_std);
    unsigned long timestamp = 0;
    int total_steps = (int)(cfg.duration_sec / cfg.dt_sec);
    std::cout << "time,raw,filtered\n";
    for (int i = 0; i < total_steps; i++)
    {
        float t = i * cfg.dt_sec;
        float noise = noise_dist(rng);
        float motion = 0.0f;
        if (cfg.inject_motion)
        {
            motion = cfg.motion_amplitude *
                     sinf(2.0f * M_PI * cfg.motion_frequency * t);
        }
        float raw_signal = cfg.drift_rate + noise + motion;
        Vec3 input(raw_signal, 0, 0);
        Vec3 output = lowpass.filter(input, timestamp);
        std::cout << t << ","
                  << raw_signal << ","
                  << output.x << "\n";
        timestamp += cfg.dt_us;
    }

    return 0;
}