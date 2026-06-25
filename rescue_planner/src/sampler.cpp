#include "sampler.hpp"

#include <cstdlib>
#include <ctime>

SamplePoint sampleRandomPoint(const WorldModel& world){
    static bool initialized = false;

    if(!initialized){
        srand(time(nullptr));
        initialized = true;
    }

    SamplePoint sample;
    sample.x = -10.0 + 20.0 * ((double)rand()/RAND_MAX);
    sample.y = -8.66 + 17.32 * ((double)rand()/RAND_MAX);

    return sample;
}