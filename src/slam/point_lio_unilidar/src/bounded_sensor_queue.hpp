#pragma once
#include <cstddef>
#include <stdexcept>

namespace point_lio {
// A selected front scan may be held while sync waits for IMU coverage. Drop
// its successor on overflow, keeping cloud/time pairs and the borrowed front.
template<class Scans, class Times>
std::size_t TrimLidarQueue(Scans& scans, Times& times, std::size_t capacity, bool selected) {
    if (capacity < 2 || scans.size() != times.size())
        throw std::invalid_argument("Invalid paired LiDAR queue");
    std::size_t dropped = 0;
    while (scans.size() > capacity) {
        const auto index = selected ? 1 : 0;
        scans.erase(scans.begin() + index);
        times.erase(times.begin() + index);
        ++dropped;
    }
    return dropped;
}
template<class Samples>
std::size_t TrimImuQueue(Samples& samples, std::size_t capacity) {
    if (capacity < 2) throw std::invalid_argument("Invalid IMU queue");
    std::size_t dropped = 0;
    while (samples.size() > capacity) { samples.pop_front(); ++dropped; }
    return dropped;
}
}
