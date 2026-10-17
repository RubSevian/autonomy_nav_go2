#include "bounded_sensor_queue.hpp"
#include <cassert>
#include <deque>
#include <iostream>

int main() {
    std::deque<int> scans{0}, times{1000};
    // Overload while sync holds scan0: newest history remains paired; scan0
    // must not silently become another cloud when sync eventually pops it.
    std::size_t drops = 0;
    for (int i=1;i<=1000;++i) {
        scans.push_back(i); times.push_back(1000+i);
        drops += point_lio::TrimLidarQueue(scans,times,4,true);
        assert(scans.front()==0 && times.front()==1000 && scans.size()<=4);
        for (std::size_t j=0;j<scans.size();++j) assert(times[j]==1000+scans[j]);
    }
    assert(drops==997 && scans.back()==1000 && scans[1]==998);
    scans.pop_front();times.pop_front();
    scans.push_back(1001);times.push_back(2001);
    scans.push_back(1002);times.push_back(2002);
    assert(point_lio::TrimLidarQueue(scans,times,4,false)==1);
    assert(scans.front()==999 && times.front()==1999);
    std::deque<int> imu;
    for(int i=0;i<1000;++i)imu.push_back(i);
    assert(point_lio::TrimImuQueue(imu,512)==488);
    assert(imu.front()==488 && imu.back()==999);
    // Equal cap and invalid/mismatched pairs fail without partially trimming.
    assert(point_lio::TrimLidarQueue(scans,times,4,false)==0);
    auto original=scans;times.pop_back();bool threw=false;
    try {point_lio::TrimLidarQueue(scans,times,4,true);}catch(const std::invalid_argument&){threw=true;}
    assert(threw && scans==original);
    std::cout << "PASS overload: selected scan preserved, latest scans retained, paired timestamps and IMU history bounded\n";
}
