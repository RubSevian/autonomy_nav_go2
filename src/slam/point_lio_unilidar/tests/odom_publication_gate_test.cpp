#include "odom_publication_gate.h"
#include <cassert>
#include <cmath>
#include <limits>
#include <stdexcept>

int main() {
  OdomPublicationGate gate(100.0);
  assert(gate.ShouldPublish(0));
  assert(!gate.ShouldPublish(9'999'999));
  assert(gate.ShouldPublish(10'000'000));
  // A queued burst can contain source timestamps far apart. Its wall-time
  // output must still be capped, without trying to catch up in a burst.
  assert(!gate.ShouldPublish(10'100'000));
  assert(gate.ShouldPublish(20'000'000));
  OdomPublicationGate unlimited(0.0);
  assert(unlimited.ShouldPublish(0));
  assert(unlimited.ShouldPublish(0));
  for (double bad : {-1.0, 1000.1, std::numeric_limits<double>::quiet_NaN()}) {
    bool rejected = false;
    try { OdomPublicationGate invalid(bad); }
    catch (const std::invalid_argument&) { rejected = true; }
    assert(rejected);
  }
}
