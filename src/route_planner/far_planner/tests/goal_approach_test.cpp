#include "far_planner/goal_approach.h"
#include <cassert>
#include <cmath>
int main() {
 using D=GoalApproach::Decision;
 GoalApproach p;
 // Recorded task: reject the metre jump, retain and confirm the nearby point.
 assert(p.Update(1.,.096999,-.283226,.201860,-.526567,-.653458,.038966,.25)==D::CORRECTION_REJECTED);
 assert(p.Update(2.,.096999,-.283226,.201860,-.526567,.251860,-.476567,.25)==D::CORRECTION_WAIT);
 assert(p.Update(2.5,.096999,-.283226,.201860,-.526567,.251860,-.476567,.25)==D::ADJUSTED_WAIT);
 assert(p.Update(2.7,.096999,-.283226,.201860,-.526567,.251860,-.476567,.25)==D::CORRECTION_WAIT);
 assert(p.Update(3.,.096999,-.283226,.201860,-.526567,.251860,-.476567,.25)==D::CONTINUE_ROUTE);
 assert(p.Update(3.2,.12,-.31,.201860,-.526567,.251860,-.476567,.25)==D::ARRIVAL_HOLD);
 // Same B with no remaining follower segment cannot cause repeated motion.
 p.Reset();p.Update(4.,.3,0.,0.,0.,.5,0.,.25);
 assert(p.Update(5.,.3,0.,0.,0.,.5,0.,.25)==D::ADJUSTED_WAIT);
 assert(p.Update(6.,.3,0.,0.,0.,.5,0.,.25)==D::ADJUSTED_WAIT);
 // Changing candidates do not become stable by accumulating small steps.
 p.Reset();
 for(int i=0;i<20;++i)
   assert(p.Update(10.+i*.1,-1.,0.,0.,0.,i*.04,0.,.25)==D::CORRECTION_WAIT);
 assert(p.Update(NAN,0.,0.,0.,0.,0.,0.,.25)==D::CORRECTION_REJECTED);
}
