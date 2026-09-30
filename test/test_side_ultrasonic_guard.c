#include <assert.h>
#include <math.h>
#include "side_ultrasonic_guard.h"
int main(void) {
    const float speeds[][2]={{.05f,.05f},{.3f,.3f},{-.05f,.05f},{.2f,0},{0,.2f}};
    for(unsigned i=0;i<sizeof(speeds)/sizeof(speeds[0]);++i) {
        for(unsigned side=0;side<2;++side) {
            assert(side_ultrasonic_blocked(.12f,1,0,side,speeds[i][0],speeds[i][1]));
            assert(!side_ultrasonic_blocked(.12001f,1,0,side,speeds[i][0],speeds[i][1]));
            assert(!side_ultrasonic_blocked(.30f,1,0,side,speeds[i][0],speeds[i][1]));
        }
    }
    assert(side_ultrasonic_blocked(.3f,3,100,true,.1f,.1f));
    assert(side_ultrasonic_blocked(.3f,1,601,true,.1f,.1f));
    assert(side_ultrasonic_blocked(NAN,1,0,true,.1f,.1f));
    assert(!side_ultrasonic_blocked(.03f,3,100,true,-.05f,-.05f));
    assert(!side_ultrasonic_blocked(4,2,100,true,.1f,.1f));
    assert(!side_ultrasonic_blocked(.02f,1,0,true,0,0));
    return 0;
}
