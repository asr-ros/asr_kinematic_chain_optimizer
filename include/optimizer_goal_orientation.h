/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Jäkel Rainer, Wittenbeck Valerij
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/


#ifndef __INC_OPTIMIZER_GOAL_ORIENTATION
#define __INC_OPTIMIZER_GOAL_ORIENTATION

#include <vector>  
#include <optimizer_goal.h>

namespace robotLibPbD {



class OptimizerGoalOrientation : public OptimizerGoal
{ 
public: 
  virtual double getDistance() 
  {
    double angle;
    CVec axis;
    CMatrix pose;
    LOG_MSG(10, "First:\n%s", first->getRelativeToBase().toString().c_str());
    LOG_MSG(10, "Second:\n%s", second->getRelativeToBase().toString().c_str());


    pose = first->getRelativeToBase();
    pose.invert();
    pose.mul(pose, second->getRelativeToBase());

    //LOG_MSG(5, "Difference:\n%s", pose.toString().c_str());

    CMathLib::getRotationFromMatrix(pose, axis, angle);
    axis *= angle;  
    
    return 57.295779513082325 * axis.length();
  };
}; 

};

#endif
