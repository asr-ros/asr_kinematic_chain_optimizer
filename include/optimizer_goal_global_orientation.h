/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Jäkel Rainer, Wittenbeck Valerij
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/


#ifndef __INC_OPTIMIZER_GOAL_GLOBAL_ORIENTATION
#define __INC_OPTIMIZER_GOAL_GLOBAL_ORIENTATION

#include <vector>  
#include <optimizer_goal_global.h>

namespace robotLibPbD {
 
class OptimizerGoalGlobalOrientation : public OptimizerGoalGlobal
{ 
protected: 
    double angle;
    CVec axis;
    CMatrix pose;
public: 
  virtual void add()
  {  
    item.resize(3); 
    pose = first->getRelativeToBase(); 
    CMathLib::getRotationFromMatrix(pose, axis, angle);
    axis *= 57.295779513082325 * angle;  
    LOG_MSG(6, "Orientation: %s \n", axis.toString().c_str()); 
     
    item[0] = axis.x;
    item[1] = axis.y;
    item[2] = axis.z;
    data.push_back(item);
  }; 
};
 
};

#endif
