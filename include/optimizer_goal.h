/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Jäkel Rainer, Wittenbeck Valerij
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef __INC_OPTIMIZER_GOAL
#define __INC_OPTIMIZER_GOAL

#include <vector>  
#include <frame.h>
#include <vecmath.h>
#include <log.h>
#include <utils.h>
#include <datapairs.h>

namespace robotLibPbD {

class OptimizerGoal
{
protected:
  DataPairs information;
  std::string function;
public:
  CFrame *first, *second;
  ~OptimizerGoal() {}
  void loadFromXml(CFrameContainer &frames, TiXmlElement* frameNode)
  {
    first = frames.getFrame(CConfiguration::getAttributeString(frameNode, "first", ""));
    second = frames.getFrame(CConfiguration::getAttributeString(frameNode, "second", ""));
    if (second == NULL)
      second = first;

    function = CConfiguration::getAttributeString(frameNode, "value", "");
  };

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

    information.add("x", printToString("%f", pose.a[12]), true);
    information.add("y", printToString("%f", pose.a[13]), true);
    information.add("z", printToString("%f", pose.a[14]), true);
    information.add("rx", printToString("%f", axis.x), true);
    information.add("ry", printToString("%f", axis.y), true);
    information.add("rz", printToString("%f", axis.z), true);

    return atof(information.resolveString(function).c_str());
  };

  void getDistance(CVec &from, CVec &to)
  {
    from = first->getRelativeToBase()[3];
    to = second->getRelativeToBase()[3];
  };

  void getDistance(CVec &to)
  {
    CMatrix pose = first->getRelativeToBase();
    pose.invert();
    to = pose * second->getRelativeToBase()[3];
  };
};
};

#endif
