/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Jäkel Rainer, Wittenbeck Valerij
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/


#ifndef __INC_OPTIMIZER_CONTAINER
#define __INC_OPTIMIZER_CONTAINER


namespace robotLibPbD {

class OptimizerContainer
{
public:
  CFrame *frame;
  std::vector<double> dofs_max, dofs_min;
  std::vector<unsigned int> dofs; 
  void setValue(double *x, unsigned int index) 
  {
    CMatrix pose;
    CVec axis, position;
    
    for (unsigned int i=0; i<dofs.size(); i++)
      {
	LOG_MSG(11, "DOF %d -> %d: %f ( %f %f )\n", i, dofs[i], x[index + i], dofs_min[dofs[i]], dofs_max[dofs[i]]);
      switch (dofs[i])
	{
	case 0:
	  position.x = dofs_min[dofs[i]] + x[index + i] * (dofs_max[dofs[i]] - dofs_min[dofs[i]]);
	  break;
	case 1:
	  position.y = dofs_min[dofs[i]] + x[index + i] * (dofs_max[dofs[i]] - dofs_min[dofs[i]]);
	  break;
	case 2:
	  position.z = dofs_min[dofs[i]] + x[index + i] * (dofs_max[dofs[i]] - dofs_min[dofs[i]]);
	  break;
	case 3:
	  axis.x = dofs_min[dofs[i]] + x[index + i] * (dofs_max[dofs[i]] - dofs_min[dofs[i]]);
	  break;
	case 4:
	  axis.y = dofs_min[dofs[i]] + x[index + i] * (dofs_max[dofs[i]] - dofs_min[dofs[i]]);
	  break;
	case 5:
	  axis.z = dofs_min[dofs[i]] + x[index + i] * (dofs_max[dofs[i]] - dofs_min[dofs[i]]);
	  break;
	};
      }

    double angle = axis.length();
    axis.normalize();
    CMathLib::getMatrixFromRotation(pose, axis, angle);
    pose.a[12] = position.x;
    pose.a[13] = position.y;
    pose.a[14] = position.z;


    frame->setPose(pose);

    LOG_MSG(11, "%s:\n%s", frame->getName(), pose.toString().c_str());
    frame->invalidate();
  };
};

};


#endif
