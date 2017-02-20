/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Jäkel Rainer, Wittenbeck Valerij
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/


#ifndef __INC_VALUE_SETTER
#define __INC_VALUE_SETTER

#include <vector>  
#include <frame.h>
#include <vecmath.h>
#include <utils.h>

namespace robotLibPbD
{

    class ValueSetter
    {
    public:
      CFrame *frame;
      int x,y,z;
      int a, b, g;
      int qx,qy,qz,qw;

      ValueSetter()
      {
        a = b = g = x = y = z = qx = qw = qy = qz = -1;
      }

      bool set(std::vector<double> &values)
      {
        if (frame == NULL)
          return false;

        CMatrix pose;
        if (a != -1 || b != -1 || g != -1)
        {
            /*double alpha, beta, gamma = 0.0;
            if (a >= 0)
                alpha = values[a];
            if (b >= 0)
                beta = values[b];
            if (g >= 0)
                gamma = values[g];*/
        }
        else
        {
            CVec quater;

            if (qw >= 0)
              quater.w = values[qw];
            if (qx >= 0)
              quater.x = values[qx];
            if (qy >= 0)
              quater.y = values[qy];
            if (qz >= 0)
              quater.z = values[qz];

            CMathLib::matrixFromQuaternion(quater, pose);
        }

        if (x >= 0)
          pose.a[12] = values[x];
        if (y >= 0)
          pose.a[13] = values[y];
        if (z >= 0)
          pose.a[14] = values[z];

        frame->setPose(pose);
        return true;
      }
    };
}

#endif
