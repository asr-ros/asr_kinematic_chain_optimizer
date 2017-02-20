/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Jäkel Rainer, Wittenbeck Valerij
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <cstdio>
#include <iostream>
#include <cstring>
#include <fstream> 
 
#include <optimizer_ros.h>

using namespace robotLibPbD;

 
std::string COptimizerRos::generateOutput()
{
  CFrameInterface interface;
  std::string text;

  text = "<launch>\n";

  unsigned int counter = 0;
  for (unsigned int i=0; i<frames.getFrames().size(); i++)
    {
      interface.setFrame(frames.getFrame(i));
      LOG_VERBOSE("Frame: %s\n", interface.getFrameAsXml().c_str());

      CVec position, quater;
      position = frames.getFrame(i)->getPose()[3];
      CMathLib::quaternionFromMatrix(frames.getFrame(i)->getPose(), quater); 
 
      if (frames.getFrame(i)->isData())
	{
	  text += printToString("# input: %s\n", frames.getFrame(i)->getName());
	} else
	{
	  if (frames.getFrame(i)->getBase() == NULL && frames.getFrame(i)->getRelativeToBase().length() < 1.0e-10)
	    continue;

	  text += printToString("<node pkg=\"tf\" type=\"static_transform_publisher\" name=\"%s\" args=\"%f %f %f %f %f %f %f %s %s 100\"/> \n",
				printToString("calibration_publisher_%d", counter).c_str(),
				position.x / 1000.0, position.y / 1000.0, position.z / 1000.0,
				quater.x, quater.y, quater.z, quater.w,
				frames.getFrame(i)->getBase() == NULL ? "CalibrationBase" : frames.getFrame(i)->getBase()->getName(),
				frames.getFrame(i)->getName());
				counter++;
	}
    }

  text += "</launch>\n";

  return text;
}
 
