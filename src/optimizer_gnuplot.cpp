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
#include <optimizer_gnuplot.h>

using namespace robotLibPbD;

 
bool COptimizerGnuplot::generateGnuPlotCoords(double *st, std::string filenameScript, std::string filenameData)
{
  setData(examples[0]);
  setValue(st);
  std::string text;
  
  CMatrix pose;
  std::vector<CVec> points;
  const double len = 20.0;
  points.push_back(CVec(0.0, 0.0, 0.0));
  points.push_back(CVec(len, 0.0, 0.0));
  points.push_back(CVec(0.0, 0.0, 0.0));
  points.push_back(CVec(0.0, len, 0.0));
  points.push_back(CVec(0.0, 0.0, 0.0));
  points.push_back(CVec(0.0, 0.0, len));
  
  std::vector<int> linestyle;
  linestyle.push_back(1);
  linestyle.push_back(1);
  linestyle.push_back(2);
  linestyle.push_back(2);
  linestyle.push_back(3);
  linestyle.push_back(3);
  
  std::string plotdata;
  for (unsigned int i=0; i<frames.getFrames().size(); i++)
    {
      pose = frames.getFrame(i)->getRelativeToBase();
      for (unsigned int j=0; j<points.size(); j++)
	plotdata += (pose * points[j]).toString() + "\n";
    }
  
  std::ofstream textstream2(filenameData.c_str());
  if (textstream2.fail())
    return false;
  
  textstream2 << plotdata;
  textstream2.close();
  
  std::vector<double> mins(3, 1.0e30);
  std::vector<double> maxs(3, -1.0e30);
  for (unsigned int i=0; i<frames.getFrames().size(); i++)
    {
      pose = frames.getFrame(i)->getRelativeToBase();
      for (unsigned int k=0; k<3; k++)
	if (pose.a[12 + k] < mins[k])
	  mins[k] = pose.a[12 + k];
	else if (pose.a[12 + k] > maxs[k])
	  maxs[k] = pose.a[12 + k];
      
      for (unsigned int j=0; j<points.size(); j+=2)
	{
	  CVec t1, t2;
	  t1 = pose * points[j];
	  t2 = pose * points[j+1]; 
	  text += printToString("set arrow from %f,%f,%f to %f,%f,%f linestyle %d\n",
				t1.x, t1.y, t1.z,
				t2.x, t2.y, t2.z,
				linestyle[i]);
	  
	}
    }
  
  text += "splot 'plot_data' with points ps 0\n";
  text += "pause -1\n";
  
  
  std::ofstream textstream(filenameScript.c_str());
  if (textstream.fail())
    return false;
  
  textstream << text;
  textstream.close();
  return true;
}

bool COptimizerGnuplot::generateGnuPlot(double *st, std::string filenameScript, std::string filenameData)
{
  std::string text;
  std::string plotdata; 
  CVec t1, t2;
  std::vector<CVec> t1start(functions.size());
  for (unsigned int i=0; i<examples.size(); i++)
    { 
      setData(examples[i]);
      setValue(st);
      for (unsigned int j=0; j<functions.size(); j++)
	{
	  functions[j]->getDistance(t1, t2);
 
	  if (i == 0)
	    t1start[j] = t1;

	  text += printToString("set arrow from %f,%f,%f to %f,%f,%f linestyle %d\n",
				t1.x, t1.y, t1.z,
				t2.x, t2.y, t2.z,
				1);

	  plotdata += printToString("%f %f %f\n",
				t1.x, t1.y, t1.z);

	  functions[j]->getDistance(t2);
	  text += printToString("set arrow from %f,%f,%f to %f,%f,%f linestyle %d\n",
				t1start[j].x, t1start[j].y, t1start[j].z,
				t1start[j].x + t2.x, t1start[j].y + t2.y, t1start[j].z + t2.z,
				3);
	} 
    }
   
  text += "splot 'plot_data' with points ps 0\n";
  text += "pause -1\n"; 

  std::ofstream textstream(filenameScript.c_str());
  if (textstream.fail())
    return false;
  
  textstream << text;
  textstream.close();
  
  std::ofstream textstream2(filenameData.c_str());
  if (textstream2.fail())
    return false;
  
  textstream2 << plotdata;
  textstream2.close();
  return true;
}
 
