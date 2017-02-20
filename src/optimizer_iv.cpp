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
 
#include <optimizer_iv.h>

using namespace robotLibPbD;

std::string COptimizerIv::addLine(CVec &first, CVec &second, double width)
{
  std::vector<double> end, start;
  start.push_back(first.x);
  start.push_back(first.y);
  start.push_back(first.z);
  end.push_back(second.x);
  end.push_back(second.y);
  end.push_back(second.z);

   CVec diff(end.at(0) - start.at(0), end.at(1)-start.at(1), end.at(2) - start.at(2));
   CVec linIndep;

   if(diff.x == 0) 
   {
     linIndep.set(1,0,0);
   }   
   else
   { 
     linIndep.set(0,1,0);
   }

   if (diff.x != 0.0)
     {
       linIndep.y = 1.0;
       linIndep.z = 1.0;
       linIndep.x = (-linIndep.y*diff.y - linIndep.z*diff.z) / diff.x;
     }
   else if (diff.y != 0.0)
     {
       linIndep.x = 1.0;
       linIndep.z = 1.0;
       linIndep.y = (-linIndep.x*diff.x - linIndep.z*diff.z) / diff.y;
     }
   else if (diff.z != 0.0)
     {
       linIndep.x = 1.0;
       linIndep.y = 1.0;
       linIndep.z = (-linIndep.x*diff.x - linIndep.y*diff.y) / diff.z;
     }


   linIndep.normalize();
   diff.normalize();


   CVec orth(diff.y * linIndep.z - diff.z * linIndep.y, diff.z * linIndep.x - diff.x - linIndep.z, diff.x * linIndep.y - diff.y * linIndep.w);

   orth = linIndep ^ diff;


   orth.normalize();    

   CMatrix pose;
   //pose.set(linIndep.x, linIndep.y, linIndep.z, 0, diff.x, diff.y, diff.z, 0, orth.x, orth.y, orth.z, 0, 0,0,0,1);
   pose.set(linIndep.x, diff.x, orth.x, 0, linIndep.y, diff.y, orth.y, 0, linIndep.z, diff.z, orth.z, 0, 0, 0, 0, 1);   

 
   CVec axis;
   double angle;

   CMathLib::getRotationFromMatrix(pose, axis, angle);
   axis.normalize();

   std::string file;


  file += "Separator {\n";
  
  file += printToString("Transform { translation %f %f %f\n rotation %f %f %f %f\n }\n",
			start.at(0) + (end.at(0) - start.at(0)) / 2, start.at(1) + (end.at(1) - start.at(1)) / 2, start.at(2) + (end.at(2) - start.at(2)) / 2,
			axis.x, axis.y, axis.z, angle);
   
  file += "Material { ambientColor 0.6 0.6 0.6 diffuseColor 0.6 0.6 0.6 }\n";
  
  file += printToString("Cylinder { radius %f height %f }\n", width, sqrt(pow(end.at(0) - start.at(0),2) + pow(end.at(1) - start.at(1),2) + pow(end.at(2) - start.at(2),2)));

  file += "}\n";  


   return file;  
}

std::string COptimizerIv::getInventorCoordinateSystem(double scaleFactor, std::string ivCoordFilename)
{
  std::string file;
  file += "Separator {\n";

  file += printToString("Transform { scaleFactor %f %f %f }\n", scaleFactor, scaleFactor, scaleFactor);
  
  file += printToString("File { name \"%s\" }\n", ivCoordFilename.c_str());

  file += "}\n";
  return file;
}
 
bool COptimizerIv::generateInventor(std::vector<double> values, std::string filename, std::string ivCoordFilename)
{
  double *st = (double*)calloc(dofs,sizeof(double));
  for (unsigned int i=0; i<dofs && i<values.size(); i++)
    st[i] = values[i];

  std::string text;

  CFrame* tcpFrame;
  if (!frames.getFrameByName("TCP", tcpFrame))
    tcpFrame = NULL;

  CFrame* cameraFrame;
  if (!frames.getFrameByName("Camera", cameraFrame))
    cameraFrame = NULL;

  CVec axis;
  double angle;

  std::string file;
  file += "#Inventor V2.1 ascii\n";
  file += "Separator {\n";	 


  file += "Separator {\n";
   
  file += getInventorCoordinateSystem(ivCoordScale, ivCoordFilename);
  file += "Sphere { radius 10.0 }\n";
  file += "}\n";

  CMatrix pose;

  if (cameraFrame != NULL)
    {
      pose = cameraFrame->getRelativeToBase();
      CMathLib::getRotationFromMatrix(pose, axis, angle);
      
      file += "Separator {\n";
      
      file += printToString("Transform { translation %f %f %f\n rotation %f %f %f %f\n  }\n",
			    pose.a[12], pose.a[13], pose.a[14],
			    axis.x, axis.y, axis.z, angle);
      
      file += getInventorCoordinateSystem(ivCoordScale, ivCoordFilename);
      
      file += "Material { ambientColor 0.5 0.5 0.5 diffuseColor 0.5 0.5 0.5 }\n";
      
      file += printToString("Sphere { radius %f }\n", ivSphereRadius);
      
      file += "}\n"; 
    }

  for (unsigned int i=0; i<examples.size(); i++)
    {  
      setData(examples[i]);
      setValue(st); 

      if (false && tcpFrame != NULL)
	{
      pose = tcpFrame->getRelativeToBase();
      CMathLib::getRotationFromMatrix(pose, axis, angle);
      
      file += "Separator {\n";
      
      file += printToString("Transform { translation %f %f %f\n rotation %f %f %f %f\n    }\n",
			    pose.a[12], pose.a[13], pose.a[14],
			    axis.x, axis.y, axis.z, angle);
      
      file += getInventorCoordinateSystem(ivCoordScale, ivCoordFilename);
      file += "}\n"; 
	}
      // create inventor file 
      
      for (unsigned int j=0; j<globalFunctions.size(); j++) 
	{
	  CMatrix pose = globalFunctions[j]->first->getRelativeToBase();
	  CMathLib::getRotationFromMatrix(pose, axis, angle);
	  
	  file += "Separator {\n";
 
	  file += printToString("Transform { translation %f %f %f\n rotation %f %f %f %f\n   }\n",
				pose.a[12], pose.a[13], pose.a[14],
				axis.x, axis.y, axis.z, angle);
	  
	  file += getInventorCoordinateSystem(ivCoordScale, ivCoordFilename);

	  file += "Material { ambientColor 0.0 0.0 1.0 diffuseColor 0.0 0.0 1.0 }\n";

	  file += printToString("Sphere { radius %f }\n", ivSphereRadius);

	  

	  file += "}\n"; 
	}

      for (unsigned int j=0; j<functions.size(); j++) 
	{
	  CMatrix pose = functions[j]->first->getRelativeToBase();
	  CMathLib::getRotationFromMatrix(pose, axis, angle);
	  
	  file += "Separator {\n";


	  file += addLine(functions[j]->first->getRelativeToBase()[3], functions[j]->second->getRelativeToBase()[3], ivLineWidth);

	  file += printToString("Transform { translation %f %f %f\n rotation %f %f %f %f\n   }\n",
				pose.a[12], pose.a[13], pose.a[14],
				axis.x, axis.y, axis.z, angle);
	  
	  file += getInventorCoordinateSystem(ivCoordScale, ivCoordFilename);

	  file += "Material { ambientColor 0.0 0.0 1.0 diffuseColor 0.0 0.0 1.0 }\n";

	  file += printToString("Sphere { radius %f }\n", ivSphereRadius);

	  

	  file += "}\n"; 

	  pose = functions[j]->second->getRelativeToBase();
	  CMathLib::getRotationFromMatrix(pose, axis, angle);
	  
	  file += "Separator {\n";


	  file += printToString("Transform { translation %f %f %f\n rotation %f %f %f %f\n   }\n",
				pose.a[12], pose.a[13], pose.a[14],
				axis.x, axis.y, axis.z, angle);
	   
	  file += getInventorCoordinateSystem(ivCoordScale, ivCoordFilename);

	  file += "Material { ambientColor 0.0 1.0 0.0 diffuseColor 0.0 1.0 0.0 }\n";

	  file += printToString("Sphere { radius %f }\n", ivSphereRadius);

	  

	  file += "}\n"; 
	  }
      

      for (unsigned int j=0; j<functions.size(); j++) 
	{
	  CVec fpos, spos;
	  CMatrix pose = functions[j]->first->getRelativeToBase();
	  pose.invert();

	  fpos = pose * functions[j]->first->getRelativeToBase()[3];
	  spos = pose * functions[j]->second->getRelativeToBase()[3];

 	  
	  file += "Separator {\n";

	  file += addLine(fpos, spos, ivLineWidth);

	  file += "Material { ambientColor 0.0 1.0 0.0 diffuseColor 0.0 1.0 0.0 }\n";

	  file += printToString("Sphere { radius %f }\n", ivSphereRadius);

	  file += printToString("Transform { translation %f %f %f\n }\n",
				spos.x, spos.y, spos.z);
	  
	  //file += "File { name \"data/SAH-coord.iv\" }\n";

	  

	  file += "}\n"; 
	  }
      

    }


  for (unsigned int j=0; j<globalFunctions.size(); j++) 
    {
      globalFunctions[j]->getDistance();
      CMatrix pose = globalFunctions[j]->first->getRelativeToBase();
      CMathLib::getRotationFromMatrix(pose, axis, angle);
      pose.a[12] = globalFunctions[j]->mean[0];
      pose.a[13] = globalFunctions[j]->mean[1];
      pose.a[14] = globalFunctions[j]->mean[2];
      
      file += "Separator {\n";
      
      file += printToString("Transform { translation %f %f %f\n rotation %f %f %f %f\n   }\n",
			    pose.a[12], pose.a[13], pose.a[14],
			    axis.x, axis.y, axis.z, angle);
      
      file += getInventorCoordinateSystem(ivCoordScale, ivCoordFilename);

      file += "Material { ambientColor 1.0 0.0 0.0 diffuseColor 1.0 0.0 0.0 }\n";

      file += printToString("Sphere { radius %f }\n", ivSphereRadius*1.01);
            
      file += "}\n"; 
    }

  file += "}\n";

  free(st);

  std::ofstream textstream(filename.c_str());
  if (textstream.fail())
    return false;
  
  textstream << file;
  textstream.close(); 

  return true;
}
 
