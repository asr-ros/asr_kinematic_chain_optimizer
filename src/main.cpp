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

#include <boost/program_options.hpp>

#include <log.h>
#include <frame.h>
#include <datapairs.h>
#include <value_setter.h>
#include <optimizer_goal.h>
#include <optimizer_goal_global_position.h>
#include <optimizer_goal_global_orientation.h>
#include <optimizer_goal_orientation.h>
#include <optimizer_goal_position.h>
#include <optimizer_container.h>
#include <optimizer_iv.h>

using namespace robotLibPbD;
namespace po = boost::program_options;

// main function
int main(int argc, char *argv[])
{    
  std::string filename, initial, cfg;
  unsigned int iterations;
  bool useRandom;
  double trim, epsilon;
  bool useShowResult;
  //unsigned int trimming, examples;
  unsigned int examplesMax;
  unsigned int counterMod; 
  double ivLineWidth, ivSphereRadius, ivCoordScale;
  

  po::options_description desc("Usage : kinematic_chain_optimizer [options]");
  desc.add_options()
    ( "help,h","show help screen")
    ( "exit","show only initial error")
    ( "filename",po::value<std::string>(&filename)->default_value("data/calib_dump"),"load data from filename")
    ( "initial",po::value<std::string>(&initial)->default_value(""), "Initial value for optimization, e.g. 0.6 0.4 0.3 (n = 3)") 
    ( "cfg",po::value<std::string>(&cfg)->default_value("data/frames.xml"),"load cfg from filename")
    ( "debug",po::value<int>(&globalLog.debugLevel)->default_value(5),"debug level")
    ( "trim",po::value<double>(&trim)->default_value(0.2),"trimming percentage")
    ( "epsilon",po::value<double>(&epsilon)->default_value(1.0e-8),"convergence limit, 0.00000001")
    ( "random",po::value<bool>(&useRandom)->default_value(false),"use random start value" ) 
    ( "always",po::value<bool>(&useShowResult)->default_value(false),"always show result" ) 
    ( "max",po::value<unsigned int>(&examplesMax)->default_value(10000000),"use max number of data" )  
    ( "iterations",po::value<unsigned int>(&iterations)->default_value(500),"use max number of iterations" ) 
    ( "debugiterations",po::value<unsigned int>(&counterMod)->default_value(100),"show result each x iterations" ) 
    ( "ivlinewidth",po::value<double>(&ivLineWidth)->default_value(0.5),"width of Inventor line")
    ( "ivsphereradius",po::value<double>(&ivSphereRadius)->default_value(1.0),"radius of Inventor sphere")
    ( "ivcoordscale",po::value<double>(&ivCoordScale)->default_value(0.2),"scaleFactor of Inventor coordinate system")
    ;
 
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cout << desc <<std::endl;
    return 0;
  }

  bool doQuit = vm.count("exit");
  
  // parse initial values
  std::vector<double> initialValues;
  strToArray(initial, initialValues);

  // set random generator
  unsigned int seed = (unsigned int) getTickCount(); 
  LOG_MSG(2, "Random Generator Seed is %d\n", seed);
  
  setUniformSeed(seed);
  srand(seed);

  COptimizerIv optimizer; 

  // set parameters
  optimizer.setEpsilon(epsilon);
  optimizer.setTrimming(trim);
  optimizer.setIterations(iterations);
  optimizer.setRandomStart(useRandom);
  optimizer.setQuit(doQuit);
  optimizer.setDataMax(examplesMax);
  optimizer.setShowResult(useShowResult);
  optimizer.setShowModulo(counterMod);
  optimizer.setIvLineWidth(ivLineWidth); 
  optimizer.setIvSphereRadius(ivSphereRadius);
  optimizer.setIvCoordScale(ivCoordScale);
 
  optimizer.load(cfg, filename); 

  optimizer.run(initialValues);
 
  COptimizerResult result;
  if (!optimizer.getResult(result))
    {
      LOG_MSG(1, "Error: Optimization failed\n.");
    }
  else
    {
      printf("Showing final result:\n");
      printf("Value: %f\n", result.result);
      printf("Values: %s\n", arrayToString(result.values).c_str());

      for (unsigned int i=0; i<result.optimizedPoses.size(); i++)
	{
	  CMatrix pose = result.optimizedPoses[i].second;
	  CVec position, quater; 
	  position = pose[3];
	  CMathLib::quaternionFromMatrix(pose, quater); 
	  
	  printf("Name: %s Position (xyz): %f %f %f Quaternion (wxyz): %f %f %f %f\n", result.optimizedPoses[i].first.c_str(),
		 position.x, position.y, position.z,
		 quater.w, quater.x, quater.y, quater.z);
	}

      optimizer.generateInventor(result.values, "data/plot.iv"); 
    }

  return 0;
}


