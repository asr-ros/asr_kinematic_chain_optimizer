/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Jäkel Rainer, Wittenbeck Valerij
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/


#ifndef __INC_OPTIMIZER
#define __INC_OPTIMIZER

#include <log.h>
#include <frame.h>
#include <configuration.h>
#include <datapairs.h>
#include <value_setter.h>
#include <optimizer_goal.h>
#include <optimizer_goal_global_position.h>
#include <optimizer_goal_global_orientation.h>
#include <optimizer_goal_orientation.h>
#include <optimizer_goal_position.h>
#include <optimizer_container.h>

namespace robotLibPbD {

class COptimizerResult
{
 public:
  std::vector<std::vector<std::pair<std::string, CMatrix> > > timelineOfPoses;
  
  std::vector<std::pair<std::string, CMatrix> > allPoses; 
  std::vector<std::pair<std::string, CMatrix> > optimizedPoses;
  std::vector<double> values;
  double result;

  void clear();
};

class COptimizer
{
 protected:
  std::vector<ValueSetter> valueSetters; 
  std::vector<double> result;

  std::vector<OptimizerContainer> data;
  std::vector<OptimizerGoalGlobal*> globalFunctions;
  std::vector<OptimizerGoal*> functions;
  CFrameContainer frames; 
  DataPairs information;
  //unsigned int trimming;
  std::vector<std::vector<double> > examples;
  std::vector<double> distances;
  unsigned int counter;
  double trim;
  unsigned int examplesMax, iterations;
  unsigned int dofs;
  unsigned int counterMod, trimming;
  bool useRandom, useShowResult, doQuit;
  double resultValue, epsilon;

  void setValue(double *x);
  bool writeToFile(std::string filename, std::string buffer);
  std::string readFromFile(std::string filename);
  void setData(std::vector<double> &values);
  void callback(int nparam, double *x, double *fj);
  
  bool isEqual(std::vector<double> &first, std::vector<double> &second, double eps = 0.1);
  
  void loadData(std::string filename, unsigned int startId = 0);
  void loadDofs(std::string filename);
  void loadGoals(std::string filename);
  void loadValueSetters(std::string filename);
  
  int getValue(TiXmlElement *node, std::string item);
  void reset();
  
 public:
 COptimizer() : counter(0), trim(0.0), examplesMax(100000),dofs(0),counterMod(10000),useRandom(false),useShowResult(false),doQuit(false),epsilon(1.0e-8) {}
  
  static void rosenbrockCallback(int nparam, double *x, double *fj, void *extraparams);
  
  void load(std::string cfg, std::string data, unsigned int start = 0);
  
  virtual std::string generateOutput(); 
  
  void run(const std::vector<double> &initialValues);
  
  void setEpsilon(double eps) { this->epsilon = eps; }
  void setTrimming(double trim) { this->trim = trim; }
  void setIterations(unsigned int iterations) { this->iterations = iterations; }
  void setRandomStart(bool useRandom) { this->useRandom = useRandom; }
  void setQuit(bool doQuit) { this->doQuit = doQuit; }
  void setDataMax(unsigned int examplesMax) { this->examplesMax = examplesMax; }
  void setShowResult(bool useShowResult) { this->useShowResult = useShowResult; }
  void setShowModulo(unsigned int counterMod) { this->counterMod = counterMod; }

  bool getResult(COptimizerResult &out);
};

};

#endif
