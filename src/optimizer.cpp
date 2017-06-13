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
namespace po = boost::program_options;

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
#include <optimizer.h>

using namespace robotLibPbD;

void COptimizer::setValue(double *x)
{
  unsigned int counter = 0;
  for (unsigned int i=0; i<data.size(); i++)
    {
      data[i].setValue(x, counter);
      counter += data[i].dofs.size();
    }
}

std::string COptimizer::readFromFile(std::string filename)
{
  std::vector<std::string> text;
  std::string line;
  std::ifstream textstream;

  textstream.open(filename.c_str());
  

  while (textstream.good() && std::getline(textstream, line)) {
    text.push_back(line + "\n");
  }
  textstream.close();
  std::string alltext;
  for (unsigned int i=0; i < text.size(); i++)
    alltext += text[i];

  return alltext;
}


void COptimizer::setData(std::vector<double> &values)
{
  for (unsigned int i=0; i<valueSetters.size(); i++)
    {
      valueSetters[i].set(values);
    }
  /*
  for (unsigned int i=0; i<values.size(); i++)
    {
      LOG_MSG(10, "Setting: %s = %s\n", printToString("%d", i+1).c_str(), printToString("%f", values[i]).c_str());
      information.add(printToString("%d", i+1), printToString("%f", values[i]), true);
    }
  frames.loadFromFile(cfg.c_str(), information, config, false);
  frames.invalidate();
  */
}

void COptimizer::rosenbrockCallback(int nparam, double *x, double *fj, void *extraparams)
{
  if (extraparams != NULL)
    ((COptimizer*) extraparams)->callback(nparam, x, fj);
}

void COptimizer::callback(int nparam, double *x, double *fj)
{  
  LOG_MSG(10, "X: ");
  for (int i=0; i<nparam; i++)
    LOG_MSG(10, "%f ", x[i]);
  LOG_MSG(10, "\n");

  fj[0] = 0.0;


  for (unsigned int i=0; i<globalFunctions.size(); i++)
    globalFunctions[i]->reset();

  setValue(x);

  for (unsigned int j=0; j<examples.size(); j++)
    {
      setData(examples[j]);
   
      if (LOG_MSG_LEVEL(10))
	{
	  CFrameInterface interface;
	  for (unsigned int i=0; i<frames.getFrames().size(); i++)
	    {
	      interface.setFrame(frames.getFrame(i));
	      LOG_MSG(10, "%s", interface.getFrameAsXml().c_str());
	      LOG_MSG(10, "%s", frames.getFrame(i)->getRelativeToBase().toString().c_str());
	    }
	}

      distances[j] = 0.0;
      for (unsigned int i=0; i<functions.size(); i++)
	{
	  double tmpf = functions[i]->getDistance(); 
	  LOG_MSG(6, "%d %d: %f\n", j, i, tmpf);

	  distances[j] += tmpf;

	  //fj[0] += tmpf / (double) examples.size();
	}

      
      for (unsigned int i=0; i<globalFunctions.size(); i++)
	globalFunctions[i]->add();
    }

  
  for (unsigned int i=0; i<globalFunctions.size(); i++)
    fj[0] += globalFunctions[i]->getDistance();  
  
  if (functions.size() > 0)
    {
      if (distances.size() == 1)
	{
	  fj[0] += distances[0];
	} else
	{
	  std::sort(distances.begin(), distances.end());
	  for (unsigned int i=trimming; i<distances.size() - trimming; i++)
	    fj[0] += distances[i] / (double) (distances.size() - 2*trimming);
	}
    }

  if (LOG_MSG_LEVEL(5))
    {
      LOG_MSG(2,"Value: %f X: ", fj[0]);
      for (int i=0; i<nparam; i++)
	LOG_MSG(2,"%f ", x[i]);
      LOG_MSG(2,"\n");
    }

  counter++;
  if (useShowResult || counter % counterMod == 0 )
    {
      LOG_MSG(2,"Goal-Value: %f State-Values: ", fj[0]);
      for (int i=0; i<nparam; i++)
	LOG_MSG(2,"%f ", x[i]);
      LOG_MSG(2,"\n");
      std::string tmp = generateOutput();
      LOG_MSG(2,"%s\n", tmp.c_str());
    }
}

bool COptimizer::isEqual(std::vector<double> &first, std::vector<double> &second, double eps)
{
  for (unsigned int i=0; i<first.size() && i<second.size(); i++)
    if (fabsf(first[i] - second[i]) > eps)
      return false;

  return true;
}

void COptimizer::loadData(std::string filename, unsigned int startId)
{
    std::string line, prev;
    std::ifstream textstream;
    std::vector<double> input;
    std::vector<std::vector<double> > examples2;
    LOG_VERBOSE("Reading data from %s.\n", filename.c_str());

    textstream.open(filename.c_str());

    unsigned int lines = 0;
    while (textstream.good() && std::getline(textstream, line))
    {
        lines++;
        if (lines <= startId)
        continue;

        input.clear();

        if (line.find(":") != std::string::npos)
        line = line.substr(line.find(":")+1, line.length());



        if (line.size() <= 0)
        continue;

        LOG_VERBOSE("%s\n", line.c_str());
        strToArray(line, input, ";");

        //ToDo: Debug isEqual
        //if (examples2.size() > 0 && isEqual(examples2.back(), input))
        //continue;

        examples2.push_back(input);
        prev = line;
    }
    textstream.close();

    int exampleCount = examples2.size();
    LOG_VERBOSE("Loaded %d ( %d ) examples\n", exampleCount, lines);
    unsigned int offset = examples2.size() / examplesMax;
    if (offset < 1)
    offset = 1;
    for (unsigned int i=0; i<examples2.size(); i+=offset)
    examples.push_back(examples2[i]);

    distances.resize(examples.size());
}


std::string COptimizer::generateOutput()
{
    std::string tmp;
    CFrameInterface interface;
    for (unsigned int i=0; i<data.size(); i++)
    {
        CVec position, quater;
        CMatrix matrix = data[i].frame->getPose();
        position = matrix[3];
        CMathLib::quaternionFromMatrix(matrix, quater);
        interface.setFrame(data[i].frame);
        CVec eulerAngles;
        CMathLib::getEulerZXZ(matrix, eulerAngles);
        tmp += printToString("Result: %s", interface.getFrameAsXml().c_str());
        tmp += printToString("Position (xyz): %f %f %f Quaternion (wxyz): %f %f %f %f EulerAngles(ZXZ): %f %f %f \n\n", position.x, position.y, position.z,
         quater.w, quater.x, quater.y, quater.z, eulerAngles.x*180.0/M_PI, eulerAngles.y*180.0/M_PI, eulerAngles.z*180.0/M_PI);
    }
    tmp += printToString("<-- result -->\n\n");
    return tmp;
}

bool COptimizer::writeToFile(std::string filename, std::string buffer)
{
  std::ofstream textstream(filename.c_str());
  if (textstream.fail())
    return false;

  textstream << buffer;
  textstream.close();
  return true;
}

int COptimizer::getValue(TiXmlElement *node, std::string item)
{
  std::string tmp, tmp2;
  tmp = CConfiguration::getAttributeString(node, item.c_str(), "");
  size_t found = tmp.find("([");
  size_t found2 = tmp.find("])");
  
  if (found != std::string::npos && found2 !=std::string::npos)
    {
      tmp2 = tmp.substr(found+2, found2-found-2);
      LOG_MSG(2, "Value: %s\n", tmp2.c_str());
      return (int) atof(tmp2.c_str()) - 1;
    }

  return -1;
}
 
void COptimizer::loadDofs(std::string filename)
 {
  // load frame structure from xml 
  frames.loadFromFile(filename.c_str());
  frames.updateBaseLinks();

  CFrameInterface interface; 
  std::vector<double> dofs_max, dofs_min;
  std::vector<unsigned int> dofs; 

  unsigned int n = 0;
  for (unsigned int i=0; i<frames.getFrames().size(); i++)
    {
      interface.setFrame(frames.getFrame(i));
      LOG_VERBOSE("Frame: %s\n", interface.getFrameAsXml().c_str());

      frames.getFrame(i)->getDofs(dofs);
      if (dofs.size() > 0)
	{
	  frames.getFrame(i)->getDofs(dofs_min, dofs_max);
	  for (unsigned int j=0; j<dofs.size(); j++)
	    LOG_VERBOSE("Loaded dof %d: %g %g\n", dofs[j], dofs_min[dofs[j]], dofs_max[dofs[j]]);

	  n += dofs.size();

	  OptimizerContainer item;
	  item.frame = frames.getFrame(i);
	  item.dofs_max = dofs_max;
	  item.dofs_min = dofs_min;
	  item.dofs = dofs; 
	  data.push_back(item);
	}
    }
  
  LOG_VERBOSE("Total number of dofs: %d\n", n);
  this->dofs = n;
 }


void COptimizer::loadValueSetters(std::string filename)
 {
  // load frame structure from xml
   CConfiguration config("Data"); 
  //bool okay = config.load(filename.c_str());
   config.load(filename.c_str());
 
  // Load goal
  std::vector<TiXmlElement*> result; 
  config.findNodes("Frames.Frame", result);
  for (unsigned int i=0; i<result.size(); i++)
    {
      LOG_MSG(2,"Frame: %s\n", CConfiguration::getAttributeString(result[i], "name", ""));
      ValueSetter valueSetter; 

      valueSetter.x = getValue(result[i], "x");
      valueSetter.y = getValue(result[i], "y");
      valueSetter.z = getValue(result[i], "z");
      valueSetter.qw = getValue(result[i], "qw");
      valueSetter.qx = getValue(result[i], "qx");
      valueSetter.qy = getValue(result[i], "qy");
      valueSetter.qz = getValue(result[i], "qz");
      valueSetter.a = getValue(result[i], "a");
      valueSetter.b = getValue(result[i], "b");
      valueSetter.g = getValue(result[i], "g");

      if (valueSetter.x < 0 &&
	  valueSetter.y < 0 &&
	  valueSetter.z < 0 &&
      valueSetter.qw < 0 &&
	  valueSetter.qx < 0 &&
	  valueSetter.qy < 0 &&
      valueSetter.qz < 0 &&
      valueSetter.a < 0 &&
      valueSetter.b < 0 &&
      valueSetter.g < 0)
	continue;

      int frameId = frames.getFrameByName(CConfiguration::getAttributeString(result[i], "name", ""));
      if (frameId < 0)
	continue;

      valueSetter.frame = frames.getFrame(frameId);
      
      valueSetters.push_back(valueSetter);
    }
 }


void COptimizer::loadGoals(std::string filename)
 {
  // load frame structure from xml
   CConfiguration config("Data"); 
  //bool okay = config.load(filename.c_str());
   config.load(filename.c_str());
 
  // Load goal
  std::vector<TiXmlElement*> result;
  config.findNodes("Goal.Function", result);
  for (unsigned int i=0; i<result.size(); i++)
    {
      OptimizerGoal* goal = new OptimizerGoal();
      goal->loadFromXml(frames, result[i]);
      functions.push_back(goal);
    }

  result.clear();
  config.findNodes("Goal.GlobalPosition", result);
  for (unsigned int i=0; i<result.size(); i++)
    {
      OptimizerGoalGlobalPosition* goal = new OptimizerGoalGlobalPosition();
      goal->loadFromXml(frames, result[i]);
      goal->setTrimming(trimming);
      globalFunctions.push_back(goal);
    }


  result.clear();
  config.findNodes("Goal.GlobalOrientation", result);
  for (unsigned int i=0; i<result.size(); i++)
    {
      OptimizerGoalGlobalOrientation* goal = new OptimizerGoalGlobalOrientation();
      goal->loadFromXml(frames, result[i]);
      goal->setTrimming(trimming);
      globalFunctions.push_back(goal);
    }

  result.clear();
  config.findNodes("Goal.Position", result);
  for (unsigned int i=0; i<result.size(); i++)
    {
      OptimizerGoalPosition* goal = new OptimizerGoalPosition();
      goal->loadFromXml(frames, result[i]);
      functions.push_back(goal);
    }

  result.clear();
  config.findNodes("Goal.Orientation", result);
  for (unsigned int i=0; i<result.size(); i++)
    {
      OptimizerGoalOrientation* goal = new OptimizerGoalOrientation();
      goal->loadFromXml(frames, result[i]);
      functions.push_back(goal);
    }
 }


void COptimizer::reset()
{
  result.clear();
  valueSetters.clear();
  data.clear();


  globalFunctions.clear();


  functions.clear();

  information.clear();
  frames.clear();

  examples.clear();
  distances.clear();
}

void COptimizer::load(std::string cfg, std::string data, unsigned int start)
{
  reset();

  loadData(data, start);

  loadDofs(cfg);

  loadGoals(cfg);

  loadValueSetters(cfg);
  
}

void COptimizer::run(const std::vector<double> &initialValues)
{
  // reset
  counter = 0;

  // set trimming
  trimming = 0;
  if (trim > 0.0 && trim < 1.0)
    {
      trimming = (unsigned int) (trim / 2.0 * (double) examples.size());
      if (trimming == 0)
	trimming = 0;

      LOG_VERBOSE("Trimming best %d and worst %d items\n", trimming, trimming);
    }
  for (unsigned int i=0; i<globalFunctions.size(); i++)
    globalFunctions[i]->setTrimming(trimming);
  
  int nparam,maxIter,verbosity;
  double *st,*bl,*bu,bigbnd,eps;
  nparam=dofs;
  st=(double*)calloc(nparam,sizeof(double));
  bl=(double *)calloc(nparam,sizeof(double));
  bu=(double *)calloc(nparam,sizeof(double));
  bigbnd=1.e10;
  maxIter=iterations;
  eps=epsilon;//CHANGED 1.e-8;
  verbosity=0;

  for (int i=0; i<nparam; i++)
    {
      st[i] = 0.5;
      bl[i] = 0.0;
      bu[i] = 1.0;
    }

  if ((int)initialValues.size() >= nparam)
    for (int i=0; i<nparam; i++)
      {
	st[i] = initialValues[i];
	if (st[i] < 0.0 || st[i] > 1.0)
	  LOG_MSG(2,"Error: %d'th value %f has to be in [0,1].\n", i, st[i]);
      }
  
  if (useRandom)
    for (int i=0; i<nparam; i++)
      {
	st[i] = getUniform(); 
      }
  double y[1];


  LOG_MSG(2, "Initial:\n");
  for (int i=0; i<nparam; i++)
    LOG_MSG(2, "%f\n", st[i]);

  if (doQuit)
    {
      rosenbrockCallback(nparam, st, y, this);
      exit(0);
    }
 

  rosenbrock(nparam,st,bl,bu,bigbnd,maxIter,eps,verbosity,rosenbrockCallback, this);
  
  rosenbrockCallback(nparam, st, y, this);

  LOG_MSG(2, "Best: %f\n", y[0]);

  LOG_MSG(2, "Result:\n");
  for (int i=0; i<nparam; i++)
    LOG_MSG(2, "%f\n", st[i]);

  LOG_MSG(1, "%f ", y[0]);
 
  for (int i=0; i<nparam; i++)
    LOG_MSG(1, "%f ", st[i]);
  LOG_MSG(1, "%f\n", y[0]);
 
  setValue(st);  
  std::string output = generateOutput();
  LOG_MSG(2, "Final result:\n");
  LOG_MSG(2, "%s\n", output.c_str()); 
  
  this->resultValue = y[0];
  
  this->result.resize(nparam);
  for (int i=0; i<nparam; i++)
    this->result[i] = st[i];

  free(st);
  free(bl);
  free(bu);
}


void COptimizerResult::clear()
{
  allPoses.clear();
  optimizedPoses.clear();
  values.clear();
  timelineOfPoses.clear(); 
}

bool COptimizer::getResult(COptimizerResult &out)
{
  if (result.size() == 0)
    return false;

  double *st = (double*)calloc(result.size(),sizeof(double));
  for (unsigned int i=0; i<result.size(); i++)
    st[i] = result[i];
  
  setValue(st); 

  // reset
  out.clear();

  // set result
  out.result = resultValue;
  out.values = result;
  for (unsigned int i=0; i<data.size(); i++)
    {
      out.optimizedPoses.push_back(make_pair((std::string)data[i].frame->getName(), data[i].frame->getPose()));
    }

  for (unsigned int i=0; i<frames.getFrames().size(); i++)
    out.allPoses.push_back(make_pair((std::string)frames.getFrame(i)->getName(), frames.getFrame(i)->getPose()));
    
  // generate timeline
  out.timelineOfPoses.resize(examples.size());
  for (unsigned int j=0; j<examples.size(); j++)
    {  
      setData(examples[j]);
      setValue(st); 

      for (unsigned int i=0; i<frames.getFrames().size(); i++)
	out.timelineOfPoses[j].push_back(make_pair((std::string)frames.getFrame(i)->getName(), frames.getFrame(i)->getPose()));
    }

  free(st);
  return true;
}
