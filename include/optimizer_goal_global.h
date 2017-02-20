/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Jäkel Rainer, Wittenbeck Valerij
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/


#ifndef __INC_OPTIMIZER_GOAL_GLOBAL
#define __INC_OPTIMIZER_GOAL_GLOBAL

#include <vector>  
#include <algorithm> 
#include <optimizer_goal.h>

namespace robotLibPbD {
 

class OptimizerGoalGlobal : public OptimizerGoal
{ 
public:
  std::vector<double> item, mean;
  std::vector<std::pair<double, int> > sorted;
  std::vector<int> ids;
  std::vector<std::vector<double> > data;
  unsigned int trimming;
 
  OptimizerGoalGlobal() 
  {
    trimming = 0;
  };

  virtual void add()
  {  
    data.push_back(item);
  };
  virtual void reset()
  {
    data.clear();
  };

  void setTrimming(unsigned int value)
  {
    trimming = value;
  };

  double dist(std::vector<double> &first, std::vector<double> &second)
  {
    double tmpf = 0.0;
    for (unsigned int i=0; i<first.size() && i<second.size(); i++)
      tmpf += (first[i] - second[i])*(first[i] - second[i]);
    return sqrt(tmpf);
  } 
  
  double getTrimmedMeanAndVariance(std::vector<std::vector<double> > &data, int index, std::vector<double> &item)
  { 
    if (data.size() == 0)
      return 10000.0;

    // get variance
    sorted.clear();
    for (unsigned int i=0; i<data.size(); i++) 
      if ((int)i != index)
    sorted.push_back(std::make_pair(dist(data[i],item), i));
    
    std::sort(sorted.begin(), sorted.end(), comparePairs<double, int>);

    ids.clear();
    for (unsigned int i=trimming; i<sorted.size() - trimming; i++)
      ids.push_back(sorted[i].second);

    if (index >= 0)
      ids.push_back(index);

    double n = (double)ids.size();

    // get mean
    item.clear();
    item.resize(data[0].size(), 0.0);
    for (unsigned int i=0; i<ids.size(); i++) 
      for (unsigned int j=0; j<item.size(); j++)
	item[j] += data[ids[i]][j] / n;
 
    // get variance
    n -= 1.0;
    double tmpf = 0.0, tmpf2;
    for (unsigned int i=0; i<ids.size(); i++) 
      {
	tmpf2 = dist(data[ids[i]], item);
	tmpf += (tmpf2 * tmpf2) / n;
      }
    return tmpf;
      
  }

  virtual double getDistance() 
  {  
    if (data.size() == 0)
      return 10000.0;

    // get best trimmed mean
    double min = 1.0e30;
    std::vector<double> tmpMean;
    for (unsigned int i=0; i<data.size(); i++)
      {
	tmpMean.clear();
	tmpMean = data[i];
	double tmpf = getTrimmedMeanAndVariance(data, i, tmpMean);
	if (tmpf < min)
	  {
	    min = tmpf;
	    mean = tmpMean;
	  }
      }
 
    return sqrt(min);
  };
};

};

#endif
