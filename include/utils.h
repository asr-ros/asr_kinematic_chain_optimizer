/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Jäkel Rainer, Wittenbeck Valerij
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/


#ifndef __UTILS

#define __UTILS

#include <string>
#include <cstdlib>
#include <vector>
#include <list>
#include <utility>

#include <vecmath.h>

namespace robotLibPbD {

int getDirectoryFiles(std::string dir, std::vector<std::string> &files, bool appendDir = false);
bool stringEndsWith(std::string text, std::string end, std::string sep = "_");
bool stringStartsWith(std::string text, std::string end, std::string sep = "_");
std::string strtrim(std::string& s,const std::string& drop = " ");
std::string strreplace(const std::string &stringSearchString, const std::string &stringReplaceString, std::string stringStringToReplace);
void strreplaceWithCondition(const std::string &stringSearchString, const std::string &stringReplaceString, std::string &stringStringToReplace, std::string condition = "");
void strreplace(char* text, char what = '.', char with = ',');
double strToDouble(const char* text, double value = 0.0);
void strToArray(std::string text, std::vector<double> &result, std::string delimiter = " ");

 std::string combineStrings(std::vector<std::string> &input, std::string delimiter = " ");

void strtokenize(const std::string& str,
                      std::vector<std::string>& tokens,
          const std::string& delimiters);

void matrixToArray(const CMatrix &matrix, std::vector<double> &values);
void arrayToMatrix(CMatrix &matrix, std::vector<double> &values);

void vectorToMatrix(CMatrix &matrix, const std::vector<double> &values);
void vectorToMatrix6(CMatrix &matrix, const double* values);
void matrixToVector(const CMatrix &matrix, std::vector<double> &values);
void matrixToVector6(const CMatrix &matrix, double *values);

void getConvexHullXY(std::vector<CVec> &points, std::vector<CVec> &hull);

std::string printToString(const char* format, ...);

std::string boolToString(bool value);

template< typename T >
  bool inRange(std::vector<T> &array, T min, T max)
{
  for (unsigned int i=0; i<array.size(); i++)
    if ((array[i] < min) || (array[i] > max))
      return false;
  
  return true;
};

template< typename T >
bool isIncluded(std::vector<T> &array, T &value)
{
  for (unsigned int i=0; i<array.size(); i++)
    if (array[i] == value)
      return true;

  return false;
};

template< typename T >
std::string arrayToString(std::vector<T> &array)
{
  std::string result;
  for (unsigned int i=0; i<array.size(); i++)
    result += printToString("%g ", (double)array[i]);
  return result;
};


template< typename T, typename S >
bool comparePairs(const std::pair<T, S> &first, const std::pair<T, S> &second)
{
  return first.first < second.first;
};

template< typename T, typename S >
bool comparePairsVector(const std::pair<T, S> &first, const std::pair<T, S> &second)
{
  for (unsigned int i=0; i<first.first.size(); i++)
    if (first.first[i] < second.first[i])
      return true;
    else if (first.first[i] > second.first[i])
      return false;

  return true;
};

unsigned long getTickCount();


std::string waitForReturn(std::string msg = "Press any key.\n");
};



#endif
