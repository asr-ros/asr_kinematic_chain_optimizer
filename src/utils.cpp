/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Jäkel Rainer, Wittenbeck Valerij
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <vector>

#include <sys/times.h>
#include <sys/time.h>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <vector>
#include <string>
#include <iostream>

#include <utils.h>

namespace robotLibPbD {


  std::string waitForReturn(std::string msg)
  {
    std::cout << msg;
    char bChar[255];
    std::cin.getline(bChar, 255);
    std::string result = bChar;
    return result;
  }

  std::string combineStrings(std::vector<std::string> &input, std::string delimiter)
 {
   std::string tmp;
   for (unsigned int i=0; i<input.size(); i++)
     {
       tmp += input[i];

       if (i+1 < input.size())
	 tmp += delimiter;
     }

   return tmp;
 }

int getDirectoryFiles(std::string dir, std::vector<std::string> &files, bool appendDir)
{
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(dir.c_str())) == NULL) 
      {
        std::cout << "Error(" << errno << ") opening " << dir << std::endl;
        return errno;
      }

    if (dir.length() > 0 && dir[dir.length()-1] != '/')
      dir = dir + "/";
    
    while ((dirp = readdir(dp)) != NULL) 
      {
	if (appendDir)
      files.push_back(dir + std::string(dirp->d_name));
	else
      files.push_back(std::string(dirp->d_name));
      }
    closedir(dp);
    return 0;
}


std::string boolToString(bool value){ if (value) return "true"; return "false"; }

void strtokenize(const std::string& str,
                      std::vector<std::string>& tokens,
                      const std::string& delimiters)
{
    // Skip delimiters at beginning.
    std::string::size_type lastPos = str.find_first_not_of(delimiters, 0);
    // Find first "non-delimiter".
    std::string::size_type pos     = str.find_first_of(delimiters, lastPos);

    while (std::string::npos != pos || std::string::npos != lastPos)
    {
        // Found a token, add it to the vector.
        tokens.push_back(str.substr(lastPos, pos - lastPos));
        // Skip delimiters.  Note the "not_of"
        lastPos = str.find_first_not_of(delimiters, pos);
        // Find next "non-delimiter"
        pos = str.find_first_of(delimiters, lastPos);
    }
}

//Ersetzt einen String
//stringSearchString = string der mit stringReplaceString ersetzt wird
//stringStringToReplace ist der string in dem gesucht und ersetzt wird
std::string strreplace(const std::string &stringSearchString, const std::string &stringReplaceString, std::string stringStringToReplace)
{
        std::string::size_type pos = stringStringToReplace.find(stringSearchString, 0);
        int intLengthSearch = stringSearchString.length();
        int intLengthReplacment = stringReplaceString.length();

        while(std::string::npos != pos)
        {
                stringStringToReplace.replace(pos, intLengthSearch, stringReplaceString);
                pos = stringStringToReplace.find(stringSearchString, pos + intLengthReplacment);
        }

        return stringStringToReplace;
} 

void strreplaceWithCondition(const std::string &stringSearchString, const std::string &stringReplaceString, std::string &stringStringToReplace, std::string condition)
{
  if (condition.length() == 0)
    {
      stringStringToReplace = strreplace(stringSearchString, stringReplaceString, stringStringToReplace);
      return;
    }

  std::string::size_type pos = stringStringToReplace.find(stringSearchString, 0);
  int intLengthSearch = stringSearchString.length();
  int intLengthReplacment = stringReplaceString.length();
  
  while(std::string::npos != pos)
    {
      if (pos > 0 && stringStringToReplace[pos] == condition[0])
	{
	  pos = stringStringToReplace.find(stringSearchString, pos + 1);
	  continue;
	}
      stringStringToReplace.replace(pos, intLengthSearch, stringReplaceString);
      pos = stringStringToReplace.find(stringSearchString, pos + intLengthReplacment);
    }
} 

std::string printToString(const char* format, ...)
{
  char buffer[65536];
  va_list argp;
  va_start(argp, format);
  vsprintf(buffer, format, argp);
  va_end(argp);
  
  return buffer;
}


void strToArray(std::string text, std::vector<double> &result, std::string delimiter)
{
  std::vector<std::string> tokens;
  strtokenize(text, tokens, delimiter);
  result.clear();
  for (unsigned int i=0; i<tokens.size(); i++)
    if (tokens[i] != "")
      try
	{
	  result.push_back(atof(tokens[i].c_str()));
	} catch(...)
	{
	}
}

void matrixToArray(const CMatrix &matrix, std::vector<double> &values)
{
    values[0] = matrix.a[12];
    values[1] = matrix.a[13];
    values[2] = matrix.a[14];
                    
    values[3] = matrix.a[0];
    values[4] = matrix.a[1];
    values[5] = matrix.a[2];
                    
    values[6] = matrix.a[4];
    values[7] = matrix.a[5];
    values[8] = matrix.a[6];
                    
    values[9]  = matrix.a[8];
    values[10] = matrix.a[9];
    values[11] = matrix.a[10];
}

void arrayToMatrix(CMatrix &matrix, std::vector<double> &values)
{
    matrix.a[12] = values[0];
    matrix.a[13] = values[1];
    matrix.a[14] = values[2];
    matrix.a[15] = 1.0;
               
    matrix.a[0] = values[3];
    matrix.a[1] = values[4];
    matrix.a[2] = values[5];
    matrix.a[3] = 0.0;
               
    matrix.a[4] = values[6];
    matrix.a[5] = values[7];
    matrix.a[6] = values[8];
    matrix.a[7] = 0.0;
               
    matrix.a[8] = values[9];
    matrix.a[9] = values[10];
    matrix.a[10] = values[11];
    matrix.a[11] = 0.0;
}

bool stringStartsWith(std::string text, std::string end, std::string sep)
{
  std::vector<std::string> tokens;
  strtokenize(text, tokens, sep);
  if (tokens.size() == 0)
    return false;
  return strcasecmp(tokens[0].c_str(), end.c_str()) == 0;
}


bool stringEndsWith(std::string text, std::string end, std::string sep)
{
  std::vector<std::string> tokens;
  strtokenize(text, tokens, sep);
  if (tokens.size() == 0)
    return false;
  return strcasecmp(tokens.back().c_str(), end.c_str()) == 0;
}

void vectorToMatrix6(CMatrix &matrix, const double* values)
{
  CVec tmp;
  double angle;
  tmp.x = values[3];
  tmp.y = values[4];
  tmp.z = values[5];
  
  angle = tmp.length();
  if (angle > 0.0)
    tmp /= angle;

  CMathLib::getMatrixFromRotation(matrix, tmp, angle);
  matrix.a[12] = values[0];
  matrix.a[13] = values[1];
  matrix.a[14] = values[2];
}

void vectorToMatrix(CMatrix &matrix, const std::vector<double> &values)
{
  CVec tmp;
  float angle;
  if (values.size() > 3)
    {
      tmp.x = values[3];
      if (values.size() > 4)
	{
	  tmp.y = values[4];
	  if (values.size() > 5)
	    {
	      tmp.z = values[5];
	    }
	}
    }
  angle = tmp.length();
  if (angle > 0.0)
    tmp /= angle;

  CMathLib::getMatrixFromRotation(matrix, tmp, angle);
  if (values.size() > 0)
    {
      matrix.a[12] = values[0];
      if (values.size() > 1)
	{
	  matrix.a[13] = values[1];
	  if (values.size() > 2)
	    {
	      matrix.a[14] = values[2];
	    } else matrix.a[14] = 0.0;
	} else matrix.a[13] = 0.0;
    } else matrix.a[12] = 0.0;
}

void matrixToVector(const CMatrix &matrix, std::vector<double> &values)
{
  CVec tmp;
  double angle;
  CMathLib::getRotationFromMatrix(matrix, tmp, angle);
  values.resize(6);
  values[0] = matrix.a[12];
  values[1] = matrix.a[13];
  values[2] = matrix.a[14];
  values[3] = tmp.x * angle;
  values[4] = tmp.y * angle;
  values[5] = tmp.z * angle;
}

void matrixToVector6(const CMatrix &matrix, double *values)
{
  CVec tmp;
  double angle;
  CMathLib::getRotationFromMatrix(matrix, tmp, angle);
  values[0] = matrix.a[12];
  values[1] = matrix.a[13];
  values[2] = matrix.a[14];
  values[3] = tmp.x * angle;
  values[4] = tmp.y * angle;
  values[5] = tmp.z * angle;
}

double strToDouble(const char* text, double value)
{
  if (text == NULL)
    return value;

  try
    {
      return atof(text);
    } catch (...)
    {
      return value;
    }
}

std::string strtrim(std::string& s,const std::string& drop)
{
 std::string r=s.erase(s.find_last_not_of(drop)+1);
 return r.erase(0,r.find_first_not_of(drop));
}


void strreplace(char *text, char what, char with)
{
        char* ptr = text;
        
        while (*ptr != '\0')
        {
            if (*ptr == what)
               *ptr = with;
            
            ptr++;
        }
}

unsigned long getTickCount()
{
     static struct timeval tstart;
     static struct timezone tz;
     gettimeofday(&tstart, &tz);
     return tstart.tv_sec*1000 + tstart.tv_usec / 1000;  
}


void getConvexHullXY(std::vector<CVec> &points, std::vector<CVec> &hull)
{
  std::vector<std::pair<double, unsigned int> > angles;
  
}

}


