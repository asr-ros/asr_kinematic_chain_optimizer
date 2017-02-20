/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Jäkel Rainer, Wittenbeck Valerij
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/


#include <iostream>
#include <cstring>
#include <fstream>
#include <cstdio>
#include <sys/times.h>
#include <sys/time.h>

#include "log.h"

CLog globalLog;

CLog::CLog()
{
  debugLevel = 0;
  inputLevel = 0;
}
void CLog::setInputLevel(int inputLevel)
{
  this->inputLevel = inputLevel;
}

void CLog::setDebugLevel(int debugLevel)
{
  this->debugLevel = debugLevel;
}

void CLog::writeLine(std::string line, bool append)
{
  std::ofstream newFile;
  newFile.open(this->filename.c_str(), std::ios::out | append ? std::ios::app : std::ios::trunc);

  if (newFile)
    newFile << getTimeAsStr() << ": " << line << "\n";
  newFile.close();
}

std::string CLog::getTimeAsStr()
{
  static struct timeval tstart;
  static struct timezone tz;
  struct tm* ptm;

  gettimeofday(&tstart, &tz);
  ptm = localtime(&tstart.tv_sec);
  char time[1024];
  strftime(time, sizeof (time), "%Y-%m-%d %H:%M:%S", ptm); 
  return time;
}
std::string CLog::generateLogname(std::string path)
{
  char buffer[1024];
  std::string time = getTimeAsStr();
  sprintf(buffer, "%s/logs/log_%s", path.c_str(), time.c_str());
  return buffer;
}

void CLog::setFilename(std::string filename)
{
  this->filename = filename;
}
