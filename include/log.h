/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Jäkel Rainer, Wittenbeck Valerij
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/


#ifndef __LOGFILE

#define __LOGFILE

#include <string.h>

#define LOG_MSG_NO_ERROR_VERBOSE 5
#define LOG_MSG_ERROR_VERBOSE 1
#define LOG_MSG_ERROR_SEVERE 0


#define LOG(format, ...) { char buffer[1024]; snprintf(buffer, 1024, format, ## __VA_ARGS__); std::string str = buffer; globalLog.writeLine(str); printf(format, ## __VA_ARGS__); }

#define LOG_MSG(index, format, ...) { if (index <= globalLog.debugLevel) { char buffer[1024]; snprintf(buffer, 1024, format, ## __VA_ARGS__); std::string str = buffer; globalLog.writeLine(str);  printf(format, ## __VA_ARGS__); } }

#define LOG_VERBOSE(format, ...) { if (LOG_MSG_NO_ERROR_VERBOSE <= globalLog.debugLevel) { char buffer[1024]; snprintf(buffer, 1024, format, ## __VA_ARGS__); std::string str = buffer; globalLog.writeLine(str);  printf(format, ## __VA_ARGS__); } }

#define LOG_ERROR(format, ...) { if (LOG_MSG_ERROR_VERBOSE <= globalLog.debugLevel) { char buffer[1024]; snprintf(buffer, 1024, format, ## __VA_ARGS__); std::string str = buffer; globalLog.writeLine(str);  printf(format, ## __VA_ARGS__); } }

#define LOG_ERROR_SEVERE(format, ...) { if (LOG_MSG_ERROR_SEVERE <= globalLog.debugLevel) { char buffer[1024]; snprintf(buffer, 1024, format, ## __VA_ARGS__); std::string str = buffer; globalLog.writeLine(str); printf(format, ## __VA_ARGS__); } }


#define LOG_MSG_LEVEL(index) (index <= globalLog.debugLevel)
#define LOG_INPUT_LEVEL(index) (index <= globalLog.inputLevel)

class CLog
{
 private:
  std::string filename, path;
 public:
  int debugLevel;
  int inputLevel;
  static std::string getTimeAsStr();

  CLog();
  static std::string generateLogname(std::string path);
  void setFilename(std::string filename);
  void writeLine(std::string line, bool append = true);

  void setDebugLevel(int debugLevel);
  void setInputLevel(int inputLevel);
};   


extern CLog globalLog;
                  
#endif
