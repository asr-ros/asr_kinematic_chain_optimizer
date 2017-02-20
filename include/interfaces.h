/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Jäkel Rainer, Wittenbeck Valerij
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/



#ifndef __INTERFACES

#define __INTERFACES

#include <string.h>

class CCopyInterface
{
 public:
  virtual void* getCopy() { return NULL; };
};

class CStorageInterface
{
 public:
  virtual std::string getAsXml() { return ""; };
  virtual bool getFromXml(const std::string &data) { return false; };
};

class CMutexInterface
{
 public:
  virtual void lock() {};
  virtual void unlock() {};
};

class CTimestampInterface
{
 protected:
  unsigned long int timestamp;
 public:
  virtual unsigned long int getTimestamp() { return timestamp; };
  virtual void setTimestamp(unsigned long int timestamp) { this->timestamp = timestamp; };

  bool compareTimestamp(unsigned long int first, unsigned long int second) { return first > second; };
};

class CNameInterface
{
 protected:
  std::string _name;
 public:
  void setName(std::string value) { this->_name = value; };
  std::string getName() { return this->_name; };
};

class CTypeInterface
{
 protected:
  std::string _type;
 public:
  void setType(std::string value) { this->_type = value; };
  std::string getType() { return this->_type; };
};

class CEdgeInterface
{
 protected:
  std::string _from, _to;
 public:
  void setFrom(std::string value) { this->_from = value; };
  std::string getFrom() { return this->_from; };

  void setTo(std::string value) { this->_to = value; };
  std::string getTo() { return this->_to; };

};

class CValidInterface
{
 protected:
  bool _isValid;
 public:
  void setValid(bool value = true) { this->_isValid = value; };
  bool isValid() { return this->_isValid; };

};

class CPriorityInterface
{
 protected:
  double _priority;
 public:
  void setPriority(double value) { this->_priority = value; };
  double getPriority() { return this->_priority; };

  bool comparePriority(double first, double second) { return first > second; };
};

#endif
