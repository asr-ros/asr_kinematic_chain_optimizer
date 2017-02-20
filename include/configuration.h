/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Jäkel Rainer, Wittenbeck Valerij
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/


#ifndef __CONFIGURATION

#define __CONFIGURATION

#include <vector>
#include <string>
#include "tinyxml.h"
#include "utils.h"

/*! \brief Configuration file wrapper

Loads and stores configuration data (in a xml file). Set \a rootNode 
to root node of xml file first, then use setters and getters to access the data. 
\see CGlobalContainer::init
*/
class CConfiguration
{
      protected:
      TiXmlDocument doc;
      char   rootNode[255];

      void findNodes(const char* name, std::vector<TiXmlElement*> &result, TiXmlElement* start, unsigned int level);  
      
      public:
      CConfiguration(const char* rootNode);
      CConfiguration();

      virtual ~CConfiguration();
      void findNodes(const char* name, std::vector<TiXmlElement*> &result);
      void findNodes(const char* name, std::vector<TiXmlElement*> &result, TiXmlElement* start);   
      TiXmlElement* findNode(const char* name);
      TiXmlElement* findNode(const char* name, TiXmlElement* start);
      void   setRootNode(const char* node);
      
      bool   load(const char* cfgFile);
      bool   parse(const char* str);
      void   save(const char* cfgFile);
      void clear();
      float  getFloat(const char* str, float def = 0.0);
      double getDouble(const char* str, double def = 0.0);
      const char*  getString(const char* str, const char* def = "");
      int    getInteger(const char* str, int def = 0);
      unsigned long  getUnsignedLong(const char* str, unsigned long def = 0);
      bool   getBoolean(const char* str, bool def = false);

      static std::string getText(TiXmlElement* node);
      
      float  getFloat(const char* str, TiXmlElement* start, float def = 0.0);
      double getDouble(const char* str, TiXmlElement* start, double def = 0.0);
      const  char* getString(const char* str, TiXmlElement* start, const char* def = "");
      int    getInteger(const char* str, TiXmlElement* start, int def = 0);
      bool   getBoolean(const char* str, TiXmlElement* start, bool def = false); 
      
      void   setFloat(const char* str, float value);
      void   setDouble(const char* str, double value);
      void   setString(const char* str, const char* value);
      void   setInteger(const char* str, int value);
      void   setBoolean(const char* str, bool value);     
      
      static float  getAttributeFloat(TiXmlElement* node, const char* str, float def = 0.0);
      static unsigned long    getAttributeUnsignedLong(TiXmlElement* node, const char* str, unsigned long def = 0);
      static double getAttributeDouble(TiXmlElement* node, const char* str, double def = 0.0);
      static const char*  getAttributeString(TiXmlElement* node, const char* str, const char* def = "");
      static int    getAttributeInteger(TiXmlElement* node, const char* str, int def = 0);
      static bool   getAttributeBoolean(TiXmlElement* node, const char* str, bool def = false); 
      
      static void   setAttributeFloat(TiXmlElement* node, const char* str, float value);
      static void   setAttributeDouble(TiXmlElement* node, const char* str, double value);
      static void   setAttributeString(TiXmlElement* node, const char* str, const char* value);
      static void   setAttributeInteger(TiXmlElement* node, const char* str, int value);
      static void   setAttributeBoolean(TiXmlElement* node, const char* str, bool value);      

      TiXmlDocument &getDocument();

      TiXmlElement* getNode(const char* name);
      bool addNode(TiXmlElement* node);
      bool addNode(TiXmlElement* node, TiXmlElement* parent);
      bool insertNode(TiXmlElement* node, TiXmlElement* afterThis);
      bool insertNode(TiXmlElement* node, TiXmlElement* afterThis, TiXmlElement* parent);

      bool removeNode(TiXmlElement* element);
      bool removeNode(TiXmlElement* element, TiXmlElement* parent);
      bool removeAttribute(TiXmlElement* element, const char* name);
};

#define XML_LOADER_INIT std::vector<std::string> xml_loader_strings;
#define XML_LOADER_ADD(Y,X) xml_loader_strings.push_back(robotLibPbD::printToString("<%s>%s</%s>", Y, X.c_str(), Y));
#define XML_LOADER_ADD_STRING(Y,X) xml_loader_strings.push_back(robotLibPbD::printToString("<%s>%s</%s>", Y, X.c_str(), Y));
#define XML_LOADER_ADD_INTEGER(Y,X) xml_loader_strings.push_back(robotLibPbD::printToString("<%s>%d</%s>", Y, X, Y));
#define XML_LOADER_ADD_DOUBLE(Y,X) xml_loader_strings.push_back(robotLibPbD::printToString("<%s>%g</%s>", Y, X, Y));
#define XML_LOADER_ADD_FLOAT(Y,X) xml_loader_strings.push_back(robotLibPbD::printToString("<%s>%g</%s>", Y, X, Y));
#define XML_LOADER_ADD_BOOLEAN(Y,X) xml_loader_strings.push_back(robotLibPbD::printToString("<%s>%s</%s>", Y, X ? "true" : "false", Y));
#define XML_LOADER_GET(Y) { \
    for (unsigned int i=0; i<xml_loader_strings.size(); i++)	\
      {								\
	Y += xml_loader_strings[i] + "\n";			\
      }								\
  } 

class CXmlLoaderInterface
{
 public:
  virtual bool writeAsXml(std::string &output, bool addBracket = true) { return false; };
  virtual bool readFromXml(CConfiguration &configuration, TiXmlElement* xmlNode) { return false; };
};

void tokenize(const std::string& str,
                      std::vector<std::string>& tokens,
                      const std::string& delimiters);
                      
#endif
