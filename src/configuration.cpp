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
#include <string>
#include <fstream>

using namespace std;

#include "log.h"
#include "configuration.h"

CConfiguration::CConfiguration()
{
  setRootNode("Data");
}

CConfiguration::CConfiguration(const char* rootNode)
{
  setRootNode(rootNode);
}

TiXmlDocument &CConfiguration::getDocument()
{
  return doc;
}

CConfiguration::~CConfiguration()
{
}

void CConfiguration::clear()
{
    doc.Clear();
}

void CConfiguration::setRootNode(const char* node)
{
     if (node != NULL)
     {
        sprintf(rootNode, "%s", node);
     }
     else
     {
        sprintf(rootNode, "%s", "");
     }
}

bool CConfiguration::load(const char* cfgFile)
{
     bool okay = doc.LoadFile(cfgFile);

     return okay;
}

bool CConfiguration::parse(const char* str)
{
    doc.Parse(str);
    
    return true;
}


unsigned long CConfiguration::getUnsignedLong(const char* str, unsigned long def)
{
    const char* value = getString(str, "");
    if (strcmp(value, "") == 0)
        return def;
    else return strtoul(value, NULL, 10);
}

unsigned long CConfiguration::getAttributeUnsignedLong(TiXmlElement* node, const char* str, unsigned long def)
{
    const char* value = getAttributeString(node,str, "");
    if (strcmp(value, "") == 0)
        return def;
    else return strtoul(value, NULL, 10);
}

void CConfiguration::save(const char* cfgFile)
{
  doc.SaveFile(cfgFile);
}

TiXmlElement* CConfiguration::findNode(const char* name)
{
    TiXmlElement* iter = doc.FirstChildElement( rootNode );
    
    return findNode(name, iter);
}

TiXmlElement* CConfiguration::findNode(const char* name, TiXmlElement* start)
{
    TiXmlElement* iter = start;
    
    if (name == NULL || strcasecmp(name, "") == 0)
      return start;

    char *pch;
    char sBuffer[1024];
    strcpy(sBuffer, name);
    
    pch = strtok(sBuffer, ".");
    
    while (iter && (pch != NULL))
    {
        iter = iter->FirstChildElement(pch);
        pch = strtok(NULL, ".");
    }

    if ((pch == NULL) && (iter))
        return iter;
    else
        return NULL;
}

void tokenize(const std::string& str,
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

void CConfiguration::findNodes(const char* name, std::vector<TiXmlElement*> &result)
{
    TiXmlElement* iter = doc.FirstChildElement( rootNode );
    
    findNodes(name, result, iter, 0);
}

void CConfiguration::findNodes(const char* name, std::vector<TiXmlElement*> &result, TiXmlElement* start, unsigned int level)
{
  std::string str = name;
  std::vector<std::string> tokens;
  tokenize(str, tokens, ".");

  if (level + 1 > tokens.size())
    return;

  if (start == NULL)
    return;

  TiXmlElement* iter = start->FirstChildElement();
  while (iter != NULL)
    {
      if (strcasecmp(iter->Value(), tokens[level].c_str()) == 0)
    {
      if (level + 1 == tokens.size())
        result.push_back(iter);
      else
        findNodes(name, result, iter, level + 1);
    }
      iter = iter->NextSiblingElement();
    }
}

void CConfiguration::findNodes(const char* name, std::vector<TiXmlElement*> &result, TiXmlElement* start)
{
  findNodes(name, result, start, 0);
  return;

  /*
  TiXmlElement* iter = start;

  std::string str = name;
  std::vector<std::string> tokens;
  tokenize(str, tokens, ".");
  
  int index = 0;
  while (iter != NULL && index < tokens.size())
    {
      iter = iter->FirstChildElement(tokens[index].c_str());
      index++;
    }

  if (iter == NULL)
    return;

  while (iter != NULL)
    {
      result.push_back(iter);
      iter = iter->NextSiblingElement(tokens[tokens.size()-1].c_str());
    }
  return;
  */
}



float  CConfiguration::getAttributeFloat(TiXmlElement* node,const  char* str, float def)
{
    const char* value = getAttributeString(node,str, "");
    if (strcmp(value, "") == 0)
        return def;
    else return atof(value);
}

double CConfiguration::getAttributeDouble(TiXmlElement* node, const char* str, double def)
{
    
    const char* value = getAttributeString(node,str, "");
    if (strcmp(value, "") == 0)
        return def;
    else return atof(value);
}


const char* CConfiguration::getAttributeString(TiXmlElement* node, const char* str, const char* def)
{
    const char* tmpstr = node->Attribute(str);
    if (tmpstr == NULL)
       return def;
    else return tmpstr;
}


int    CConfiguration::getAttributeInteger(TiXmlElement* node, const char* str, int def)
{
    const char* value = getAttributeString(node,str, "");
    if (strcmp(value, "") == 0)
        return def;
    else return atoi(value);
}


bool   CConfiguration::getAttributeBoolean(TiXmlElement* node, const char* str, bool def)
{
     const char* value = getAttributeString(node,str,"");
     if (strcmp(value, "") == 0)
        return def;
     else if (strcasecmp(value, "true") == 0)
        return true;
     else if (strcasecmp(value, "false") == 0)
        return false;
     else return def;
}







void   CConfiguration::setAttributeFloat(TiXmlElement* node, const char* str, float value)
{
     char buffer[255];
     sprintf(buffer, "%f", value);
     node->SetAttribute(str, buffer);
}


void   CConfiguration::setAttributeDouble(TiXmlElement* node, const char* str, double value)
{
     char buffer[255];
     sprintf(buffer, "%f", value);
     node->SetAttribute(str, buffer);
}


void   CConfiguration::setAttributeString(TiXmlElement* node, const char* str, const char* value)
{       
     node->SetAttribute(str, value);
}


void   CConfiguration::setAttributeInteger(TiXmlElement* node, const char* str, int value)
{
     char buffer[255];
     sprintf(buffer, "%d", value);
     node->SetAttribute(str, buffer);
}


void   CConfiguration::setAttributeBoolean(TiXmlElement* node, const char* str, bool value)
{
     if (value)
     node->SetAttribute(str, "true");
     else
     node->SetAttribute(str, "false");
}



void CConfiguration::setFloat(const char* str, float value)
{
     char buffer[255];
     sprintf(buffer, "%f", value);
     setString(str, buffer);
}

void CConfiguration::setDouble(const char* str, double value)
{
     char buffer[255];
     sprintf(buffer, "%f", value);
     setString(str, buffer);
}

void CConfiguration::setString(const char* str, const char* value)
{
    TiXmlElement* result = findNode(str);
    
    if (result != NULL)
    {
       TiXmlText* text = result->FirstChild()->ToText();
       text->SetValue(value);
    }
}

void CConfiguration::setInteger(const char* str, int value)
{
     char buffer[255];
     sprintf(buffer, "%d", value);
     setString(str, buffer);
}

void CConfiguration::setBoolean(const char* str, bool value)
{
     if (value)
        setString(str, "true");
     else
        setString(str, "false");
}

std::string CConfiguration::getText(TiXmlElement* node)
{
  if (node != NULL)
    return node->GetText();

  return "";
}

const char* CConfiguration::getString(const char* str, const char* def)
{
    TiXmlElement* result = findNode(str);
    
    if (result != NULL && result->GetText() != NULL)
       return (const char*)result->GetText();
    else
       return def;
}

float CConfiguration::getFloat(const char* str, float def)
{
    return (float) getDouble(str, (double)def);
}

double CConfiguration::getDouble(const char* str, double def)
{
    const char* value = getString(str, "");
    if (strcmp(value, "") == 0)
        return def;
    else return atof(value);
}

int CConfiguration::getInteger(const char* str, int def)
{
    const char* value = getString(str, "");
    if (strcmp(value, "") == 0)
        return def;
    else return atoi(value);
}

bool CConfiguration::getBoolean(const char* str, bool def)
{
     const char* value = getString(str, "");
     if (strcmp(value, "") == 0)
        return def;
     else if (strcasecmp(value, "true") == 0)
        return true;
     else if (strcasecmp(value, "false") == 0)
        return false;
     else return def;
}


const char* CConfiguration::getString(const char* str, TiXmlElement* start, const char* def)
{
  TiXmlElement* result = findNode(str, start);
    
    if (result != NULL && result->GetText() != NULL)
       return (const char*)result->GetText();
    else
       return def;
}

float CConfiguration::getFloat(const char* str, TiXmlElement* start, float def)
{
  return (float) getDouble(str, start, (double)def);
}

double CConfiguration::getDouble(const char* str, TiXmlElement* start, double def)
{
  const char* value = getString(str, start, "");
    if (strcmp(value, "") == 0)
        return def;
    else return atof(value);
}

int CConfiguration::getInteger(const char* str, TiXmlElement* start, int def)
{
  const char* value = getString(str, start,"");
    if (strcmp(value, "") == 0)
        return def;
    else return atoi(value);
}

bool CConfiguration::getBoolean(const char* str, TiXmlElement* start, bool def)
{
  const char* value = getString(str, start,"");
     if (strcmp(value, "") == 0)
        return def;
     else if (strcasecmp(value, "true") == 0)
        return true;
     else if (strcasecmp(value, "false") == 0)
        return false;
     else return def;
}

TiXmlElement* CConfiguration::getNode(const char* name)
{
  return new TiXmlElement( name );
}

bool CConfiguration::addNode(TiXmlElement* element)
{
  if (element == NULL)
    return false;

  doc.LinkEndChild( element );
  return true;
}

bool CConfiguration::addNode(TiXmlElement * element, TiXmlElement* parent)
{ 
  if (element == NULL)
    return false;

  if (parent == NULL)
    return addNode(element);

  parent->LinkEndChild( element );
  return element;
}

bool CConfiguration::insertNode(TiXmlElement* node, TiXmlElement* afterThis)
{
  if (node == NULL)
    return false;

  if (afterThis == NULL)
    return false;

  doc.InsertAfterChild((TiXmlNode*)afterThis, *(TiXmlNode*)node);
  return true;
}

bool CConfiguration::insertNode(TiXmlElement* node, TiXmlElement* afterThis, TiXmlElement* parent)
{
  if (node == NULL)
    return false;

  if (afterThis == NULL)
    return false;

  if (parent == NULL)
    return insertNode(node, afterThis);

  parent->InsertAfterChild(afterThis, *node);
  return true;
}

bool CConfiguration::removeNode(TiXmlElement* element, TiXmlElement* parent)
{
  if (element == NULL)
    return false;

  if (parent == NULL)
    return removeNode(element);

  return parent->RemoveChild(element);
}

bool CConfiguration::removeNode(TiXmlElement* element)
{
  if (element == NULL)
    return false;

  return doc.RemoveChild(element);
}

bool CConfiguration::removeAttribute(TiXmlElement* element, const char* name)
{
  if (element == NULL)
    return false;

  element->RemoveAttribute(name);
  return true;
}

