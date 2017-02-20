/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Jäkel Rainer, Wittenbeck Valerij
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "datapairs.h"
//#include "manipulation/mathexpr.h"
#include "muParser.h"

#include "log.h"
#include "utils.h"
#include "vecmath.h"


bool DataPairs::getItem(unsigned int index, std::vector<std::string> &item)
{
    if (index >= values.size())
       return false;
    
    item = values[index];
    
    return true;
}
      

void DataPairs::copyFrom(DataPairs &other)
{
     clear();
     for (unsigned int i=0; i<other.getLength(); i++)
     {
         std::vector<std::string> item ;
         if (other.getItem(i, item) && item.size() > 1)
          add(item[0], item[1]);
     }
}

void DataPairs::print()
{
     for (unsigned int i=0; i<values.size(); i++)
     {
       LOG_VERBOSE("%s: ", values[i][0].c_str());
         for (unsigned int j=1; j<values[i].size(); j++)
	   LOG_VERBOSE("%s ",  values[i][j].c_str());
	 LOG_VERBOSE("\n");
     }
}

// sets max array size
void DataPairs::setMaxSize(unsigned int size)
{
    values.reserve(size);
}

// returns first index of item with name "str"
// name[10] = "hund" value[10] = "dobermann"
// name[12] = "hund" value[12] = "dackel"
// -> getFirstIndex("hund") = 10
int DataPairs::getFirstIndex(std::string str)
{
    for (unsigned int i=0; i<values.size(); i++)
      if (values[i].size() > 0 && strcasecmp(values[i][0].c_str(), str.c_str()) == 0)
           return i;
           
    return -1;
}

// returns array with indize of all items with name "str"
int DataPairs::getIndex(std::string str, std::vector<unsigned int> &ids)
{
    ids.clear();
    
    for (unsigned int i=0; i<values.size(); i++)
        if (strcasecmp(values[i][0].c_str(), str.c_str())  == 0)
           ids.push_back(i);
          
    if (ids.size() == 0)
        return -1;
         
    return ids[0];
}

// below: get item with name "str" and return its value as type "Int", "Bool", etc 
int DataPairs::getInt(std::string str, int def)
{
    int id = getFirstIndex(str);
    
    if (id >= 0)
       return atoi(values[id][1].c_str());
       
    return def;     
}

float DataPairs::getFloat(std::string str, float def)
{
    int id = getFirstIndex(str);
    
    if (id >= 0)
       return atof(values[id][1].c_str());
       
    return def; 
}


std::string DataPairs::getString(std::string str, std::string def)
{
    int id = getFirstIndex(str);
    
    if (id >= 0)
       return values[id][1];
       
    return def; 
}

void DataPairs::clear()
{
    values.clear();
}

unsigned int DataPairs::getLength()
{
    return values.size();
}

// get all strings
void DataPairs::getStrings(std::string str, std::vector<std::string> &result)
{
    result.clear();
    std::vector<unsigned int> ids;
    getIndex(str, ids);
    
    for (unsigned int i=0;i<ids.size();i++)
        result.push_back(values[ids[i]][1]);
       
    return; 
}


bool DataPairs::getBool(std::string str, bool def)
{
    int id = getFirstIndex(str);
    
    if (id >= 0)
       return strcasecmp(values[id][1].c_str(), "true") == 0;
       
    return def; 
}

void DataPairs::add(std::string n, std::string v, bool replace)
{
  int id = getFirstIndex(n);

  if (id >= 0 && replace)
    {
      values[id].clear();
      values[id].push_back(n);
      values[id].push_back(v);
      return;
    }
  
  std::vector<std::string> newItem;
  newItem.push_back(n);
  newItem.push_back(v);
  
  values.push_back(newItem);
}


bool DataPairs::getIntValue(std::string str, int &result)
{
  int id = getFirstIndex(str);
  if (id < 0)
    return false;

  try
    {
      result = (int) atof(values[id][1].c_str());
    }
  catch (...)
    {
      return false;
    }
  return true;
}

bool DataPairs::getFloatValue(std::string str, float &result)
{
  int id = getFirstIndex(str);
  if (id < 0)
    return false;

  try
    {
      result = atof(values[id][1].c_str());
    }
  catch (...)
    {
      return false;
    }
  return true;
}

bool DataPairs::getStringValue(std::string str, std::string &result)
{
  int id = getFirstIndex(str);
  if (id < 0)
    return false;

  result = values[id][1];
  return true;
}

double parseMathExprRandomDouble01()
{
  return robotLibPbD::getUniform();
}

double parseMathExprRandomInteger(double first, double second)
{
  return (double)robotLibPbD::getUniform((int) first, (int) second);
}

std::string parseMathExpr(std::string output)
{
  using namespace mu;

  std::string result;

  std::vector<std::string> tokens;
  robotLibPbD::strtokenize(output, tokens, " ");
  
  Parser p;
  //p.DefineFun("rand", parseMathExprRandomInteger, false);
  //p.DefineFun("uniform", parseMathExprRandomDouble01, false);

  result = "";
  for (unsigned int i=0; i<tokens.size(); i++)
    {
      //printf("%s ->", tokens[i].c_str());

      try
	{
	  p.SetExpr(tokens[i]);
	  double value = p.Eval();
	  //printf("Math: %s = %g\n", tokens[i].c_str(), value);

	  tokens[i] = robotLibPbD::printToString("%g", value);
	}
      catch (Parser::exception_type &e)
	{
	}
      
      if (i+1 < tokens.size())
	result += tokens[i] + " ";
      else
	result += tokens[i];
    }

  return result;
}

bool DataPairs::getBoolValue(std::string str, bool &result)
{
  int id = getFirstIndex(str);
  if (id < 0)
    return false;

  result = strcasecmp(values[id][1].c_str(), "true") == 0;
  return true;
}


void DataPairs::resolveString(std::string input, std::string &output)
{
  char buffer[1024], buffer2[1024], buffer3[1024];
  sprintf(buffer, "%s", input.c_str());
  char *start, *end;

 

  start = &buffer[0];
  while (start != NULL && *start != '\0')
    {
      if (*start == '[')
	{
	  bool replaced = false;
	  start++;
	  end = start;
	  while (end != NULL && *end != '\0')
	    {
	      if (*end == ']')
		{
		  *end = '\0';
		  sprintf(buffer3, "%s", start);

		  if (getFirstIndex(buffer3) >= 0)
		    sprintf(buffer3, "%s", getString(buffer3).c_str());
		  else
		    sprintf(buffer3, "%s", buffer3);

		  end++;
		  sprintf(buffer2, "%s", end);

		  start--;
		  sprintf(start, "%s%s", buffer3, buffer2);
		  
		  replaced = true;
		  break;
		}
	      end++;
	    }

	  if (replaced)
	    {
	      start = &buffer[0];
	      continue;
	    }
	}

      start++;
    }

  output = buffer;
  
  if (true)
    {
      output = parseMathExpr(output);
    }
}


std::string DataPairs::resolveString(std::string input)
{
  std::string output;
  resolveString(input, output);
  return output;
}

void  DataPairs::resolve()
{
  for (unsigned int i=0; i<values.size(); i++)
    for (unsigned int j=1; j<values[i].size(); j++)
      values[i][j] = resolveString(values[i][j]);
}



void DataPairs::loadFromXml(CConfiguration &config, TiXmlElement* node, DataPairs* replace)
{
  std::vector<TiXmlElement*> result;
  config.findNodes("Value", result, node);
  for (unsigned int i=0; i<result.size(); i++)
    {
      std::string name, value;
      name = config.getAttributeString(result[i], "name", "");
      value = config.getAttributeString(result[i], "value", "");

      if (replace != NULL)
	{
	  name = replace->resolveString(name);
	  value = replace->resolveString(value);
	}

      if (name != "")
	add(name, value, true);
    }
}
