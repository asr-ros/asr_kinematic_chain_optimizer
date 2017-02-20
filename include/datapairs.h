/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Jäkel Rainer, Wittenbeck Valerij
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/


#ifndef __DATAPAIRS

#define __DATAPAIRS

#include <string>
#include <cstdlib>
#include <vector> 
#include <utility>

#include "configuration.h"


/*! \brief Data Storage Class (Attribute-Value Pairs)
*/
class DataPairs
{
      protected:
      /// Attribute Names and Values
      std::vector< std::vector< std::string > > values;
      
      public:
      /// Returns Number of pairs stored
      unsigned int getLength();

      /// Sets number of storable items to \a size
      void setMaxSize(unsigned int size);
      
      bool getItem(unsigned int index, std::vector< std::string > &item);
      
      /// Returns Max. number of stored pairs
      unsigned int getMaxSize();
      /// Adds the attribute-value pair \a attribute, \a value
      void add(std::string attribute, std::string value, bool replace = false);
      
      /// Clears storage buffer
      void clear();

      /*! \brief Retrieves all indexes of Attribute-Value pairs with Attribute name equal to attr
      
      \param str Attribute-Name to search for
      \param ids Array of retrieved indexes, has to be allocated
      \param len Number of retrieved pairs
      \return Index of first result
      */
      int getIndex(std::string attr, std::vector<unsigned int> &ids);
      int getFirstIndex(std::string str);
      
      /*! \brief Retrieves all Attribute-Value pairs with Attribute name equal to attr
      
      \param str Attribute-Name to search for
      \param result Array of pointers to retrieved Value-strings, has to be of allocated
      \param len Number of retrieved pairs
      \return Index of first result
      */
      void getStrings(std::string str, std::vector<std::string> &result);

      
      /// \return Value of Attribute str as integer 
      int getInt(std::string str, int defaultValue = 0);
      /// \return Value of Attribute str as float 
      float getFloat(std::string str, float defaultValue = 0.0);
      /// \return Value of Attribute str as string 
      std::string getString(std::string str, std::string defaultValue = "");
      /// \return Value of Attribute str as boolean 
      bool getBool(std::string str, bool defaultValue = false);


      /// \return Value of Attribute str as integer 
      bool getIntValue(std::string str, int &result);
      /// \return Value of Attribute str as float 
      bool getFloatValue(std::string str, float &result);
      /// \return Value of Attribute str as string 
      bool getStringValue(std::string str, std::string &result);
      /// \return Value of Attribute str as boolean 
      bool getBoolValue(std::string str, bool &result);
      
      /// Copies all Attribute-Value pairs
      void copyFrom(DataPairs &other);
      
      /// Prints all Attribute-Value pairs
      void print();

      void resolveString(std::string input, std::string &output);
      std::string resolveString(std::string input);
      void resolve();

      void loadFromXml(CConfiguration &config, TiXmlElement* node, DataPairs* replace = NULL);
};


#endif
