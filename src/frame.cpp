/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Jäkel Rainer, Wittenbeck Valerij
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/


// TODO: split into several files

#include <math.h>
#include <iostream>
#include <cstdlib>
#include <cstring> 

#include "frame.h"
#include "log.h"

using namespace std;

#undef DEBUG_MESSAGES_FRAME_CPP

namespace robotLibPbD {

bool CFrameContainer::xmlToFrame(CFrame *frame, TiXmlElement* frameNode, bool create)
{
  DataPairs dataPairs;
  return xmlToFrame(frame, frameNode, dataPairs, create);
}


int CFrameContainer::compareBase(CFrame* first, CFrame* second)
{
  std::vector<CFrame*> firstFrames, secondFrames;
  
  while (first != NULL)
    {    
      firstFrames.push_back(first);
      first = first->getBase();
    }

  while (second != NULL)
    {    
      secondFrames.push_back(second);
      second = second->getBase();
    }

  for (unsigned int i=0; i<firstFrames.size(); i++)
    for (unsigned int j=0; j<secondFrames.size(); j++)
      {
	if (firstFrames[i] == secondFrames[j])
	  {
	    
	    return firstFrames.size()-1 - i;
	  }
      }

  return -1;
}


void CFrameContainer::resolve(std::vector<std::string> &in, std::vector<CFrame*> &out)
{
  out.clear();
  for (unsigned int i=0; i<in.size(); i++)
    {
      int id = getFrameByName(in[i].c_str(), false);
      if (id >= 0)
	out.push_back(getFrame(id));
    }
}


  void CFrame::setDofs(const std::vector<double> &dofs_min, const std::vector<double> &dofs_max)
  {
    this->dofs_min.clear();
    this->dofs_min = dofs_min;
    this->dofs_max.clear();
    this->dofs_max = dofs_max;
  }

bool CFrameContainer::xmlToFrame(CFrame *frame, TiXmlElement* frameNode, DataPairs &additionalData, bool create)
{
    // get min max
    std::vector<double> dofs_min(6), dofs_max(6);
    const char* dofs[6] = { "x", "y", "z", "rx", "ry", "rz" };
    for (unsigned int i=0; i<6 && i<dofs_min.size() && i<dofs_max.size(); i++)
    {
        dofs_min[i] = CConfiguration::getAttributeDouble(frameNode, printToString("min_%s", dofs[i]).c_str(), 0.0);
        dofs_max[i] = CConfiguration::getAttributeDouble(frameNode, printToString("max_%s", dofs[i]).c_str(), 0.0);
        //LOG_VERBOSE("min: %g max: %g\n", dofs_min[i], dofs_max[i]);
    }
    frame->setDofs(dofs_min, dofs_max);


    frame->setData(CConfiguration::getAttributeBoolean(frameNode, "data", false));

    double a,b,g,x,y,z, qx,qy,qz,qw;
    a = atof(additionalData.resolveString(CConfiguration::getAttributeString(frameNode, "a", "0.0")).c_str());
    b = atof(additionalData.resolveString(CConfiguration::getAttributeString(frameNode, "b", "0.0")).c_str());
    g = atof(additionalData.resolveString(CConfiguration::getAttributeString(frameNode, "g", "0.0")).c_str());
    x = atof(additionalData.resolveString(CConfiguration::getAttributeString(frameNode, "x", "0.0")).c_str());
    y = atof(additionalData.resolveString(CConfiguration::getAttributeString(frameNode, "y", "0.0")).c_str());
    z = atof(additionalData.resolveString(CConfiguration::getAttributeString(frameNode, "z", "0.0")).c_str());


    qx = atof(additionalData.resolveString(CConfiguration::getAttributeString(frameNode, "qx", "0.0")).c_str());
    qy = atof(additionalData.resolveString(CConfiguration::getAttributeString(frameNode, "qy", "0.0")).c_str());
    qz = atof(additionalData.resolveString(CConfiguration::getAttributeString(frameNode, "qz", "0.0")).c_str());
    qw = atof(additionalData.resolveString(CConfiguration::getAttributeString(frameNode, "qw", "0.0")).c_str());

    if (qx != 0.0 || qw != 0.0 || qw != 0.0 || qw != 0.0)
    {
        CVec quater;
        CMatrix pose;

        quater.x = qx;
        quater.y = qy;
        quater.z = qz;
        quater.w = qw;
        CMathLib::matrixFromQuaternion(quater, pose);

        CVec tmp;
        CMathLib::getOrientation(pose, tmp, tmp);
        a = tmp.x;
        b = tmp.y;
        g = tmp.z;

        a *= 180.0/M_PI;
        b *= 180.0/M_PI;
        g *= 180.0/M_PI;
    }
    else if (a == 0 && b == 0 && g == 0)
    {
        CMatrix mat, test;
        CVec tmp;
        double angle;
        tmp.x = atof(additionalData.resolveString(CConfiguration::getAttributeString(frameNode, "rx", "0.0")).c_str());
        tmp.y = atof(additionalData.resolveString(CConfiguration::getAttributeString(frameNode, "ry", "0.0")).c_str());
        tmp.z = atof(additionalData.resolveString(CConfiguration::getAttributeString(frameNode, "rz", "0.0")).c_str());
        angle = atof(additionalData.resolveString(CConfiguration::getAttributeString(frameNode, "ra", "0.0")).c_str());

        CMathLib::getMatrixFromRotation(mat, tmp, angle);
        CMathLib::getOrientation(mat, tmp, tmp);
        a = tmp.x;
        b = tmp.y;
        g = tmp.z;

        CMathLib::getRotation(test, tmp);
        test.invert();
        test.mul(test, mat);

        double dist = test.length();
        if (dist > 0.01)
        {
            LOG_VERBOSE("Error: xmlToFrame, matrix conversion failed, distance is %g\n", dist);
            a = 0.0;
            b = 0.0;
            g = 0.0;
        }

        a *= 180.0/M_PI;
        b *= 180.0/M_PI;
        g *= 180.0/M_PI;
    }

    CMatrix pose;
    pose.set(1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0);
    CMathLib::getRotation(pose, a*M_PI/180.0, b*M_PI/180.0, g*M_PI/180.0);

    if (a == 0 && b == 0 && g == 0)
    {
        pose.a[0] = atof(additionalData.resolveString(CConfiguration::getAttributeString(frameNode, "a0", "1.0")).c_str());
        pose.a[1] = atof(additionalData.resolveString(CConfiguration::getAttributeString(frameNode, "a1", "0.0")).c_str());
        pose.a[2] = atof(additionalData.resolveString(CConfiguration::getAttributeString(frameNode, "a2", "0.0")).c_str());

        pose.a[4] = atof(additionalData.resolveString(CConfiguration::getAttributeString(frameNode, "a4", "0.0")).c_str());
        pose.a[5] = atof(additionalData.resolveString(CConfiguration::getAttributeString(frameNode, "a5", "1.0")).c_str());
        pose.a[6] = atof(additionalData.resolveString(CConfiguration::getAttributeString(frameNode, "a6", "0.0")).c_str());

        pose.a[8] = atof(additionalData.resolveString(CConfiguration::getAttributeString(frameNode, "a8", "0.0")).c_str());
        pose.a[9] = atof(additionalData.resolveString(CConfiguration::getAttributeString(frameNode, "a9", "0.0")).c_str());
        pose.a[10] = atof(additionalData.resolveString(CConfiguration::getAttributeString(frameNode, "a10", "1.0")).c_str());
    }

    pose.a[12] = x;
    pose.a[13] = y;
    pose.a[14] = z;
    frame->setPose(pose);

    char buffer[255];
    sprintf(buffer, "%s", additionalData.resolveString(CConfiguration::getAttributeString(frameNode, "name", "")).c_str());
    frame->setName(buffer);

    // frame type
    frame->setFrameType(CFrame::FRAME_POSITION);
    sprintf(buffer, "%s", CConfiguration::getAttributeString(frameNode, "type", ""));
    if (strcasecmp(buffer, "velocity") == 0)
    {
        //LOG_VERBOSE("Frame: Type = VELOCITY\n");
        frame->setFrameType(CFrame::FRAME_VELOCITY);
    }
    sprintf(buffer, "%s", additionalData.resolveString(CConfiguration::getAttributeString(frameNode, "base", "")).c_str());

    std::string tmp = buffer;
    std::vector<std::string> tokens;
    tokenize(tmp, tokens, " ");
    if (tokens.size() > 0)
    tmp = tokens.back();
    tokens.clear();

    sprintf(buffer, "%s", tmp.c_str());

    tokenize(tmp, tokens, ":");
    if (tokens.size() > 1)
    {
        if (strcasecmp(tokens[0].c_str(), "geometry") == 0)
        {
            frame->setBaseType(CFrame::BASE_GEOMETRY);
            frame->setBaseName(tokens[1].c_str());
        } else frame->setBaseName(buffer);
    } else frame->setBaseName(buffer);


    frame->setBase(NULL);

    switch (frame->getBaseType())
    {
        case CFrame::BASE_NORMAL:
        default:
        {
            if (strcasecmp(buffer, "") == 0)
            create = false;

            // create referenced base frame if it doesnt exist yet
            int i = getFrameByName(buffer, create);
            if (i >= 0)
            frame->setBase(frames[i]);
        }
    }
    return true;
}


void CFrameContainer::clear()
{
  for (unsigned int i=0; i<frames.size(); i++)
    {
      frames[i]->setBase(NULL);
    }

  for (unsigned int i=0; i<frames.size(); i++)
    {
      delete frames[i];
    }

  frames.clear();
}

bool CFrameContainer::xmlToFrameCombination(CFrameCombination *frame, TiXmlElement* frameNode, DataPairs &additionalData, bool create)
{
    double a,b,g,x,y,z;
    a = CConfiguration::getAttributeDouble(frameNode, "a", 0.0);
    b = CConfiguration::getAttributeDouble(frameNode, "b", 0.0);
    g = CConfiguration::getAttributeDouble(frameNode, "g", 0.0);
    x = CConfiguration::getAttributeDouble(frameNode, "x", 0.0);
    y = CConfiguration::getAttributeDouble(frameNode, "y", 0.0);
    z = CConfiguration::getAttributeDouble(frameNode, "z", 0.0);

    if (a == 0 && b == 0 && g == 0)
    {
        CMatrix mat, test;
        CVec tmp;
        double angle;
        tmp.x = CConfiguration::getAttributeDouble(frameNode, "rx", 0.0);
        tmp.y = CConfiguration::getAttributeDouble(frameNode, "ry", 0.0);
        tmp.z = CConfiguration::getAttributeDouble(frameNode, "rz", 0.0);
        angle = CConfiguration::getAttributeDouble(frameNode, "ra", 0.0);
        CMathLib::getMatrixFromRotation(mat, tmp, angle);
        CMathLib::getOrientation(mat, tmp, tmp);
        a = tmp.x;
        b = tmp.y;
        g = tmp.z;

        CMathLib::getRotation(test, tmp);
        test.invert();
        test.mul(test, mat);

        double dist = test.length();
        if (dist > 0.01)
        {
            LOG_VERBOSE("Error: xmlToFrame, matrix conversion failed, distance is %g\n", dist);
            a = 0.0;
            b = 0.0;
            g = 0.0;
        }
    }


    CMatrix pose;
    pose.set(1.0, 0.0, 0.0, 0.0,
      0.0, 1.0, 0.0, 0.0,
      0.0, 0.0, 1.0, 0.0,
      0.0, 0.0, 0.0, 1.0);
    CMathLib::getRotation(pose, a*M_PI/180.0, b*M_PI/180.0, g*M_PI/180.0);

    pose.a[12] = x;
    pose.a[13] = y;
    pose.a[14] = z;
    frame->setPose(pose);

    char buffer[255];
    sprintf(buffer, "%s", additionalData.resolveString(CConfiguration::getAttributeString(frameNode, "name", "")).c_str());
    frame->setName(buffer);

    // frame type
    frame->setFrameType(CFrame::FRAME_POSITION);
    sprintf(buffer, "%s", CConfiguration::getAttributeString(frameNode, "type", ""));
    if (strcasecmp(buffer, "velocity") == 0)
    {
        //printf("Frame: Type = VELOCITY\n");
        frame->setFrameType(CFrame::FRAME_VELOCITY);
    }
    sprintf(buffer, "%s", additionalData.resolveString(CConfiguration::getAttributeString(frameNode, "base", "")).c_str());

    std::string tmp = buffer;
    std::vector<std::string> tokens;
    tokenize(tmp, tokens, " ");
    if (tokens.size() > 0)
    tmp = tokens.back();
    tokens.clear();

    sprintf(buffer, "%s", tmp.c_str());

    tokenize(tmp, tokens, ":");
    if (tokens.size() > 1)
    {
        if (strcasecmp(tokens[0].c_str(), "geometry") == 0)
        {
            frame->setBaseType(CFrame::BASE_GEOMETRY);
            frame->setBaseName(tokens[1].c_str());
        } else frame->setBaseName(buffer);
    } else frame->setBaseName(buffer);


    frame->setBase(NULL);

    switch (frame->getBaseType())
    {
        case CFrame::BASE_NORMAL:
        default:
        {
            if (strcasecmp(buffer, "") == 0)
            create = false;

            // create referenced base frame if it doesnt exist yet
            int i = getFrameByName(buffer, create);
            if (i >= 0)
            frame->setBase(frames[i]);
        }
    }

    sprintf(buffer, "%s", additionalData.resolveString(CConfiguration::getAttributeString(frameNode, "baseOrientation", "")).c_str());
    frame->setBaseOrientation(NULL);

    if (strcasecmp(buffer, "") == 0)
    create = false;

    // create referenced base frame if it doesnt exist yet
    int i = getFrameByName(buffer, create);
    if (i >= 0)
    {
        frame->setBaseOrientationName(buffer);
        frame->setBaseOrientation(frames[i]);
    }
    return true;
}


CFrame* CFrame::getByName(char* str)
{
    if (hasName(str))
       return this;
    else
       return NULL;
}


void CFrame::removeParent(CFrame *parent)
{ 
#ifdef DEBUG_MESSAGES_FRAME_CPP
  LOG_VERBOSE("Frame: removing frame %s of %s: ", parent->getName(), getName());
#endif

  std::vector<CFrame*> tmp = parents;
  parents.clear();
  for (unsigned int i=0; i<tmp.size(); i++)
    if (tmp[i] != parent)
      parents.push_back(tmp[i]); 
#ifdef DEBUG_MESSAGES_FRAME_CPP
    else
      {
	LOG_VERBOSE("found frame to remove: ");
      } 
  
  LOG_VERBOSE("(");
  for (unsigned int i=0; i<parents.size(); i++)
    LOG_VERBOSE("%s ", parents[i]->getName());
  LOG_VERBOSE(")\n");
#endif
}

void CFrame::addParent(CFrame *parent)
{
#ifdef DEBUG_MESSAGES_FRAME_CPP
  LOG_VERBOSE("Frame: adding parent %s of %s ", parent->getName(), getName());
#endif

  if (parent != NULL && getParentId(parent) < 0)
    {
#ifdef DEBUG_MESSAGES_FRAME_CPP
      LOG_VERBOSE("added %s: ", parent->getName());
#endif
      parents.push_back(parent);
    }
#ifdef DEBUG_MESSAGES_FRAME_CPP
 else
    {
      LOG_VERBOSE("%s already included: ", parent->getName());
    } 
  LOG_VERBOSE("(");
  for (unsigned int i=0; i<parents.size(); i++)
    LOG_VERBOSE("%s ", parents[i]->getName());
  LOG_VERBOSE(")\n");
#endif
}

void* CFrame::getCopy()
{
  CFrame* ptr = new CFrame();
  ptr->setName(getName());
  ptr->setBase(getBase());
  ptr->setBaseName(getBaseName());
  ptr->setBaseType(getBaseType());
  ptr->setPose(getPose());
  ptr->setFrameType(getFrameType());
  ptr->setTime(getTime());
  if (_isLocked)
    ptr->lock();
  else
    ptr->unlock();

  return (void*) ptr;
}

void* CFrameCombination::getCopy()
{
  CFrameCombination* ptr = new CFrameCombination();
  ptr->setName(getName());
  ptr->setBase(getBase());
  ptr->setBaseName(getBaseName());
  ptr->setBaseType(getBaseType());

  ptr->setBaseOrientationName(getBaseOrientationName());
  ptr->setBaseOrientation(getBaseOrientation());
  

  ptr->setPose(getPose());
  ptr->setFrameType(getFrameType());
  ptr->setTime(getTime());
  if (_isLocked)
    ptr->lock();
  else
    ptr->unlock(); 
  
  return (void*) ptr;
}


  void CFrame::getDofs(std::vector<double> &dofs_min, std::vector<double> &dofs_max)
  {
    dofs_max.clear();
    dofs_max = this->dofs_max;
    dofs_min.clear();
    dofs_min = this->dofs_min;
  }

  void CFrame::getDofs(std::vector<unsigned int> &dofs)
  {
    dofs.clear();
    for (unsigned int i=0; i<dofs_min.size() && i<dofs_max.size(); i++)
      if (fabsf(dofs_max[i] - dofs_min[i]) > 1.0e-8)
	dofs.push_back(i);
  }

CFrame::CFrame()
{
  childs.reserve(20);
  frameType = FRAME_POSITION;
  base = NULL;
  name = "";
  _isValid = false;
  baseName = "";
  time = -1;
  baseType = BASE_NORMAL;
  _isData = false;
  _isLocked = false;
  counter = baseCounter = 0;

  dofs_min.resize(6, 0.0);
  dofs_max.resize(6, 0.0);
}


void CFrame::invalidateAll()
{
  invalidate();

  if (base != NULL)
    base->invalidateAll();
}

CFrame::CFrame(char* str)
{ 
  _isData = false;
  childs.reserve(20);
  frameType = FRAME_POSITION;
  base = NULL;
  name = str;
  _isValid = false;
  baseName = "";
  time = -1;
  baseType = BASE_NORMAL;
  _isLocked = false;
  counter = baseCounter = 0;
}

CFrame::~CFrame()
{
#ifdef DEBUG_MESSAGES_FRAME_CPP
  LOG_VERBOSE("Frame: freeing %s\n", getName());
#endif
  setBase(NULL);
}

/*
CMatrix CFrame::getRelativeToBase()
{
    CFrame *frame = this;
    relativePose = pose;
    while (frame->base != NULL)
    {
        frame = frame->base;
        relativePose.mul( frame->pose, relativePose);
    }

    isValid = true;

    return relativePose;
}*/


void CFrame::setRelativePose(const CMatrix &value) 
{ 
  if (_isLocked)
    return;

  relativePose = value; 
  _isValid = true; 
  _isUpdated = true; 
}


void CFrame::setBase(CFrame* base) 
{
  if (this->base != base)
    {
      if (this->base != NULL)
	this->base->removeParent(this);

      this->base = base; 

      if (this->base != NULL)
	this->base->addParent(this);

	if (_isValid)
	  invalidate();
    }
}


bool CFrame::hasName(char* str)
{
  return (strcasecmp(str, name.c_str()) == 0);
}

void CFrame::setName(const char* str)
{
  name = str;
}

void CFrame::update()
{

}


CFrame* CKinematicChain::getLastFrame()
{
  int id = getLength();
  if (id > 0)
    return frames[id-1];
  return NULL;
}

CFrame* CKinematicChain::getFrame(unsigned int id)
{
  if (id < getLength())
    return frames[id];

  return NULL;
}

CDh& CKinematicChain::getDhParameters(unsigned int id)
{
  if (id >= getLength())
    LOG_VERBOSE("Error: this needs fixing (CKinematicChain::getDhParameters)\n");

  return dhParameters[id];
}

CFrame* CKinematicChain::getByName(char* str)
{
        char buffer[255];
        strcpy(buffer, str);
        char* delimiter = strchr(buffer, '_');

        if (delimiter == NULL)
        {
	  return NULL;
        } else
        {
           *delimiter = 0;
           delimiter++;

           int number = atoi(delimiter);
           std::cout << "chain: " << str << " number: " << number << "\n";

           //return this;
           if ((number >= 0) && (number < length))
           {
              return frames[number];
           }
        }

        return NULL;
}


void CKinematicChain::loadFromXml(CConfiguration &config, TiXmlElement* kinChainsNode, CFrameContainer &container)
{
    if (kinChainsNode == NULL)
       return;

    char text[1024];
    int len = CConfiguration::getAttributeInteger(kinChainsNode, "len", 0);

    std::vector<TiXmlElement*> result;
    result.clear();
    config.findNodes("dh", result, kinChainsNode);

    if ((int)result.size() != len)
      {
	len = result.size();
	LOG_VERBOSE("Error: different size of dh entries, using %d\n", len);
      }

    if (len <= 0)
      {
	LOG_ERROR("Error: kinematic chain has length 0\n");
	exit(0);
      }

    sprintf(text, "%s", CConfiguration::getAttributeString(kinChainsNode, "name", ""));
    name = text;
    

    char buffer[1024], baseName[1024];
    sprintf(buffer, "%s", CConfiguration::getAttributeString(kinChainsNode, "base", ""));
    sprintf(baseName, "%s", buffer);
    CFrame* base = NULL;
    // create referenced base frame if it doesnt exist yet
    int id = container.getFrameByName(buffer);
 
    if (id >= 0)
       base = container.frames[id];

    setLength(len);

    totalArmLength = 0.0;

    for (unsigned int i=0; i<result.size(); i++)
    {
      sprintf(text, "%s_%d", name.c_str(), i);
        id = container.getFrameByName(text, true);
        if (id >= 0)
            frames[i] = container.frames[id];
        else
        {
            LOG_VERBOSE("CKinematicChain::loadFromXml() failed.\n");
            return;
        }

        dhParameters[i].speedFactor = CConfiguration::getAttributeDouble(result[i], "speed", 0.01);
	dhParameters[i].ivModel = CConfiguration::getAttributeString(result[i], "ivmodel", "");
        dhParameters[i].angle = 0.0;
	bool trans = CConfiguration::getAttributeBoolean(result[i], "translational", false) || CConfiguration::getAttributeBoolean(result[i], "trans", false);
	//printf("Frame: %s\n", trans ? "translation":"rotation");
	dhParameters[i].rotationalDof = !trans;
        dhParameters[i].rot_z = M_PI / 180.0 * CConfiguration::getAttributeDouble(result[i], "rotz", 0.0);
        dhParameters[i].rot_x = M_PI / 180.0 * CConfiguration::getAttributeDouble(result[i], "rotx", 0.0);
        dhParameters[i].trans_z = CConfiguration::getAttributeDouble(result[i], "transz", 0.0);
        dhParameters[i].trans_x = CConfiguration::getAttributeDouble(result[i], "transx", 0.0);
        dhParameters[i].id = CConfiguration::getAttributeInteger(result[i], "id", -1);
        dhParameters[i].sgn = CConfiguration::getAttributeDouble(result[i], "sgn", 1.0);
        dhParameters[i].min = M_PI / 180.0 * CConfiguration::getAttributeDouble(result[i], "min", -180.0);
        dhParameters[i].max = M_PI / 180.0 * CConfiguration::getAttributeDouble(result[i], "max", 180.0);
        dhParameters[i].axis.x = CConfiguration::getAttributeDouble(result[i], "x", 0.0);
        dhParameters[i].axis.y = CConfiguration::getAttributeDouble(result[i], "y", 0.0);
        dhParameters[i].axis.z = CConfiguration::getAttributeDouble(result[i], "z", 0.0);
	dhParameters[i].useAxis = CConfiguration::getAttributeBoolean(result[i], "axis", false);
        dhParameters[i].rot_z += CConfiguration::getAttributeDouble(result[i], "value", 0.0);

	if (dhParameters[i].useAxis && dhParameters[i].rotationalDof && dhParameters[i].axis.length() < 0.001)
	  {
	    CMatrix pose;
	    pose.a[0] = CConfiguration::getAttributeDouble(result[i], "a0", 1.0);
	    pose.a[1] = CConfiguration::getAttributeDouble(result[i], "a1", 0.0);
	    pose.a[2] = CConfiguration::getAttributeDouble(result[i], "a2", 0.0);
	    
	    pose.a[4] = CConfiguration::getAttributeDouble(result[i], "a4", 0.0);
	    pose.a[5] = CConfiguration::getAttributeDouble(result[i], "a5", 1.0);
	    pose.a[6] = CConfiguration::getAttributeDouble(result[i], "a6", 0.0);
	    
	    pose.a[8] = CConfiguration::getAttributeDouble(result[i], "a8", 0.0);
	    pose.a[9] = CConfiguration::getAttributeDouble(result[i], "a9", 0.0);
	    pose.a[10] = CConfiguration::getAttributeDouble(result[i], "a10", 1.0);
	    
	    double angle;
	    CMathLib::getRotationFromMatrix(pose, dhParameters[i].axis, angle);

	    dhParameters[i].rot_z += angle;
	  }

	if (dhParameters[i].useAxis && dhParameters[i].rotationalDof)
	  dhParameters[i].axis.normalize();

	
	totalArmLength += fabsf(dhParameters[i].trans_z);

	CMatrix pose;
        pose.setDh(dhParameters[i]);
	frames[i]->setPose(pose);
	frames[i]->setName(text);

	if (i==0)
	  {
	    frames[i]->setBase(base);
	    frames[i]->setBaseName(baseName);
	  } else
	  {
	    frames[i]->setBase(frames[i-1]);
	    frames[i]->setBaseName(frames[i-1]->getName());
	  }
    }

    // add tcp frame for kinematic chain
    std::string tcpFrame = name + "_tcp";
    int tcpFrameId = container.getFrameByName(tcpFrame.c_str(), true);
    container.getFrame(tcpFrameId)->setBaseName(frames[getLength()-1]->getName());
    container.getFrame(tcpFrameId)->setBase(frames[getLength()-1]);
    LOG_VERBOSE("Frame: added frame %s for kinematic chain\n", tcpFrame.c_str());


    std::string lastPoseBufferName = name + "_tcp_buffer";
    /*
    int lastPoseBufferId = container.getFrameByName(lastPoseBufferName.c_str(), false);
      if (lastPoseBufferId >= 0)
      {
      printf("Error: buffer frame %s already exists. You have to remove it\n", lastPoseBufferName.c_str());
      exit(0);
      }
      CFrameReference *newFrame = new CFrameReference(frames[getLength()-1]);
      lastPoseBuffer = (CFrame*) newFrame;
      container.add(lastPoseBuffer);
    */
    int lastPoseBufferId = container.getFrameByName(lastPoseBufferName.c_str(), true);
    if (lastPoseBufferId < 0)
      {
	printf("Error: buffer frame %s couldnt be generated\n", lastPoseBufferName.c_str());
	exit(0);
      }
    lastPoseBuffer = container.getFrame(lastPoseBufferId);
}


// Kinematic Chain Class
// - chain of homogenous transformation matrices based
CKinematicChain::CKinematicChain()
{
    dhParameters = 0;
    frames = 0;
    length = 0;
    totalArmLength = 0.0;
    lastPoseBuffer = NULL;
}



/*void CKinematicChain::getPose(CMatrix &result)
{
    CMatrix b;

    if (base != NULL)
       b = base->getRelativeToBase();

    result.mul(b, pose);
    }*/

CKinematicChain::~CKinematicChain()
{
  setLength(0);
}


void CKinematicChain::setLength(int len)
{
    if (length > 0)
    {
            if (dhParameters != NULL)
               delete [] dhParameters;

            if (frames != NULL)
               delete [] frames;

	    frames = NULL;
	    dhParameters = NULL;
            length = 0;
    }

    if (len > 0)
    {
       dhParameters = new CDh[len];
       if (dhParameters == NULL)
   	   {
                LOG_VERBOSE("setLength: malloc failed().\n");
                return;
       }

       frames     = new CFrame*[len];
       if (frames == NULL)
   	   {
                LOG_VERBOSE("setLength: malloc failed().\n");
                return;
       }

       length       = len;
    }
}


CKinematicChainContainer::CKinematicChainContainer()
{
  chain = NULL;
  length = 0;
}

CKinematicChainContainer::~CKinematicChainContainer()
{
    if (chain != NULL)
    {
        delete [] chain;
        chain = NULL;
    }
    length = 0;
}


void CKinematicChainContainer::loadFromXml(CConfiguration &config, TiXmlElement* kinChainsNode,  CFrameContainer &container)
{
    //char text[255];
    int len = CConfiguration::getAttributeInteger(kinChainsNode, "len", 0);
    std::vector<TiXmlElement*> result;
    result.clear();
    config.findNodes("chain", result, kinChainsNode);

    if ((int)result.size() != len)
      {
	len = result.size();
	LOG_VERBOSE("Error: different size of chain entries, using %d\n", len);
      }
    length = len;
    if (chain != NULL)
    {
        delete [] chain;
        chain = NULL;
    }
    chain = new CKinematicChain[len];

    
    for (unsigned int i=0; i<result.size(); i++)
      {
	chain[i].loadFromXml(config, result[i], container);
      }
}



bool CFrameContainer::isRelativeTo(CFrame* first, CFrame* relative)
{
  while (first != NULL)
    {
      if (first == relative)
	return true;

      first = first->getBase();
    }

  return false;
}

bool CFrameContainer::getFrameByName(const char* name, CFrame* &frame)
{
  char buffer[1024];
  //int id;
  std::string object;


  object = name;

  std::vector<std::string> tokens;
  tokenize(object, tokens, " ");

  if (tokens.size() == 0)
    sprintf(buffer, " ");
  else
    sprintf(buffer, "%s", tokens.back().c_str());

  frame = NULL;
  for (unsigned int i=0; i<frames.size(); i++)
    if (strcasecmp(frames[i]->getName(), buffer) == 0)
      {
        frame = frames[i];
        return true;
      }
  
 return false;
}

int CFrameContainer::getFrameByName(const char* name, bool create)
{
  const char* empty ="";
  if (strcasecmp(name, empty) == 0)
    return -1;
  
  char buffer[1024];
  std::string object = name;
  std::vector<std::string> tokens;
  tokenize(object, tokens, " ");
  
  if (tokens.size() == 0)
    sprintf(buffer, "%s", "");
  else
    sprintf(buffer, "%s", tokens.back().c_str());

  for (unsigned int i=0; i<frames.size(); i++)
    if (strcasecmp(frames[i]->getName(), buffer) == 0)
      {
        return (int)i;
      }
  
  if (create && (strlen(buffer) > 0))
    {
      frames.push_back(new CFrame(buffer));
      
      return ((int)frames.size()) - 1;
    }
  return -1;
}


int CFrameContainer::add(CFrame* newFrame)
{
  frames.push_back(newFrame);
  return frames.size()-1;
}


bool CFrameContainer::setBase(char* fr, char* b)
{
  int frame = getFrameByName(fr, false);
  int base = getFrameByName(b, false);

  if (frame < 0)
    return false;

  if (base < 0)
    frames[frame]->setBase(NULL);
  else frames[frame]->setBase(frames[base]);
  return true;
}


void CFrameContainer::checkBaseFrames()
{
  for (unsigned int i=0; i<frames.size(); i++)
    {
      if (frames[i]->getBase() != NULL)
	{
	  int id = -1;
	  for (unsigned int j=0; j<frames.size(); j++)
	    {
	      if (frames[j] == frames[i]->getBase())
		{
		  id = j;
		  break;
		}
	    }
	  if (id < 0)
	    {
	      LOG_VERBOSE("Removing base of frame %s\n", frames[i]->getName());
	      frames[i]->setBase(NULL);
	      frames[i]->setBaseName("");
	    }
	}
    }
}

void CFrameContainer::updateBaseLinks()
{
  updateBaseLinks(frames);
}

void CFrameContainer::updateBaseLinks(CFrame* frame)
{
  if (frame == NULL)
    return;

  std::vector<CFrame*> frames;
  frames.push_back(frame);
  updateBaseLinks(frames);
}

void CFrameContainer::updateBaseLinks(std::vector<CFrame*> &frames)
{
  for (unsigned int i=0; i<frames.size(); i++)
    {
      frames[i]->setBase(NULL);
      if (frames[i]->getBaseType() == CFrame::BASE_COMBINED)
	((CFrameCombination*)frames[i])->setBaseOrientation(NULL);
	
      frames[i]->getParents().clear();
    }

  for (unsigned int i=0; i<frames.size(); i++)
    {
      //LOG_VERBOSE("Info: Frame is %s with base %s\n", frames[i]->getName(), frames[i]->getBaseName());

      if (strcasecmp(frames[i]->getBaseName(), "") != 0)
	{
	  int id = getFrameByName(frames[i]->getBaseName(), false);
	  if (id >= 0)
	    {
	      frames[i]->setBase(this->frames[id]);
	      LOG_VERBOSE("Info: setting base of %s to %s\n", frames[i]->getName(), this->frames[id]->getName());
	    }
	  else 
	    {
	      LOG_VERBOSE("Error: Couldnt find base named %s of frame %s\n", frames[i]->getBaseName(), frames[i]->getName());
	    }
	}

      
      if (frames[i]->getBaseType() == CFrame::BASE_COMBINED)
	if (strcasecmp(((CFrameCombination*)frames[i])->getBaseOrientationName(), "") != 0)
	  {
	    int id = getFrameByName(((CFrameCombination*)frames[i])->getBaseOrientationName(), false);
	    if (id >= 0)
	      {
		((CFrameCombination*)frames[i])->setBaseOrientation(this->frames[id]);
		LOG_VERBOSE("Info: setting base orientiation of %s to %s\n", frames[i]->getName(), this->frames[id]->getName());
	      }
		else LOG_VERBOSE("Error: Couldnt find base named %s of frame %s\n", ((CFrameCombination*)frames[i])->getBaseOrientationName(), frames[i]->getName());
	}
    }
}


void CFrameContainer::loadFromFile(const char* filename)
{
  CConfiguration config("Data");
  //bool okay = config.load(filename);
  config.load(filename);

  DataPairs additionalData;
  loadFromFile(filename, additionalData, config, true);
}

void CFrameContainer::loadFromFile(const char* filename, DataPairs &additionalData, CConfiguration &config, bool loadAll)
{
  std::vector<TiXmlElement*> result;
  config.findNodes("Frames.Frame", result);
  for (unsigned int i=0; i<result.size(); i++)
    {
      if (!loadAll)
	if (!CConfiguration::getAttributeBoolean(result[i], "data", false))
	  continue;

      int frame = getFrameByName(CConfiguration::getAttributeString(result[i], "name", ""), false);

      CFrame* tmp;

      if (frame < 0)
	{
	  tmp = new CFrame();
	  add(tmp);
	} else tmp = frames[frame];

      xmlToFrame(tmp, result[i], additionalData, false);
    }

  invalidate();
}


// Denavit Hartenberg Storage Class
CDh::CDh()
{
  speedFactor = 0.01;
  rotationalDof = true;
  rot_z = trans_z = rot_x = trans_x = angle = 0.0;
  sgn = 1.0;
  min = -180.0;
  max = 180.0;
  useAxis = false;
  axis.set(0.0, 0.0, 0.0);
  id = -1;
}

// get variable rotation angle (variable portion of theta)
double CDh::getAngle()
{
	return sgn * this->angle;
}

// set variable rotation angle (variable portion of theta)
void CDh::setAngle(double angle)
{
	this->angle = sgn * angle;
}

// set fixed dh parameters
void CDh::set(double rot_z, double trans_z, double rot_x, double trans_x)
{
    this->rot_z = rot_z;
    this->trans_z = trans_z;
    this->rot_x = rot_x;
    this->trans_x = trans_x;
}



std::string CFrameInterface::getFrameAsXml()
{
  if (frame == NULL)
    return "";

  CVec t1, t2;
  CMatrix m;
  m = frame->getPose();
  CMathLib::getOrientation(m, t1, t2);

  char buffer[1024];

  if (frame->getBaseType() == CFrame::BASE_COMBINED)
    {
      sprintf(buffer, "<Frame name=\"%s\" base=\"%s\" baseOrientation=\"%s\"  a=\"%g\" b=\"%g\" g=\"%g\" x=\"%g\" y=\"%g\" z=\"%g\" />\n",
	      frame->getName(), frame->getBaseName(), ((CFrameCombination*)frame)->getBaseOrientationName(),
	      180.0/M_PI * t1.x, 180.0/M_PI * t1.y, 180.0/M_PI * t1.z, m.a[12], m.a[13],  m.a[14]); 
    } else
    {
      sprintf(buffer, "<Frame name=\"%s\" base=\"%s\" a=\"%g\" b=\"%g\" g=\"%g\" x=\"%g\" y=\"%g\" z=\"%g\" />\n",
	      frame->getName(), frame->getBaseName(),
	      180.0/M_PI * t1.x, 180.0/M_PI * t1.y, 180.0/M_PI * t1.z, m.a[12], m.a[13],  m.a[14]); 
    }
  return buffer;
};


CFrameCombination::CFrameCombination()
{
  frameType = FRAME_POSITION;
  base = NULL;
  baseOrientation = NULL;
  name = "";
  _isValid = false;
  baseName = "";
  baseOrientationName = "";
  time = -1;
  baseType = BASE_NORMAL;
  _isLocked = false;
  counter = baseCounter = 0;

  //LOG_VERBOSE("Created combined frame.\n");

  baseType = CFrame::BASE_COMBINED;
}

CFrameCombination::~CFrameCombination()
{
#ifdef DEBUG_MESSAGES_FRAME_CPP
  LOG_VERBOSE("FrameCombination: freeing %s\n", getName());
#endif
  setBaseOrientation(NULL);
  setBase(NULL);
}



CFrame* CFrameCombination::getBaseOrientation()
{
  return baseOrientation;
}

void CFrameCombination::setBaseOrientation(CFrame* baseOrientation)
{
  if (this->baseOrientation != baseOrientation)
    {
      if (this->baseOrientation != NULL && this->baseOrientation != this->base)
	this->baseOrientation->removeParent(this);
 
      this->baseOrientation = baseOrientation; 

      if (baseOrientation != NULL)
	baseOrientation->addParent(this);

	if (_isValid)
	  invalidate();
    } 
}


void CFrameCombination::invalidateAll()
{
  invalidate();

  if (base != NULL)
    base->invalidateAll();

  if (baseOrientation != NULL)
    baseOrientation->invalidateAll();
}


 

CFrameReference::CFrameReference(CFrame *reference)
{
  this->reference = reference;
}

CFrameReference::CFrameReference()
{
  reference = NULL;
}



void CFrame::invalidate()
{
  // _isUpdated = false;
    _isValid = false;

    //printf("Frame: Invalidating %s\n", name.c_str());
    /*
  if (_isLocked)// || time >= 0)
    return;

    _isValid = false;  */  

    for (unsigned int i=0; i<parents.size(); i++)
      parents[i]->invalidate();
}



// test


 /*
void CFrame::getRelativeToBase(CMatrix &mat)
{
  if (_isValid || _isLocked) 
      {
	mat = relativePose;
	return;
      } 

  if (getBase() == NULL)
    {
      mat = relativePose = pose;
      return;
    }

  childs.clear();
  CFrame *frame = getBase();
  while (frame != NULL)
    {
      childs.push_back(frame);
      if (frame->_isValid || frame->_isLocked)
	{ 
	  break;
	}
      frame = frame->getBase();
    }

  childs.back()->getRelativeToBase(tmpMatrix);
  for (int i=(int)childs.size()-2; i>= 0; i--)
    {
      childs[i]->relativePose.mulNoAlloc(childs[i+1]->relativePose, childs[i]->pose);
      childs[i]->_isValid = true;
    }
  
  relativePose.mulNoAlloc(childs[0]->relativePose, pose);
    
  _isValid = true;

  mat = relativePose;
}

CMatrix CFrame::getRelativeToBase()
  {
    CMatrix tmpMatrix;
    getRelativeToBase(tmpMatrix);
    return tmpMatrix;
  }
 */
/*
void CFrame::getRelativeToBase(CMatrix &mat)
{
  if (_isValid || _isLocked) 
      {
	mat = relativePose;
	return;
      } 

  if (getBase() != NULL)
    {
      getBase()->getRelativeToBase(tmpMatrix);
      relativePose.mulNoAlloc(tmpMatrix, pose);
    } else relativePose = pose;
  
  _isValid = true;

  mat = relativePose;
}

CMatrix CFrame::getRelativeToBase()
  {
    
  if (_isValid || _isLocked) 
      {
	return relativePose; 
      } 

  if (getBase() != NULL)
    {
      getBase()->getRelativeToBase(tmpMatrix);
      relativePose.mulNoAlloc(tmpMatrix, pose);
    } else relativePose = pose;
  
  _isValid = true;

  return relativePose;
  }
*/	

}
