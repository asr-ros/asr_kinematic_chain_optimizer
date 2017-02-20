/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Jäkel Rainer, Wittenbeck Valerij
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/



#ifndef __FRAME

#define __FRAME

#include <vector>

#include "configuration.h"
#include "vecmath.h"
#include "datapairs.h"
#include "interfaces.h"

namespace robotLibPbD {

/*! \brief Robot types with implemented inverse kinematics
*/


/*! \brief Frame in cartesian space
*/
class CFrame : public CCopyInterface
{
    protected:
  enum {
    FRAME_X,
    FRAME_Y,
    FRAME_Z,
    FRAME_RX,
    FRAME_RY,
    FRAME_RZ,
    FRAME_DOFS
  };

  std::vector<double> dofs_min, dofs_max;
  bool _isData;
  
  CMatrix tmpMatrix;
  bool _isValid, _isUpdated, _isLocked;
  CFrame *base; ///< (Relative) base frame (f.e. the static robot frame or the frame of the previous link in a kinematic chain)
  CMatrix pose; ///< Translation and rotation relative to base frame (f.e. denavit hartenberg matrix)
  CMatrix relativePose;
  std::string name, baseName; ///< Name associated with frame
  int baseType;
  int frameType;
  double time;

  std::vector<CFrame*> parents, childs;
    public:
  unsigned long int counter, baseCounter;

	enum
	{
	  BASE_NORMAL,
	  BASE_GEOMETRY,
	  BASE_COMBINED
	};

	enum
	{
	  FRAME_POSITION,
	  FRAME_VELOCITY
	};
	
    
    CFrame();
    CFrame(char* name);
    virtual ~CFrame();
    
    virtual CMatrix getRelativeToBase(); ///< Returns pose in the base (frame with no predecessor) frame
    virtual void getRelativeToBase(CMatrix &mat);

    std::vector<CFrame*>& getParents() { return parents; };
    void removeParent(CFrame *parent);
    void addParent(CFrame *parent);
    int getParentId(CFrame *parent) { for (unsigned int i=0; i<parents.size(); i++) if (parents[i] == parent) return (int)i; return -1; };

    bool isData() { return _isData; };
    bool setData(bool value) { _isData = value; return value;};
    bool isValid() { return _isValid; };
    bool isUpdated() { return _isUpdated; };
    void lock() { _isLocked = true; };
    void unlock() { _isLocked = false; };

    virtual void setName(const char* str);
    virtual char* getName() { return (char*) name.c_str(); };
    virtual void setBaseName(const char* str) { baseName = str; };
    virtual char* getBaseName() { return (char*) baseName.c_str(); };
    virtual bool hasName(char* str);

    virtual void setBaseType(unsigned int type) { baseType = type; };
    virtual unsigned int getBaseType() { return baseType; };

    virtual void setRelativePose(const CMatrix &value);

    virtual void setFrameType(unsigned int type) { frameType = type; };
    virtual unsigned int getFrameType() { return frameType; };
    
    virtual CFrame* getBase();
    virtual void setBase(CFrame* base);
    virtual CMatrix getPose() { return pose; };
    virtual void getPose(CMatrix &pose) { pose = this->pose; };
    inline virtual void setPose(const CMatrix &pose) { this->pose = pose; invalidate(); };
    inline virtual void setPoseNoInvalidation(const CMatrix &pose) { this->pose = pose;};
    
    virtual CFrame* getByName(char* str); ///< Returns frame associated with /a str
    virtual void update(); ///< Updates the frames
    virtual void invalidate();
    virtual void invalidateAll();
    
    void getDofs(std::vector<unsigned int> &dofs);
    void getDofs(std::vector<double> &dofs_min, std::vector<double> &dofs_max);
    void setDofs(const std::vector<double> &dofs_min, const std::vector<double> &dofs_max);
    
    
    virtual void setTime(double time) { this->time = time; };
    virtual double getTime() { return time; };
    virtual void* getCopy();
};

class CFrameInterface
{
 protected:
  CFrame* frame;
 public:
  CFrameInterface() { frame = NULL; };
  virtual void setFrame(CFrame* frame) { this->frame = frame; };
  virtual CFrame* getFrame() { return this->frame; }; 
  virtual void invalidate() { if (frame != NULL) frame->invalidate(); };
  virtual void invalidateAll() { if (frame != NULL) frame->invalidateAll(); };
  virtual std::string getFrameAsXml();
};
  

/*! \brief Frame in cartesian space
*/
class CFrameReference : public CFrame
{
 protected:
  CFrame *reference; 
 public: 
  CFrameReference(CFrame *reference);
  CFrameReference();

  void setPose(const CMatrix &pose) { relativePose = this->pose = pose; };
  CMatrix getRelativeToBase();
  void getRelativeToBase(CMatrix &pose);
};

/*! \brief Frame in cartesian space
*/
class CFrameCombination : public CFrame
{
    protected:
  CFrame *baseOrientation; ///< (Relative) base frame (f.e. the static robot frame or the frame of the previous link in a kinematic chain)
  
	std::string baseOrientationName; ///< Name associated with frame
 public:

    CFrameCombination();
    virtual ~CFrameCombination();

    CMatrix getRelativeToBase();
    void getRelativeToBase(CMatrix &pose);///< Returns pose in the base (frame with no predecessor) frame
    
    virtual void setBaseOrientationName(const char* str) { baseOrientationName = str; };
    virtual char* getBaseOrientationName() { return (char*) baseOrientationName.c_str(); };

    virtual CFrame* getBaseOrientation();
    virtual void setBaseOrientation(CFrame* base);
    
    virtual void invalidate();
    virtual void invalidateAll();

    virtual void* getCopy();
};


class CFrameContainer
{
 public:
  std::vector<CFrame*> frames;
  
  CFrameContainer() { frames.reserve(1000); };
  virtual ~CFrameContainer() { clear(); };

  void clear();

  unsigned int size() { return frames.size(); };
  int add(CFrame* newFrame);
  bool getFrameByName(const char* name, CFrame* &frame);
  int getFrameByName(const char* name, bool create = false);
  bool setBase(char* frame, char* base);
  /*! \brief Transforms a <FRAME> (xml object) into a frame
   */
  void invalidate(bool time = false);
  bool xmlToFrame(CFrame *frame, TiXmlElement* frameNode, bool create = false);
  bool xmlToFrame(CFrame *frame, TiXmlElement* frameNode, DataPairs &additionalData, bool create = false);
  bool xmlToFrameCombination(CFrameCombination *frame, TiXmlElement* frameNode, DataPairs &additionalData, bool create = false);

  void updateBaseLinks();
  void updateBaseLinks(std::vector<CFrame*> &frames);
  void updateBaseLinks(CFrame* frame);
  void loadFromFile(const char* filename);
  void loadFromFile(const char* filename, DataPairs &additionalData, CConfiguration &config, bool loadAll);

  CFrame* getFrame(unsigned int id) { if (id < frames.size()) return frames[id]; return NULL; };
  CFrame* getFrame(const char* name) { int id = getFrameByName(name, false); if (id < 0) return NULL; return frames[id]; };

  std::vector<CFrame*> getFrames() { return frames; };
  int compareBase(CFrame* first, CFrame* second);

  void checkBaseFrames();

  static bool isRelativeTo(CFrame* first, CFrame* relative);

  void resolve(std::vector<std::string> &in, std::vector<CFrame*> &out);
};  



/*! \brief Denavit Hartenberg Link information
*/
class CDh
{
	public:
  std::string ivModel;
  double speedFactor;
  bool rotationalDof;
  bool useAxis;
  CVec axis;
	double rot_z; ///< Rotation offset around z-axis
    double trans_z; ///< Translation offset along z-axis
    double rot_x; ///< Rotation offset around rotated x-axis
    double trans_x; ///< Translation offset along rotated x-axis
    double angle; ///< Current angle of rotation aroung z-axis (changes with the real servo position)
	double sgn; ///< Direction of rotation (+1.0 clockwise, -1.0 counterclockwise)
	int id; ///< Index of servo motor associated with the frame
    CDh();
    double min, max;
    double getAngle(); ///< Returns current angle
    void  setAngle(double angle); ///< Sets current angle
    void  set(double rot_z, double trans_z, double rot_x, double trans_x);
};


/*! \brief Kinematic chain
*/
class CKinematicChain
{
 protected:
  std::string name;
  CMatrix tmpMatrix;
	public:
  void setName(std::string value) { this->name = value; };
  std::string getName() { return this->name; };
  CFrame* lastPoseBuffer;
  
    CDh     *dhParameters; ///< Array of denavit hartenberg parameters
	
	CFrame  **frames; ///< Array of frames associated with every link in the chain
	int     length; ///< Number of links in the chain

	double totalArmLength;

    CKinematicChain();
    virtual ~CKinematicChain();
    
    /*! \brief Updates denavit hartenberg matrices
    */
    void    update();
    CDh& getDhParameters(unsigned int id);
    CFrame* getFrame(unsigned int id);

    CFrame* getLastFrame();
    /*! \brief Returns frame object if frame with name \a str exists in the chain
    */
    CFrame* getByName(char* str);
    
    /*! \brief Returns the pose of the last link in the chain (in the base frame)
    */
    //void    getPose(CMatrix &result);

    /*! \brief Sets chain length and allocates memory
    */
    void    setLength(int len);
    unsigned int getLength() { return (unsigned int) length; };
	
    /*! \brief Transforms <CHAIN> (xml object) into kinematic chain object
    */
    void loadFromXml(CConfiguration &config, TiXmlElement* kinChainsNode,  CFrameContainer &container);
        
    void invalidate();
};

/*! \brief Kinematic Robot Model

The kinematic model of the robot is given by a number of (connected) kinematic chains.
*/
class CKinematicChainContainer
{
    public:
    int length; ///< Number of kinematic chains
    
    CKinematicChain *chain; ///< Array of kinematic chains
    
    unsigned int getLength() { return (unsigned int) length; };

    CKinematicChainContainer();
    virtual ~CKinematicChainContainer();
    
    void update(); ///< Updates denavit hartenberg matrices of all chains
    void loadFromXml(CConfiguration &config, TiXmlElement* kinChainsNode,  CFrameContainer &container); ///< Loads kinematic structure of robot from <KINEMATICCHAINS> (xml object)
};
 
inline CMatrix CFrameReference::getRelativeToBase()
{
    if (_isValid || _isLocked) 
      { 
	return relativePose;
      }
    _isValid = true;
    reference->getRelativeToBase(relativePose);
    pose = relativePose;  
   return relativePose;
}

inline void CFrameReference::getRelativeToBase(CMatrix &mat)
{
  if (_isValid || _isLocked) 
      {
	mat = relativePose;
	return;
      }
  _isValid = true;
   
  reference->getRelativeToBase(relativePose);
  mat = pose = relativePose;  
}

    
inline CMatrix CFrameCombination::getRelativeToBase()
{
  if (_isValid || _isLocked)
    {
      return relativePose;
    }
 
  _isValid = true;

  if (getBaseOrientation() != NULL)
    {
      getBaseOrientation()->getRelativeToBase(relativePose);
      relativePose.a[12] = 0.0;
      relativePose.a[13] = 0.0;
      relativePose.a[14] = 0.0;
    }
  else
    relativePose.unity();

  if (getBase() != NULL)
    { 
      getBase()->getRelativeToBase(tmpMatrix);
      relativePose.a[12] = tmpMatrix.a[12];
      relativePose.a[13] = tmpMatrix.a[13];
      relativePose.a[14] = tmpMatrix.a[14];
    }

  relativePose.mul(relativePose, pose);
  
  return relativePose;
}

inline void CFrameCombination::getRelativeToBase(CMatrix &mat)
{
  if (_isValid || _isLocked)
    {
      mat = relativePose;
      return;
    }
 
  _isValid = true;

  if (getBaseOrientation() != NULL)
    {
      getBaseOrientation()->getRelativeToBase(relativePose);
      relativePose.a[12] = 0.0;
      relativePose.a[13] = 0.0;
      relativePose.a[14] = 0.0;
    }
  else
    relativePose.unity();

  if (getBase() != NULL)
    {
      getBase()->getRelativeToBase(tmpMatrix);
      
      relativePose.a[12] = tmpMatrix.a[12];
      relativePose.a[13] = tmpMatrix.a[13];
      relativePose.a[14] = tmpMatrix.a[14];
    }

  relativePose.mul(relativePose, pose);
  
  mat = relativePose;
}

inline void CKinematicChainContainer::update()
{
    for (int i=0; i<length; i++)
        chain[i].update();
}

inline void  CKinematicChain::update()
{ 
  //for (int i=0; i<length; i++)
  //  frames[i]->invalidate();
    
  frames[0]->invalidate();
  for (int i=0; i<length; i++)
    {
      tmpMatrix.setDh(dhParameters[i]);
      frames[i]->setPoseNoInvalidation(tmpMatrix);
    }
}


inline void CKinematicChain::invalidate()
{ 
  frames[0]->invalidate();
  //for (int i=0; i<length; i++)
  //     frames[i]->invalidate();
}


inline void CFrameContainer::invalidate(bool time)
{
  for (unsigned int i=0; i<frames.size(); i++)
    {
      if (time)
	{
	  frames[i]->unlock();
	}
      frames[i]->invalidate();
    }
}


inline void CFrameCombination::invalidate()
{
  //_isUpdated = false;
    _isValid = false;

    //printf("Frame: Invalidating %s\n", name.c_str());
    /*
  if (_isLocked)// || time >= 0)
    return;

    _isValid = false;  */  
    for (unsigned int i=0; i<parents.size(); i++)
      parents[i]->invalidate();
}


inline CFrame* CFrame::getBase() 
{ 
  return base;
}


inline void CFrame::getRelativeToBase(CMatrix &mat)
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


inline CMatrix CFrame::getRelativeToBase()
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
};

#endif
