/**

The rosenbrock code:

Copyright (c) Dr. Ir. Frank Vanden Berghen
For further details see: http://applied-mathematics.net/
email: frank@applied-mathematics.net


Anything else is licensed as follows:

Copyright (c) 2016, Aumann Florian, Heller Florian, Jäkel Rainer, Wittenbeck Valerij
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/



#ifndef __VECMATH

#define __VECMATH

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_01.hpp>
#include <boost/random/variate_generator.hpp>

#include <math.h>
#include <vector>
#include <string>
#include <string.h>
#include "config.h"
//#include "newmat/newmatap.h"

#define PRECISION double // todo: use template instead
#define PI360 6.283185307179586
#define PI90 1.570796326794897 
#define RAD2ANGLE(X) ((X)*57.295779513082322)
#define ANGLE2RAD(X) ((X)*1.745329251994e-02)
#define RAD2ANGLEMAP(X) (X > M_PI ? RAD2ANGLE(PI360 - X) : RAD2ANGLE(X))

namespace robotLibPbD {

class CDh;

/*! \brief Homogenous vector
*/
class CVec 
{	
	public:
	PRECISION x, y, z, w;
	
        CVec() { x = y = z = 0.0; w = 1.0;}
        CVec(PRECISION x, PRECISION y, PRECISION z)
        {
            set(x, y, z);
        };
        
	void set(PRECISION x, PRECISION y, PRECISION z);
        
        std::string toString();
	/*! \brief Prints vector as (x,y,z) to console
    */
	void print() const; 
	/*! \brief Scalar multiplication
    */
        CVec operator * ( PRECISION s) const
        {
            return CVec( x*s, y*s, z*s );
        }; 
        CVec operator / ( PRECISION s) const
        {
            return CVec( x/s, y/s, z/s );
        }; 
    /*! \brief Assigns values of vector \a v to vector
    */ 
        CVec& operator = ( const CVec& v)
        {
            x = v.x; y = v.y; z = v.z; w = 1.0f;
            return *this;
        };
	/*! \brief Array access (read)
    */
        PRECISION operator [] (unsigned int i) const
        {
            return (&x)[i];
        };
	/*! \brief Array access (write)
    */
        PRECISION& operator [] (unsigned int i)
        {
            return (&x)[i];
        };
	/*! \brief Vector addition
    */
        CVec operator + ( const CVec& v) const
        {
            return CVec( x + v.x, y + v.y, z + v.z );
        };
        
        CVec& operator += ( const CVec& v)
        {
            x += v.x;
            y += v.y;
            z += v.z;
    
            return *this;
        };
        CVec& operator -= ( const CVec& v)
        {
            x -= v.x;
            y -= v.y;
            z -= v.z;
    
            return *this;
        };
        CVec& operator *= ( PRECISION s)
        {
            x *= s;
            y *= s;
            z *= s;
    
            return *this;
        };
        CVec& operator /= ( PRECISION s)
        {
            x /= s;
            y /= s;
            z /= s;
    
            return *this;
        };
	/*! \brief Vector difference
    */
        CVec operator - ( const CVec& v) const
        {
            return CVec( x - v.x, y - v.y, z - v.z );
        };
     
	/*! \brief Vector negation
    */
        CVec operator - () const
        {
            return CVec( -x, -y, -z );
        };
	/*! \brief Dot product
    */
        PRECISION operator | (const CVec& v) const
        {
            return x*v.x + y*v.y + z*v.z;
        };
	/*! \brief Cross product
    */
        CVec operator ^ (const CVec& v) const
        {
            return CVec( y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x );
        }; 
	
        PRECISION length() const
        {
            return sqrt(x*x+y*y+z*z);
        };
        void normalize()
        {
            PRECISION len = sqrt(x*x+y*y+z*z);
     
            if (len > 0.001)
            {
                x /= len;   
                y /= len;
                z /= len;
            }
        };
};

/*! \brief Homogenous matrix
*/
class CMatrix
{
	public:
	PRECISION a[16];
	CMatrix();
	CMatrix(
							PRECISION a0, PRECISION a4, PRECISION a8,  PRECISION a12,
							PRECISION a1, PRECISION a5, PRECISION a9,  PRECISION a13,
							PRECISION a2, PRECISION a6, PRECISION a10, PRECISION a14,
							PRECISION a3 = 0.0f, PRECISION a7 = 0.0f, PRECISION a11 = 0.0f, PRECISION a15 = 1.0f);
	void set(
							PRECISION a0, PRECISION a4, PRECISION a8,  PRECISION a12,
							PRECISION a1, PRECISION a5, PRECISION a9,  PRECISION a13,
							PRECISION a2, PRECISION a6, PRECISION a10, PRECISION a14,
							PRECISION a3 = 0.0f, PRECISION a7 = 0.0f, PRECISION a11 = 0.0f, PRECISION a15 = 1.0f);
	/*! \brief Trace - sum of diagonal elements - of matrix
    */						
	PRECISION trace();
	
	/*! \brief Transposes the matrix
    */
	void transpose();
       
	void printText();
	
	/*! \brief Inverts the matrix (only for homogenous matrices)
    */
	void invert();	
	
	/*! \brief Assigns product of two matrices to matrix
    */
    void mul(const CMatrix &first, const CMatrix &second);

    void mulNoAlloc(const CMatrix &first, const CMatrix &second);
     
    std::string toString(bool round = false);
    
	/*! \brief Prints matrix to console
    */
	void print(bool round = false) const;
	
	/*! \brief Creates Denavit Hartenberg matrix
    */
	void setDh(const robotLibPbD::CDh &dh);
        void arrayToMatrix(std::vector<double> &values);
        void matrixToArray(std::vector<double> &values);

	void randomOrientation(double max = 2.0 * M_PI);
	/*! \brief Scalar multiplication
    */
	CMatrix operator * ( PRECISION f) const ;
	/*! \brief Matrix addition
    */
	CMatrix operator + ( const CMatrix &b) const ; 
	
	/*! \brief Matrix subtraction
    */
	CMatrix operator - ( const CMatrix &b) const ; 
	/*! \brief Matrix multiplication
    */
	CMatrix operator * ( const CMatrix &m) const ; 
	/*! \brief Overwrites matrix with sum of matrix and vector
    */
	CMatrix& operator += ( const CVec &v);
	/*! \brief Matrix vector product
    */
	CVec operator * ( const CVec &v) const ;
	
    /*! \brief Array access (read)
    */
	const CVec& operator [] ( int i ) const;

    /*! \brief Array access (read&write)
    */
	CVec& operator [] ( int i );
	
	/*! \brief Assigns values of matrix \a v to matrix
    */
	CMatrix& operator = ( const CMatrix& v);
        
        PRECISION length() const; 

	void unity();
};


/*! \brief Mathematical functions
*/
class CMathLib
{
    public:        
    /*! \brief Calculates dot product of two double vectors of the stated \a size
    */
    static double calcDotProduct(double firstVector[], double secondVector[], int size);
    /*! \brief Calculates vector-matrix product
    
    \param resultVector Has to be allocated
    */
    static void calcMatrixResult(double matrix[], double vector[], int rows, int columns, double resultVector[]);
    /*! \brief Transforms homogenous matrix into axis-angle representation
    */
    static void getRotationFromMatrix(const CMatrix &mat, CVec &axis, double &angle);
    static void getQuaternionFromRotation(CVec &quater, CVec &axis, double angle);
    /*! \brief Transposes a matrix
    
    Set \a resultMatrix to \a matrix if you want to transpose the matrix
    
    \param resultMatrix Has to be allocated
    */
    static void transposeMatrix(double *matrix, double *resultMatrix, int size);
    /*! \brief Multiplies two matrices
    
    \param resultMatrix Has to be allocated
    */
    static void multiplyMatrices(double *firstMatrix, double *secondMatrix, int firstRows, int firstColumns, int secondColumns, double *resultMatrix);
    /*! \brief Transforms a homogenous matrix into quaternion representation
    
    \param quaternion XYZ is the rotation axis part, W the angle part
    */
    static void quaternionFromMatrix(const CMatrix &mat, CVec &quaternion);
    /*! \brief Transforms a quaternion into homogenous matrix representation
    */
    static void matrixFromQuaternion(CVec &quaternion, CMatrix &matrix);
    /*! \brief Transforms axis-angle representation into homogenous matrix representation
    */
    static void getMatrixFromRotation(CMatrix &mat, const CVec &axis, double angle);
    /*! \brief Transforms homogenous matrix into Euler angle (YZX) representation (two solutions)
    */
    static void getEulerZXZ(CMatrix &mat, CVec &first);
    /*! \brief Transforms homogenous matrix into Euler angle (YZX) representation (two solutions)
    */
    static void getOrientation(CMatrix &mat, CVec &first, CVec &second, bool old = false);
    /*! \brief ransforms Euler angle (YZX) representation into homogenous matrix representation
    
    \see getRotation
    */
    static void getRotation(CMatrix &mat, CVec &vec, bool old = false);
    /*! \brief Transforms Euler angle (YZX) representation into homogenous matrix representation
    */
    static void getRotation(CMatrix &mat, double x, double y, double z, bool old = false);
    /*! \brief Calculates inverse kinematics of a standard two link leg
    
    \param leg1 Length of leg connected to base
    \param leg2 Length of leg connected to hand
    \param x Cartesian x position of hand
    \param y Cartesian y position of hand
    \param first Angle between base and leg1
    \param second Angle between leg1 and leg2 (elbow)
    */
    static void calcAngles(double leg1, double leg2, double x, double y, double &first, double &second);
};


double sign(double value);

double vectorlength(std::vector<double> &v);
unsigned long fac(unsigned int value);
int roundToInt(double value);

void getMeanFromVectors(std::vector<std::vector<double> > &values, std::vector<double> &mean);

double getSigmoid(double value, double offset, double factor);
double getGaussian(double value, double mean, double std);
double getGaussianWithoutNormalization(double value, double mean, double std);
 double getUniform();
 int getUniform(int min, int max);
 void setUniformSeed(unsigned int seed);
double getGaussianProb(double mean, double std);
double getLogisticProb(double alpha, double beta);
void calculateTransformationFromPlane(CVec &plane, CMatrix &transformation);
void convertMatrix(CMatrix &from, double (*R)[3], double *T);
void convertMatrix(double (*R)[3], double *T, CMatrix &to);

void orientation2scaledAxis(CMatrix &matrix, CVec &axis);
void orientation2scaledAxis(CMatrix &matrix, std::vector<double> &axis);
void scaledAxis2Orientation(CVec &axis, CMatrix &matrix);
void scaledAxis2Orientation(std::vector<double> &axis, CMatrix &matrix);

void rosenbrock(int n, double *x, double *bl, double *bu,
                double bigbnd, int maxiter, double eps, int verbose,
                void obj(int,double *,double *,void *), void *extraparams);
void simulatedannealing(int n, double *x, double *bl, double *bu,
                double bigbnd, int maxiter, double eps, int verbose,
                void obj(int,double *,double *,void *), void *extraparams);

/*! \brief Rounds a value
*/
int rnd(PRECISION value);


   
// calculates matrix product: this = first * second
//not threadsafe if global variable is used	
inline void CMatrix::mul(const CMatrix &first, const CMatrix &second)
{		  
  // optimize it!!
  PRECISION tmpCMatrixMul[16];
  tmpCMatrixMul[0]  = first.a[0] * second.a[0] + first.a[4] * second.a[1] + first.a[8] * second.a[2];
  tmpCMatrixMul[1]  = first.a[1] * second.a[0] + first.a[5] * second.a[1] + first.a[9] * second.a[2];
  tmpCMatrixMul[2]  = first.a[2] * second.a[0] + first.a[6] * second.a[1] + first.a[10] * second.a[2];
  tmpCMatrixMul[3]  = 0.0;
  tmpCMatrixMul[4]  = first.a[0] * second.a[4] + first.a[4] * second.a[5] + first.a[8] * second.a[6];
  tmpCMatrixMul[5]  = first.a[1] * second.a[4] + first.a[5] * second.a[5] + first.a[9] * second.a[6];
  tmpCMatrixMul[6]  = first.a[2] * second.a[4] + first.a[6] * second.a[5] + first.a[10] * second.a[6];
  tmpCMatrixMul[7]  = 0.0;
  tmpCMatrixMul[8]  = first.a[0] * second.a[8] + first.a[4] * second.a[9] + first.a[8] * second.a[10];
  tmpCMatrixMul[9]  = first.a[1] * second.a[8] + first.a[5] * second.a[9] + first.a[9] * second.a[10];
  tmpCMatrixMul[10] = first.a[2] * second.a[8] + first.a[6] * second.a[9] + first.a[10] * second.a[10];
  tmpCMatrixMul[11] = 0.0;
  tmpCMatrixMul[12]  = first.a[0] * second.a[12] + first.a[4] * second.a[13] + first.a[8] * second.a[14] + first.a[12] * second.a[15];
  tmpCMatrixMul[13]  = first.a[1] * second.a[12] + first.a[5] * second.a[13] + first.a[9] * second.a[14] + first.a[13] * second.a[15];
  tmpCMatrixMul[14]  = first.a[2] * second.a[12] + first.a[6] * second.a[13] + first.a[10] * second.a[14] + first.a[14] * second.a[15];
  tmpCMatrixMul[15]  = first.a[15] * second.a[15];
  memcpy(a, tmpCMatrixMul, 16*sizeof(PRECISION)); 
};


// calculates matrix product: this = first * second 
inline void CMatrix::mulNoAlloc(const CMatrix &first, const CMatrix &second)
{		   
  a[0]  = first.a[0] * second.a[0] + first.a[4] * second.a[1] + first.a[8] * second.a[2];
  a[1]  = first.a[1] * second.a[0] + first.a[5] * second.a[1] + first.a[9] * second.a[2];
  a[2]  = first.a[2] * second.a[0] + first.a[6] * second.a[1] + first.a[10] * second.a[2];
  a[3]  = 0.0;
  a[4]  = first.a[0] * second.a[4] + first.a[4] * second.a[5] + first.a[8] * second.a[6];
  a[5]  = first.a[1] * second.a[4] + first.a[5] * second.a[5] + first.a[9] * second.a[6];
  a[6]  = first.a[2] * second.a[4] + first.a[6] * second.a[5] + first.a[10] * second.a[6];
  a[7]  = 0.0;
  a[8]  = first.a[0] * second.a[8] + first.a[4] * second.a[9] + first.a[8] * second.a[10];
  a[9]  = first.a[1] * second.a[8] + first.a[5] * second.a[9] + first.a[9] * second.a[10];
  a[10] = first.a[2] * second.a[8] + first.a[6] * second.a[9] + first.a[10] * second.a[10];
  a[11] = 0.0;
  a[12]  = first.a[0] * second.a[12] + first.a[4] * second.a[13] + first.a[8] * second.a[14] + first.a[12] * second.a[15];
  a[13]  = first.a[1] * second.a[12] + first.a[5] * second.a[13] + first.a[9] * second.a[14] + first.a[13] * second.a[15];
  a[14]  = first.a[2] * second.a[12] + first.a[6] * second.a[13] + first.a[10] * second.a[14] + first.a[14] * second.a[15]; 
  a[15]  = first.a[15] * second.a[15];
};

extern boost::mt19937 pRngMt19937;
extern boost::uniform_01<double> pRngDist;
extern boost::variate_generator<boost::mt19937&, boost::uniform_01<double> > pRngUniform01;

inline double getUniform()
{
  return pRngUniform01();
  //return ((double) (rand() % (RAND_MAX-1))) / (double) RAND_MAX;
}

inline int getUniform(int min, int max)
{
  return min + (int) (((double)(max - min)) * pRngUniform01() + 0.5);
}

inline double getGaussianProb(double mean, double std)
{
    double u1, u2, v, x;
    
    do 
    {
      u1 = pRngUniform01();
      u2 = pRngUniform01();
        
        v = (2.0*u1 - 1.0)*(2.0*u1 - 1.0) + (2.0*u2 - 1.0)*(2.0*u2 - 1.0);
    } while (v >= 1.0);
    
    x = (2.0*u1 -1.0)*sqrt(-2.0*log(v) / v);
    
    return std*x + mean;
}


#define SWAPELEMENTS(x,y) tmp = a[x];a[x]=a[y];a[y] = tmp;

// invert homogenous transformation matrix    
inline void CMatrix::invert()
{ 
     /*CVec  vec, res;
     vec.set(-a[12], -a[13], -a[14]);
     SWAPELEMENTS(1,4);
     SWAPELEMENTS(2,8);
     SWAPELEMENTS(6,9);
     a[12] = a[13] = a[14] = 0.0;
     res = (*this) * vec;
     a[12] = res.x;
     a[13] = res.y;
     a[14] = res.z;  
     */

  PRECISION tmp;
  PRECISION x = -a[12];
  PRECISION y = -a[13];
  PRECISION z = -a[14];
  
  SWAPELEMENTS(1,4);
  SWAPELEMENTS(2,8);
  SWAPELEMENTS(6,9); 
  a[12] = a[0] * x + a[4] * y + a[8]  * z; 
  a[13] = a[1] * x + a[5] * y + a[9]  * z; 
  a[14] = a[2] * x + a[6] * y + a[10] * z; 
}
};

#endif

