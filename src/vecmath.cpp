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


#include <stdio.h>
#include <math.h>
#include <iostream>


#include "frame.h"
#include "vecmath.h"

#define SWAP(x,y) tmp = a[x];a[x]=a[y];a[y] = tmp;
#define ROUND(x) rnd(x*10)/10.0)

using namespace std;

namespace robotLibPbD {


boost::mt19937 pRngMt19937;
boost::uniform_01<double> pRngDist;
boost::variate_generator<boost::mt19937&, boost::uniform_01<double> > pRngUniform01(pRngMt19937, pRngDist);

//static PRECISION tmpCMatrixMul[16];

// sets vector to (x y z 1.0)
void CVec::set(PRECISION x, PRECISION y, PRECISION z)
{
     this->x = x;
	 this->y = y;
	 this->z = z;
	 this->w = 1.0;
}
	
// prints vector value
void CVec::print() const
{
     printf("%f %f %f", x, y, z);
}
    
std::string CVec::toString()
{
    char str[255];
    sprintf(str, "%f %f %f", x, y, z);
    return std::string(str);
}

// inits matrix with 1-Matrix
CMatrix::CMatrix()
{
     set( 1.0, 0.0, 0.0, 0.0, 
              0.0, 1.0, 0.0, 0.0, 
              0.0, 0.0, 1.0, 0.0, 
              0.0, 0.0, 0.0, 1.0);
}

void CMatrix::randomOrientation(double max)
{
  CVec tmp;
  tmp.x = getGaussianProb(0.0, 1.0);
  tmp.y = getGaussianProb(0.0, 1.0);
  tmp.z = getGaussianProb(0.0, 1.0);
  tmp.normalize();

  CMathLib::getMatrixFromRotation(*this, tmp, getUniform() * max);
}

void CMatrix::unity()
{
     set( 1.0, 0.0, 0.0, 0.0, 
              0.0, 1.0, 0.0, 0.0, 
              0.0, 0.0, 1.0, 0.0, 
              0.0, 0.0, 0.0, 1.0);
}

void CMatrix::printText()
{
    double x,y,z,a,b,g;
    x = this->a[12];
    y = this->a[13];
    z = this->a[14];

    CVec tmp1, tmp2;
    CMathLib::getOrientation(*this, tmp1, tmp2);

    a = tmp1.x * 180.0/M_PI;
    b = tmp1.y * 180.0/M_PI;
    g = tmp1.z * 180.0/M_PI;

    printf("%g %g %g %g %g %g\n", x, y, z, a, b, g);
}

// transposes matrix	
void CMatrix::transpose()
{
     PRECISION tmp;
     SWAP(1,4);
     SWAP(2,8);
     SWAP(6,9);
     SWAP(3,12);
     SWAP(7,13);
     SWAP(11,14);
}



// sets matrix to supplied value	
CMatrix::CMatrix( PRECISION a0, PRECISION a4, PRECISION a8,  PRECISION a12,
                  PRECISION a1, PRECISION a5, PRECISION a9,  PRECISION a13,
                  PRECISION a2, PRECISION a6, PRECISION a10, PRECISION a14,
                  PRECISION a3, PRECISION a7, PRECISION a11, PRECISION a15)
{
     a[0] = a0;  a[4] = a4;  a[8]  = a8;   a[12] = a12;
     a[1] = a1;  a[5] = a5;  a[9]  = a9;   a[13] = a13;
     a[2] = a2;  a[6] = a6;  a[10] = a10;  a[14] = a14;
     a[3] = a3;  a[7] = a7;  a[11] = a11;  a[15] = a15;
}

// sets matrix to supplied value	
void CMatrix::set( PRECISION a0, PRECISION a4, PRECISION a8,  PRECISION a12,
                   PRECISION a1, PRECISION a5, PRECISION a9,  PRECISION a13,
                   PRECISION a2, PRECISION a6, PRECISION a10, PRECISION a14,
                   PRECISION a3, PRECISION a7, PRECISION a11 , PRECISION a15)
{
     a[0] = a0;  a[4] = a4;  a[8]  = a8;   a[12] = a12;
     a[1] = a1;  a[5] = a5;  a[9]  = a9;   a[13] = a13;
     a[2] = a2;  a[6] = a6;  a[10] = a10;  a[14] = a14;
     a[3] = a3;  a[7] = a7;  a[11] = a11;  a[15] = a15;
}

// creates denavit hartenberg transformation matrix	based on CDh struct
void CMatrix::setDh(const CDh &dh)
{
  if (dh.useAxis)
    {
      if (dh.rotationalDof)
	{
	  CMathLib::getMatrixFromRotation(*this, dh.axis, dh.rot_z + dh.angle);
	  a[12] = 0.0;
	  a[13] = 0.0;
	  a[14] = 0.0;
	} else
	{
	  a[0] = 1.0;
	  a[1] = 0.0;
	  a[2] = 0.0;
	  a[3] = 0.0;
	  a[4] = 0.0;
	  a[5] = 1.0;
	  a[6] = 0.0;
	  a[7] = 0.0;
	  a[8] = 0.0;
	  a[9] = 0.0;
	  a[10] = 1.0;
	  a[11] = 0.0;
	  a[12] = dh.axis.x * (dh.rot_z + dh.angle);
	  a[13] = dh.axis.y * (dh.rot_z + dh.angle);
	  a[14] = dh.axis.z * (dh.rot_z + dh.angle);
	  a[15] = 1.0;
	}
    } else if (dh.rotationalDof)
    {
      a[0] = cos(dh.rot_z + dh.angle);  a[4] = -sin(dh.rot_z + dh.angle)*cos(dh.rot_x);  a[8]  = sin(dh.rot_x)*sin(dh.rot_z + dh.angle);   a[12] = dh.trans_x*cos(dh.rot_z + dh.angle);
      a[1] = sin(dh.rot_z + dh.angle);  a[5] = cos(dh.rot_x)*cos(dh.rot_z + dh.angle);   a[9]  =  -cos(dh.rot_z + dh.angle)*sin(dh.rot_x); a[13] = dh.trans_x*sin(dh.rot_z + dh.angle);
      a[2] = 0.0;  		        a[6] = sin(dh.rot_x);  			         a[10] = cos(dh.rot_x);  			   a[14] = dh.trans_z;
      a[3] = 0.0;  		        a[7] = 0.0;  			                 a[11] = 0.0;  	                     	           a[15] = 1.0;
    } else
    {
      a[0] = cos(dh.rot_z);  a[4] = -sin(dh.rot_z)*cos(dh.rot_x);  a[8]  = sin(dh.rot_x) * sin(dh.rot_z);   a[12] = dh.trans_x*cos(dh.rot_z);
      a[1] = sin(dh.rot_z);  a[5] = cos(dh.rot_x)*cos(dh.rot_z);   a[9]  = -cos(dh.rot_z) * sin(dh.rot_x); a[13] = dh.trans_x*sin(dh.rot_z);
      a[2] = 0.0;  	     a[6] = sin(dh.rot_x);  		   a[10] = cos(dh.rot_x);  		  a[14] = dh.trans_z + dh.angle;
      a[3] = 0.0;  	     a[7] = 0.0;  			   a[11] = 0.0;  	                  a[15] = 1.0;
    }
}

// trace of rotational 3x3 part of matrix	
PRECISION CMatrix::trace()
{
     return a[0] + a[5] + a[10];
}
       
// print matrix contents	
void CMatrix::print(bool round) const
{
   //char str[1024];
   
   #define RND(x) (round ? rnd(x*10)/10.0 : x)
   printf(      
                "%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n", 
                RND(a[0]), RND(a[4]), RND(a[8]), RND(a[12]), 
                RND(a[1]), RND(a[5]), RND(a[9]), RND(a[13]), 
                RND(a[2]), RND(a[6]), RND(a[10]), RND(a[14]), 
                RND(a[3]), RND(a[7]), RND(a[11]), RND(a[15]));
}


std::string CMatrix::toString(bool round)
{
    char str[1024];
#define RND(x) (round ? rnd(x*10)/10.0 : x)
    sprintf(     str, 
                 "%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n", 
                 RND(a[0]), RND(a[4]), RND(a[8]), RND(a[12]), 
                 RND(a[1]), RND(a[5]), RND(a[9]), RND(a[13]), 
                 RND(a[2]), RND(a[6]), RND(a[10]), RND(a[14]), 
                 RND(a[3]), RND(a[7]), RND(a[11]), RND(a[15]));
    return std::string(str);
}

// scalar multiplication    
CMatrix CMatrix::operator * ( PRECISION f) const  
{	
    return CMatrix( a[0]*f, a[4]*f, a[8]*f, a[12]*f,
                    a[1]*f, a[5]*f, a[9]*f, a[13]*f,
                    a[2]*f, a[6]*f, a[10]*f, a[14]*f,
                    0.0, 0.0, 0.0, 1.0);	
}

// matrix add operator	
CMatrix CMatrix::operator + (const  CMatrix &b) const 
{	
    return CMatrix( a[0]+b.a[0], a[4]+b.a[4], a[8]+b.a[8], a[12]+b.a[12],
                    a[1]+b.a[1], a[5]+b.a[5], a[9]+b.a[9], a[13]+b.a[13],
                    a[2]+b.a[2], a[6]+b.a[6], a[10]+b.a[10], a[14]+b.a[14],
                    0.0, 0.0, 0.0, 1.0);	
}

// matrix sub operator	
CMatrix CMatrix::operator - (const  CMatrix &b) const 
{	
    return CMatrix( a[0]-b.a[0], a[4]-b.a[4], a[8]-b.a[8], a[12]-b.a[12],
                    a[1]-b.a[1], a[5]-b.a[5], a[9]-b.a[9], a[13]-b.a[13],
                    a[2]-b.a[2], a[6]-b.a[6], a[10]-b.a[10], a[14]-b.a[14],
                    0.0, 0.0, 0.0, 1.0);	
}

// matrix product operator	
CMatrix CMatrix::operator * (const  CMatrix &m) const 
{	
    CMatrix tmp;
    tmp.mul((*this), m);
    return tmp;	
}

// add a translation to homogenous matrix 	
CMatrix& CMatrix::operator += (const  CVec &v) 
{	
    a[12] += v.x;
    a[13] += v.y;
    a[14] += v.z;
    return *this;	
}

// calculate matrix * vector	
CVec CMatrix::operator * (const  CVec &v) const  
{	
    PRECISION x, y, z;
    x = a[0]*v.x + a[4]*v.y + a[8]*v.z + a[12]*v.w;
    y = a[1]*v.x + a[5]*v.y + a[9]*v.z + a[13]*v.w;
    z = a[2]*v.x + a[6]*v.y + a[10]*v.z + a[14]*v.w;
    return CVec(x, y, z);	
}


// array operator, read only	
const CVec& CMatrix::operator [] ( int i ) const 
{	
    return *( const CVec*) &a[4*i];	
}

CVec& CMatrix::operator [] ( int i ) 
{	
    return *(CVec*) &a[4*i];	
}
	
// assignment operator
CMatrix& CMatrix::operator = ( const CMatrix& v)
{
    memcpy(a,v.a, 16*sizeof(PRECISION));
    return *this;
}

// rounds a value	
int rnd(PRECISION value)
{
     return (int) (value < 0.0 ? ceil(value - 0.5) : floor(value +0.5));
}

void CMatrix::matrixToArray(std::vector<double> &values)
{
  values.resize(12);
  values[0] = a[12];
  values[1] = a[13];
  values[2] = a[14];

  
  values[3]  = a[0];
  values[4]  = a[1];
  values[5]  = a[2];
  values[6]  = a[4];
  values[7]  = a[5];
  values[8]  = a[6];
  values[9]  = a[8];
  values[10] = a[9];
  values[11] = a[10];
}

void CMatrix::arrayToMatrix(std::vector<double> &values)
{
    a[12] = values[0];
    a[13] = values[1];
    a[14] = values[2];
    a[15] = 1.0;
               
    a[0] = values[3];
    a[1] = values[4];
    a[2] = values[5];
    a[3] = 0.0;
               
    a[4] = values[6];
    a[5] = values[7];
    a[6] = values[8];
    a[7] = 0.0;
               
    a[8] = values[9];
    a[9] = values[10];
    a[10] = values[11];
    a[11] = 0.0;
}

PRECISION CMatrix::length() const
{
    double dist = sqrt(a[12]*a[12]+a[13]*a[13]+a[14]*a[14]);
    CVec axis;
    double angle;
    CMathLib::getRotationFromMatrix(*this, axis, angle);
  
    if (2.0*M_PI - angle < angle)
      angle = 2.0*M_PI-angle;

    return dist + fabsf(angle) * 180.0/M_PI;
}



// calculates rotation matrix from quaternion
// http://www.flipcode.com/documents/matrfaq.html
void CMathLib::matrixFromQuaternion(CVec &quaternion, CMatrix &matrix)
{
    double xx,xy,xz,xw,yy,yz,yw,zz,zw;
    double X = quaternion.x;
    double Y = quaternion.y;
    double Z = quaternion.z;
    double W = quaternion.w;
    xx      = X * X;
    xy      = X * Y;
    xz      = X * Z;
    xw      = X * W;

    yy      = Y * Y;
    yz      = Y * Z;
    yw      = Y * W;

    zz      = Z * Z;
    zw      = Z * W;

    matrix.a[0]  = 1.0 - 2.0 * ( yy + zz );
    matrix.a[4]  =     2.0 * ( xy - zw );
    matrix.a[8]  =     2.0 * ( xz + yw );

    matrix.a[1]  =     2.0 * ( xy + zw );
    matrix.a[5]  = 1.0 - 2.0 * ( xx + zz );
    matrix.a[9]  =     2.0 * ( yz - xw );

    matrix.a[2]  =     2.0 * ( xz - yw );
    matrix.a[6]  =     2.0 * ( yz + xw );
    matrix.a[10] = 1.0 - 2.0 * ( xx + yy );

    matrix.a[3]  = matrix.a[7] = matrix.a[11] = matrix.a[12] = matrix.a[13] = matrix.a[14] = 0.0;
    matrix.a[15] = 1.0;
}

// calculates matrix from rotation axis and rotation angle
// http://www.flipcode.com/documents/matrfaq.html
void CMathLib::getMatrixFromRotation(CMatrix &mat, const CVec &axis, double angle)
{
    CVec tmp, quater;
    double sin_a = sin( angle / 2.0 );
    double cos_a = cos( angle / 2.0 );

    tmp.x = axis.x * sin_a;
    tmp.y = axis.y * sin_a;
    tmp.z = axis.z * sin_a;
    tmp.w = cos_a;

    // normalisieren
    double tmpf = 1.0/sqrt(tmp.x*tmp.x+
                tmp.y*tmp.y+
                tmp.z*tmp.z+
                tmp.w*tmp.w); 
    
    //tmpf = 1.0;
                
    quater.x = tmp.x * tmpf;
    quater.y = tmp.y * tmpf;
    quater.z = tmp.z * tmpf;
    quater.w = tmp.w * tmpf;   

    matrixFromQuaternion(quater, mat); 
}

void CMathLib::getQuaternionFromRotation(CVec &quater, CVec &axis, double angle)
{
    CVec tmp;
    double sin_a = sin( angle / 2.0 );
    double cos_a = cos( angle / 2.0 );

    tmp.x = axis.x * sin_a;
    tmp.y = axis.y * sin_a;
    tmp.z = axis.z * sin_a;
    tmp.w = cos_a;

    // normalisieren
    double tmpf = 1.0/sqrt(tmp.x*tmp.x+
            tmp.y*tmp.y+
            tmp.z*tmp.z+
            tmp.w*tmp.w); 
    
    //tmpf = 1.0;
                
    quater.x = tmp.x * tmpf;
    quater.y = tmp.y * tmpf;
    quater.z = tmp.z * tmpf;
    quater.w = tmp.w * tmpf;   
}

// calculates rotation axis and angle from rotation matrix
// http://www.flipcode.com/documents/matrfaq.html
void CMathLib::getRotationFromMatrix(const CMatrix &mat, CVec &result, double &angle)
{
         CVec quat, tmp;
         double rotangle, tmpf, cos_a, sin_a;
         quaternionFromMatrix(mat, tmp); 
         
         // normalisieren
         tmpf = 1.0/sqrt(tmp.x*tmp.x+
                tmp.y*tmp.y+
                tmp.z*tmp.z+
                tmp.w*tmp.w); 
                
         quat = tmp * tmpf;
         quat.w = tmp.w * tmpf;      
      
         cos_a = quat.w;
         rotangle = acos( cos_a ) * 2;
         sin_a = sqrt( 1.0 - cos_a * cos_a );
         if ( fabs( sin_a ) < 0.0005 ) sin_a = 1;
         
         result.x = quat.x / sin_a;
         result.y = quat.y / sin_a;
         result.z = quat.z / sin_a;

         angle = rotangle;
         
	 /*
	 if (result.z < 0.0)
	   {
	     result = -result;
	     angle = -angle;
	   }
	 */
}

// calculates matrix[n x m] * vector [m]
void CMathLib::calcMatrixResult(double a[], double b[], int n, int m, double result[])
{
     int i,j;
     for (i=0;i<n;i++)
     {
         result[i] = 0.0;    
         for (j=0;j<m; j++)
             result[i] += a[i*n+j]*b[j];
     }
}

// calculates dot product a[n] * b[n]
double CMathLib::calcDotProduct(double a[], double b[], int n)
{
     int i;
     double tmp = a[0]*b[0];
     for (i=1;i<n;i++)
         tmp += a[i]*b[i];
         
     return tmp;
}



// calculates quaternion from rotation matrix
// http://www.flipcode.com/documents/matrfaq.html
void CMathLib::quaternionFromMatrix(const CMatrix &mat, CVec &quaternion)
{
  double T = 1 + mat.a[0] + mat.a[5] + mat.a[10];//mat.trace();
     double S,X,Y,Z,W;
     
     if (T > 0.00000001)
     {
      S = 2 * sqrt(T);
      X = (mat.a[6] - mat.a[9])/S;
      Y = (mat.a[8] - mat.a[2])/S;
      Z = (mat.a[1] - mat.a[4])/S;   
      W = S*0.25;
     } else
     if ( mat.a[0] > mat.a[5] && mat.a[0] > mat.a[10] )  {	// Column 0: 
        S  = sqrt( 1.0 + mat.a[0] - mat.a[5] - mat.a[10] ) * 2;
        X = 0.25 * S;
        Y = (mat.a[4] + mat.a[1] ) / S;
        Z = (mat.a[2] + mat.a[8] ) / S;
        W = (mat.a[6] - mat.a[9] ) / S;
    } else if ( mat.a[5] > mat.a[10] ) {			// Column 1: 
        S  = sqrt( 1.0 + mat.a[5] - mat.a[0] - mat.a[10] ) * 2;
        X = (mat.a[4] + mat.a[1] ) / S;
        Y = 0.25 * S;
        Z = (mat.a[9] + mat.a[6] ) / S;
        W = (mat.a[8] - mat.a[2] ) / S;
    } else {						// Column 2:
        S  = sqrt( 1.0 + mat.a[10] - mat.a[0] - mat.a[5] ) * 2;
        X = (mat.a[2] + mat.a[8] ) / S;
        Y = (mat.a[9] + mat.a[6] ) / S;
        Z = 0.25 * S;
        W = (mat.a[1] - mat.a[4] ) / S;
    }
    
    quaternion.x = X;
    quaternion.y = Y;
    quaternion.z = Z;
    quaternion.w = W;
}

// transposes matrix a[n x n]
void CMathLib::transposeMatrix(double *a, double *result, int n)
{
     int i,j;
     double tmp;
     for (i=0;i<n;i++)
         for (j=0;j<i;j++)
         {
             tmp = a[i*n+j];
             result[i*n+j] = a[j*n+i];
             result[j*n+i] = tmp; 
         }
}

// multiplies two matrix a[n x m] * b [m x k]
void CMathLib::multiplyMatrices(double *a, double *b, int n, int m, int k, double *res)
{
     int i,j,l;
     for (i=0; i<n; i++)
         for (j=0; j<k; j++)
         {
             res[i*n + j] = 0.0;
             for (l=0; l<m; l++)
             res[i*n + j] += a[i*n + l] * b[l*m + j];
         }
}

// calculates euler angles representing rotation matrix
/*void CMathLib::getOrientation(CMatrix &mat, CVec &first, CVec &second)
{
     //g = roll, a = pitch, b = yaw
     double a, b, g;
     
     
     b = atan2(-mat.a[4], sqrt(mat.a[5]*mat.a[5]+mat.a[6]*mat.a[6]));

     if (fabs(b - M_PI/2.0) < 0.0001)
     {
        g = 0;
        a = atan2(mat.a[9], mat.a[10]);        
     } else if (fabs(b + M_PI/2.0) < 0.0001)
     {
        g = 0;
        a = atan2(-mat.a[9], mat.a[10]);        
     } else
     {
        a = atan2(mat.a[8]/cos(b), mat.a[0]/cos(b));
        g = atan2(mat.a[6]/cos(b), mat.a[5]/cos(b));   
     }
     
     first.set(g, a, b);
     second = first;
}*/

void CMathLib::getEulerZXZ(CMatrix &mat, CVec &first)
{
  if (mat.a[10] < 1.0)
    {
      if (mat.a[10]>-1.0)
	{
	  first.x = atan2(mat.a[8], -mat.a[9]);
	  first.y = acos(mat.a[10]);
	  first.z = atan2(mat.a[2], mat.a[6]);
	} else
	{
	  first.x = -atan2(-mat.a[4], mat.a[0]);
	  first.y = M_PI;
	  first.z = 0;
	}
    } else
    {
      first.x = 0;
      first.y = atan2(-mat.a[4], mat.a[0]);
      first.z = 0;
    }
}

// calculates euler angles representing rotation matrix
void CMathLib::getOrientation(CMatrix &mat, CVec &first, CVec &second, bool old)
{
    double a, b, g;
     
    b = atan2(-mat.a[2], sqrt(mat.a[0]*mat.a[0]+mat.a[1]*mat.a[1]));

    if (fabsf(b - M_PI/2.0) < 0.000001)
    {
        a = 0;
        g = atan2(mat.a[4], mat.a[5]);        
    } else if (fabsf(b + M_PI/2.0) < 0.000001)
    {
        a = 0;
        g = -atan2(mat.a[4], mat.a[5]);        
    } else
    {
        a = atan2(mat.a[1]/cos(b), mat.a[0]/cos(b));
        g = atan2(mat.a[6]/cos(b), mat.a[10]/cos(b));   
    }
     
    if (old)
      first.set(a, b, g);
    else
      first.set(g, b, a);

    second = first;
}


// calculates rotation matrix representing euler angles
/*void CMathLib::getRotation(CMatrix &mat, double aX, double aY, double aZ)
{
     double a = aX;
     double b = aZ; 
     double g = aY;                                       
     mat.a[0] = cos(b)*cos(g);                          
     mat.a[1] = cos(a)*sin(b)*cos(g) + sin(a)*sin(g);        
     mat.a[2] = sin(a)*sin(b)*cos(g)-cos(a)*sin(g);      
     mat.a[4] = -sin(b);                                
     mat.a[5] = cos(a)*cos(b);                          
     mat.a[6] = sin(a)*cos(b);                           
    
     mat.a[8] = cos(b)*sin(g);                           
     mat.a[9] = cos(a)*sin(b)*sin(g)-sin(a)*cos(g);      
     mat.a[10] = sin(a)*sin(b)*sin(g)+cos(a)*cos(g); 
         
}
*/

// calculates rotation matrix representing euler angles
void CMathLib::getRotation(CMatrix &mat, double aX, double aY, double aZ, bool old)
{
    double g = aX;
    double b = aY; 
    double a = aZ; 

    if (old)
    {
      a = aX;
      b = aY;
      g = aZ;
    }                                
    if (fabsf(b - M_PI/2.0) < 0.0001)
    {
        mat.a[0] = 0.0;                          
        mat.a[1] = 0.0;        
        mat.a[2] = -1.0;      
        mat.a[4] = sin(g - a);                                
        mat.a[5] = cos(g - a);                          
        mat.a[6] = 0.0;                           
    
        mat.a[8] = cos(g - a);                           
        mat.a[9] = -sin(g - a);      
        mat.a[10] = 0.0;   
    } else
        if (fabsf(b + M_PI/2.0) < 0.0001)
        {
            mat.a[0] = 0.0;                          
            mat.a[1] = 0.0;        
            mat.a[2] = 1.0;      
            mat.a[4] = -sin(g + a);                                
            mat.a[5] = cos(g + a);                          
            mat.a[6] = 0.0;                           
    
            mat.a[8] = -cos(g + a);                           
            mat.a[9] = -sin(g + a);      
            mat.a[10] = 0.0;  
        } else
        {    
            mat.a[0] = cos(b)*cos(a);                          
            mat.a[1] = sin(a)*cos(b);        
            mat.a[2] = -sin(b);      
            mat.a[4] = cos(a)*sin(b)*sin(g) - sin(a)*cos(g);                                
            mat.a[5] = sin(a)*sin(b)*sin(g) + cos(a)*cos(g);                          
            mat.a[6] = cos(b)*sin(g);                           
    
            mat.a[8] = cos(a)*sin(b)*cos(g)+sin(a)*sin(g);                           
            mat.a[9] = sin(a)*sin(b)*cos(g)-cos(a)*sin(g);      
            mat.a[10] = cos(b)*cos(g); 
        }

        /* set to zero
        mat.a[12] = 0.0; mat.a[13] = 0.0; mat.a[14] = 0.0;
        mat.a[3] = 0.0;mat.a[7] = 0.0;mat.a[11] = 0.0;mat.a[15] = 1.0;
        */
}

// convenience function
void CMathLib::getRotation(CMatrix &mat, CVec &vec, bool old)
{
    getRotation(mat, vec.x, vec.y, vec.z, old);
}

// calculates inverse kinematics of a two bone leg
void CMathLib::calcAngles(double leg1, double leg2, double x, double y, double &first, double &second)
{
	double lambda;
	
    // normalize x and y
    const double factor = 0.9999;
    if ((x*x +y*y) >= factor*(leg1+leg2)*(leg1+leg2))
	{
       x = factor*x*sqrt((leg1+leg2)*(leg1+leg2) /(x*x +y*y));
       y = factor*y*sqrt((leg1+leg2)*(leg1+leg2) /(x*x +y*y));
       first = 0.0;
       second = atan2(-x,-y);
       return;
    }
    
    lambda = leg1;
    leg1 = leg2;
    leg2 = lambda;
    double cosb, sinb, sinab, cosab, cosa, sina;
    lambda = sqrt(x*x+y*y);
    sina = y / lambda;
    cosa = -x / lambda;
    cosb = (leg1*leg1 + lambda*lambda - leg2*leg2) / (2*leg1*lambda);
    sinb = sqrt(1-cosb*cosb);
    sinab = sina*cosb + cosa*sinb;
    cosab = cosa*cosb-sina*sinb;
    second = atan2(sinab, cosab)+0.5*M_PI;
    
    cosa = (leg1*leg1 + leg2*leg2 - lambda*lambda)/(2*leg1*leg2);
    sina = sqrt(1-cosa*cosa);
    first = 0.5*M_PI-atan2(-cosa, -sina);
    
    first = atan2(sina, cosa) - M_PI;
}

double vectorlength(std::vector<double> &v)
{
  double sum = 0.0;
  for (unsigned int i=0; i<v.size(); i++)
    sum += v[i]*v[i];
  return sqrt(sum);
}

char rosenbrock_version[] = "rosenbrock 0.99";

#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))

void simulatedannealing(int n, double *x, double *bl, double *bu,
                double bigbnd, int maxiter, double eps, int verbose,
                void obj(int,double *,double *,void *), void *extraparams)
{
    double *best = (double*)calloc(n,sizeof(double));
    double besty, t0, beta, y, delta;;
    
    beta = 0.9999;
    t0 = 3000.0;
    
    memcpy(best,x,n*sizeof(double));
    
    (*obj)(n,best,&besty,extraparams);
    
    unsigned int steps, unchanged;
    
    steps = 0;
    unchanged = 0;
    while ((int)steps < maxiter && unchanged < 100)
    {
        for (int i=0; i<n;i++)
        {
            double rnd = getUniform();
            
            double lo, hi;
            
            if ((x[i] - eps) < bl[i])
            {
                lo = bl[i];
                hi = lo + 2*eps;
            } else
                if ((x[i] + eps) > bu[i])
                {
                    hi = bu[i];
                    lo = hi -2*eps;
                    
                } else
                {
                    lo = x[i] - eps;
                    hi = x[i] + eps;
                }   
            
            x[i] = lo + (hi -lo) * rnd;
        }
        
        (*obj)(n,x,&y,extraparams);
        //if (y > 1000)
         //   continue;
        
        delta = y - besty;
        
        if (delta <= 0 || getUniform() <= exp(-delta/(t0 * pow(beta, steps))))
        {
            memcpy(best,x,n*sizeof(double));
            besty = y;
            
            unchanged = 0;
        } else unchanged++;
        
        steps++;
    }
    
    printf("steps: %d best: %g\n", steps, besty);
    
    memcpy(x,best,n*sizeof(double));
}

void rosenbrock(int n, double *x, double *bl, double *bu,
                double bigbnd, int maxiter, double eps, int verbose,
                void obj(int,double *,double *,void *), void *extraparams)
{
    double **xi=(double**)calloc(n,sizeof(double*)),
    *temp1=(double*)calloc(n*n,sizeof(double)),
    **A=(double**)calloc(n,sizeof(double*)),
    *temp2=(double*)calloc(n*n,sizeof(double)),
    *d=(double*)calloc(n,sizeof(double)),
    *lambda=(double*)calloc(n,sizeof(double)),
    *xk=(double*)calloc(n,sizeof(double)),
    *xcurrent=(double*)calloc(n,sizeof(double)),
    *t=(double*)calloc(n,sizeof(double)),
    alpha=2,
    beta=0.5,
    yfirst,yfirstfirst,ybest,ycurrent,mini,div;
    int i,k,j,restart,numfeval=0;

    memset(temp1,0,n*n*sizeof(double));
    for(i=0; i<n; i++)
      { 
        temp1[i]=1; 
	xi[i]=temp1; 
	temp1+=n;
        A[i]=temp2; 
	temp2+=n;
      };
    // memcpy(destination,source,nbre_of_byte)
    memcpy(xk,x,n*sizeof(double));
    for (i=0; i<n; i++) 
      d[i]=.1;
    memset(lambda,0,n*sizeof(double));
    (*obj)(n,x,&yfirstfirst,extraparams); 
    numfeval++; 

    do
    {
        ybest=yfirstfirst; 
        do
        {
            yfirst=ybest;
            for (i=0; i<n; i++)
            {
                for (j=0; j<n; j++) 
		  {
		    xcurrent[j]=xk[j]+d[i]*xi[i][j];
		    if (xcurrent[j] < bl[j])
		      xcurrent[j] = bl[j]; 
		    else if (xcurrent[j] > bu[j])
		      xcurrent[j] = bu[j]; 
		  }
                (*obj)(n,xcurrent,&ycurrent,extraparams); 
		numfeval++;

		if (ycurrent<ybest) 
                { 
                    lambda[i]+=d[i];        // success
                    d[i]*=alpha;
                    ybest=ycurrent;
                    memcpy(xk,xcurrent,n*sizeof(double));
                } else
                {
                    d[i]*=-beta;             // failure
                }

                if (numfeval > maxiter)
		  {
		    memcpy(x,xk,n*sizeof(double));
		    goto  LABEL_ROSENBROCK_DONE;
		  }
            }
        } while (ybest<yfirst);

        mini=bigbnd;
        for (i=0; i<n; i++) 
	  mini=MIN(mini,fabs(d[i]));
        restart=mini>eps;

	memcpy(x,xk,n*sizeof(double));

        if (ybest<yfirstfirst)
        {
            mini=bigbnd;
            for (i=0; i<n; i++) 
	      mini=MIN(mini,fabs(xk[i]-x[i]));
            restart=restart||(mini>eps);

            if (restart)
            {
                // nous avons:
                // xk[j]-x[j]=(somme sur i de) lambda[i]*xi[i][j];

                for (i=0; i<n; i++) 
		  A[n-1][i]=lambda[n-1]*xi[n-1][i];

                for (k=n-2; k>=0; k--)
                    for (i=0; i<n; i++) 
		      A[k][i]=A[k+1][i]+lambda[k]*xi[k][i];

                t[n-1]=lambda[n-1]*lambda[n-1];
                for (i=n-2; i>=0; i--) 
		  t[i]=t[i+1]+lambda[i]*lambda[i];
                for (i=n-1; i>0; i--)
                {
                    div=sqrt(t[i-1]*t[i]);
                    if (div!=0)
                        for (j=0; j<n; j++)
                            xi[i][j]=(lambda[i-1]*A[i][j]-xi[i-1][j]*t[i])/div;
                }
                div=sqrt(t[0]);
                for (i=0; i<n; i++) 
		  xi[0][i]=A[0][i]/div;

                memcpy(x,xk,n*sizeof(double));
                memset(lambda,0,n*sizeof(double));
                for (i=0; i<n; i++) 
		  d[i]=.1;
                yfirstfirst=ybest;
            }
        }

    } while ((restart)&&(numfeval<maxiter));

 LABEL_ROSENBROCK_DONE:
    // the maximum number of evaluation is approximative
    // because in 1 iteration there is n function evaluations.
    if (verbose)
    {
        printf("ROSENBROCK method for local optimization (minimization)\n"
                "number of evaluation of the objective function= %i\n",numfeval);
    }
    free(xi[0]);
    free(A[0]);
    free(d);
    free(lambda);
    free(xk);
    free(xcurrent);
    free(t);
}

void calculateTransformationFromPlane(CVec &plane, CMatrix &transformation)
{
    CVec z;
    z.x = plane.x; // plane is 4d!
    z.y = plane.y;
    z.z = plane.z;
    
    CVec y;

    if (plane.x != 0.0)
    {
        y.x = (plane.y + plane.z) / plane.x;
        y.y = -1.0;
        y.z = -1.0;
    } else
    if (plane.y != 0.0)
    {
        y.y = (plane.x + plane.z) / plane.y;
        y.x = -1.0;
        y.z = -1.0;
    } else
    if (plane.z != 0.0)
    {
        y.z = (plane.y + plane.x) / plane.z;
        y.y = -1.0;
        y.x = -1.0;
    } else printf("error\n");
    
    CVec x = y ^ z;
    
    x.normalize();
    y.normalize();
    z.normalize();

    transformation.a[0] = x.x;
    transformation.a[1] = x.y;
    transformation.a[2] = x.z;
    
    transformation.a[4] = y.x;
    transformation.a[5] = y.y;
    transformation.a[6] = y.z;
    
    transformation.a[8] = z.x;
    transformation.a[9] = z.y;
    transformation.a[10] = z.z;
}


void setUniformSeed(unsigned int seed)
{ 
  pRngUniform01.engine().seed(seed);
  pRngUniform01.distribution().reset();
  //pRngMt19937.seed(seed);
}

double getSigmoid(double value, double offset, double factor)
{
       return 1.0/(1.0+exp(-factor*(value - offset)));
}

double getGaussian(double value, double mean, double std)
{
       // 0.063494 = 1.0 * sqrt(2 * M_PI)
       
       return 0.063494 / std * exp(-0.5 * (value - mean) * (value - mean) / (std * std) );
}

double getGaussianWithoutNormalization(double value, double mean, double std)
{
       return exp(-0.5 * (value - mean) * (value - mean) / (std * std) );
}



double getLogisticProb(double alpha, double beta)
{
  double u = getUniform();
    
    if (u==0.0)
       return 1E30;
       
    if (u==1.0)
       return -1E30;
    
    return alpha - beta * log(u/(1.0-u));
}



void convertMatrix(double (*R)[3], double *T, CMatrix &to)
{
  to.a[12] = T[0];
  to.a[13] = T[1];
  to.a[14] = T[2];

  to.a[0] = R[0][0];
  to.a[4] = R[0][1];
  to.a[8] = R[0][2];

  to.a[1] = R[1][0];
  to.a[5] = R[1][1];
  to.a[9] = R[1][2];

  to.a[2] = R[2][0];
  to.a[6] = R[2][1];
  to.a[10] = R[2][2];
}

void convertMatrix(CMatrix &from, double (*R)[3], double *T)
{
  R[0][0] = from.a[0];
  R[0][1] = from.a[4];
  R[0][2] = from.a[8];

  R[1][0] = from.a[1];
  R[1][1] = from.a[5];
  R[1][2] = from.a[9];

  R[2][0] = from.a[2];
  R[2][1] = from.a[6];
  R[2][2] = from.a[10];

  T[0] = from.a[12];
  T[1] = from.a[13];
  T[2] = from.a[14];
}


void getMeanFromVectors(std::vector<std::vector<double> > &values, std::vector<double> &mean)
{
  if (values.size() == 0)
    return;

  mean.clear();
  mean.resize(values[0].size(), 0.0);
  unsigned int counter = 0;
  for (unsigned int i=0; i<values.size(); i++)
    {
      for (unsigned int j=0; j<mean.size() && j<values[i].size(); j++)
	mean[j] += values[i][j];

      counter++;
    }

  if (counter > 0)
    for (unsigned int j=0; j<mean.size(); j++)
      mean[j] /= (double) counter;
}



void orientation2scaledAxis(CMatrix &matrix, CVec &axis)
{
  double angle;
  CMathLib::getRotationFromMatrix(matrix, axis, angle);
  axis *= angle;
}

void orientation2scaledAxis(CMatrix &matrix, std::vector<double> &axis2)
{
  CVec axis;
  double angle;
  CMathLib::getRotationFromMatrix(matrix, axis, angle);
  axis2.resize(3);
  axis2[0] = axis.x * angle;
  axis2[1] = axis.y * angle;
  axis2[2] = axis.z * angle;
}

void scaledAxis2Orientation(CVec &axis2, CMatrix &matrix)
{
  CVec axis;
  double angle = axis2.length();
  if (angle == 0.0)
    axis = axis2;
  else
    axis = axis2 / angle;

  CMathLib::getMatrixFromRotation(matrix, axis, angle);
}

void scaledAxis2Orientation(std::vector<double> &axis2, CMatrix &matrix)
{
  CVec axis(axis2[0], axis2[1], axis2[2]);
  double angle = axis.length(); 
  axis.normalize();

  CMathLib::getMatrixFromRotation(matrix, axis, angle);
}


}
