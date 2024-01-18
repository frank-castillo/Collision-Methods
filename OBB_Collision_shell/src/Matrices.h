///////////////////////////////////////////////////////////////////////////////
// Matrice.h
// =========
// NxN Matrix Math classes
// Reza Nezami: It is not row-first matrix

///////////////////////////////////////////////////////////////////////////////
#pragma once
#ifndef MATH_MATRICES_H
#define MATH_MATRICES_H

#include <iostream>
#include <iomanip>
#include "vectors.h"
#include <assert.h>
///////////////////////////////////////////////////////////////////////////
// 2x2 matrix
///////////////////////////////////////////////////////////////////////////
class Matrix2
{
public:
    // constructors
    Matrix2();  // init with identity
	Matrix2(const float2D& row1, const float2D& row2) : rows{ row1, row2 } {}

	void setRow(size_t idx, const float2D row) { rows[idx] = row; }

	const float2D getRow(size_t id) const {
		assert(id < 2); return rows[id];
	}
	const float2D getCol(size_t id) const {
		assert(id < 2); 
		return{ rows[0][id], rows[1][id] };
	}
	float getDeterminant() {
		return rows[0].x * rows[1].y - rows[0].y * rows[1].x;
	}

	static Matrix2 identity() { return Matrix2(	{ 1.f, 0.f }, 
												{ 0.f, 1.f }); }
	static Matrix2 zero()	{	return Matrix2({ 0.f, 0.f },
												{ 0.f, 0.f }); }
	Matrix2    transpose() {return Matrix2(rows[1], rows[0]);}

    Matrix2    invert();

    // operators
    Matrix2     operator+(const Matrix2& rhs) const;    // add rhs
    Matrix2     operator-(const Matrix2& rhs) const;    // subtract rhs
    Matrix2&    operator+=(const Matrix2& rhs);         // add rhs and update this object
    Matrix2&    operator-=(const Matrix2& rhs);         // subtract rhs and update this object
    float2D     operator*(const float2D& rhs) const;    // multiplication: v' = M * v
    Matrix2     operator*(const Matrix2& rhs) const;    // multiplication: M3 = M1 * M2
    Matrix2&    operator*=(const Matrix2& rhs);         // multiplication: M1' = M1 * M2
    bool        operator==(const Matrix2& rhs) const;   // exact compare, no epsilon
    bool        operator!=(const Matrix2& rhs) const;   // exact compare, no epsilon
    float       operator()(int index1, int index2) const;            // subscript operator v[0], v[1]

    friend Matrix2 operator-(const Matrix2& m);                     // unary operator (-)
    friend Matrix2 operator*(float scalar, const Matrix2& m);       // pre-multiplication
    friend float2D operator*(const float2D& vec, const Matrix2& m); // pre-multiplication
    friend std::ostream& operator<<(std::ostream& os, const Matrix2& m);

protected:

private:
	float2D rows[2];

};

///////////////////////////////////////////////////////////////////////////
// inline functions for Matrix2
///////////////////////////////////////////////////////////////////////////
inline Matrix2::Matrix2()
{
	*this = identity();
}

inline Matrix2 Matrix2::operator+(const Matrix2& rhs) const
{
	return Matrix2(rows[0] + rhs.rows[0], rows[1] + rhs.rows[1]);
}

inline Matrix2 Matrix2::operator-(const Matrix2& rhs) const
{
	return Matrix2(rows[0] - rhs.rows[0], rows[1] - rhs.rows[1]);
}

inline Matrix2& Matrix2::operator+=(const Matrix2& rhs)
{
	rows[0] += rhs.rows[0];
	rows[1] += rhs.rows[1];
	return *this;
}

inline Matrix2& Matrix2::operator-=(const Matrix2& rhs)
{
	rows[0] -= rhs.rows[0];
	rows[1] -= rhs.rows[1];
	return *this;
}

inline float2D Matrix2::operator*(const float2D& rhs) const
{
	return float2D(Dot(rows[0],rhs), Dot(rows[1],rhs));
}

inline Matrix2 Matrix2::operator*(const Matrix2& rhs) const
{
	return Matrix2(	{ Dot(rows[0], rhs.getCol(0)) , Dot(rows[0], rhs.getCol(1)) },
					{ Dot(rows[1], rhs.getCol(0)) , Dot(rows[1], rhs.getCol(1)) });
}

inline Matrix2& Matrix2::operator*=(const Matrix2& rhs)
{
	*this = *this * rhs;
	return *this;
}

inline bool Matrix2::operator==(const Matrix2& rhs) const
{
	return (rows[0]==rhs.rows[0] && rows[1]==rhs.rows[1]);
}

inline bool Matrix2::operator!=(const Matrix2& rhs) const
{
	return !(*this == rhs);
}

//////////// friends //////////////////////
inline Matrix2 operator-(const Matrix2& rhs)
{
	return Matrix2(-rhs.rows[0], -rhs.rows[1]);
}

inline Matrix2 operator*(float s, const Matrix2& rhs)
{
	return Matrix2(s*rhs.rows[0], s*rhs.rows[1]);
}

inline float2D operator*(const float2D& v, const Matrix2& rhs)
{
	return{ Dot(v, rhs.getCol(0)), Dot(v, rhs.getCol(1)) };
}

inline std::ostream& operator<<(std::ostream& os, const Matrix2& m)
{
	os << std::fixed << std::setprecision(5);
	os << "[" << std::setw(10) << m.rows[0][0] << " " << std::setw(10) << m.rows[0][1] << "]\n"
		<< "[" << std::setw(10) << m.rows[1][0] << " " << std::setw(10) << m.rows[1][1] << "]\n";
	os << std::resetiosflags(std::ios_base::fixed | std::ios_base::floatfield);
	return os;
}
// END OF MATRIX2 INLINE //////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////
// 3x3 matrix
///////////////////////////////////////////////////////////////////////////

class Matrix3
{
public:
    // constructors
    Matrix3();  // init with identity
	Matrix3(const float3D& row1, const float3D& row2, const float3D& row3)
		: rows{ row1, row2, row3 } {}

    void  setRow(size_t idx, const float3D& row)
	{
		assert(idx < 3);
		rows[idx] = row;
	}
	void  setCol(size_t idx, float3D& v) {
		rows[0].set(idx, v[0]);
		rows[1].set(idx, v[1]);
		rows[2].set(idx, v[2]);
	}

	const float3D& getRow(size_t i) const {
		assert(i < 3); return rows[i];
	}
	const float3D getCol(size_t i) const
	{
		assert(i < 3);
		if (i == 0)
		{
			return{ rows[0].x, rows[1].x, rows[2].x };
		}
		else if (i == 1)
		{
			return{ rows[0].y, rows[1].y, rows[2].y };
		}
		else if (i == 2)
		{
			return{ rows[0].z, rows[1].z, rows[2].z };
		}
	}
	float       getDeterminant();

	static Matrix3  identity() { return Matrix3({ 1.f, 0.f, 0.f }, 
												{ 0.f, 1.f, 0.f },
												{ 0.f, 0.f, 1.f }); }
	static Matrix3  zero() {return Matrix3(		{ 0.f, 0.f, 0.f },
												{ 0.f, 0.f, 0.f },
												{ 0.f, 0.f, 0.f });
	}
	Matrix3    transpose() {return Matrix3(getCol(0), getCol(1), getCol(2)); }
    Matrix3    invert();

    // operators
    Matrix3     operator+(const Matrix3& rhs) const;    // add rhs
    Matrix3     operator-(const Matrix3& rhs) const;    // subtract rhs
    Matrix3&    operator+=(const Matrix3& rhs);         // add rhs and update this object
    Matrix3&    operator-=(const Matrix3& rhs);         // subtract rhs and update this object
	float3D     operator*(const float3D& rhs) const;    // multiplication: v' = M * v
    Matrix3     operator*(const Matrix3& rhs) const;    // multiplication: M3 = M1 * M2
    Matrix3&    operator*=(const Matrix3& rhs);         // multiplication: M1' = M1 * M2
    bool        operator==(const Matrix3& rhs) const;   // exact compare, no epsilon
    bool        operator!=(const Matrix3& rhs) const;   // exact compare, no epsilon
    float       operator[](int index) const;            // subscript operator v[0], v[1]
    float&      operator[](int index);                  // subscript operator v[0], v[1]

    friend Matrix3 operator-(const Matrix3& m);                     // unary operator (-)
    friend Matrix3 operator*(float scalar, const Matrix3& m);       // pre-multiplication
    friend float3D operator*(const float3D& vec, const Matrix3& m); // pre-multiplication
    friend std::ostream& operator<<(std::ostream& os, const Matrix3& m);

protected:

private:
	float3D rows[3];
};

///////////////////////////////////////////////////////////////////////////
// inline functions for Matrix3
///////////////////////////////////////////////////////////////////////////

inline Matrix3::Matrix3()
{
	*this = identity();
}
inline Matrix3 Matrix3::operator+(const Matrix3& rhs) const
{
	return Matrix3(rows[0] + rhs.rows[0], rows[1] + rhs.rows[1], rows[2] + rhs.rows[2]);
}

inline Matrix3 Matrix3::operator-(const Matrix3& rhs) const
{
	return Matrix3(rows[0] - rhs.rows[0], rows[1] - rhs.rows[1], rows[2] - rhs.rows[2]);
}

inline Matrix3& Matrix3::operator+=(const Matrix3& rhs)
{
	rows[0] += rhs.rows[0];
	rows[1] += rhs.rows[1];
	rows[2] += rhs.rows[2];
	return *this;
}

inline Matrix3& Matrix3::operator-=(const Matrix3& rhs)
{
	rows[0] -= rhs.rows[0];
	rows[1] -= rhs.rows[1];
	rows[2] -= rhs.rows[2];
	return *this;
}

inline float3D Matrix3::operator*(const float3D& rhs) const
{
	return float3D(Dot(rows[0], rhs), Dot(rows[1], rhs), Dot(rows[2], rhs));
}

inline Matrix3 Matrix3::operator*(const Matrix3& rhs) const
{
	return Matrix3( { Dot(rows[0], rhs.getCol(0)) , Dot(rows[0], rhs.getCol(1)), Dot(rows[0], rhs.getCol(2)) },
					{ Dot(rows[1], rhs.getCol(0)) , Dot(rows[1], rhs.getCol(1)), Dot(rows[1], rhs.getCol(2)) },
					{ Dot(rows[2], rhs.getCol(0)) , Dot(rows[2], rhs.getCol(1)), Dot(rows[2], rhs.getCol(2)) } );
}

inline Matrix3& Matrix3::operator*=(const Matrix3& rhs)
{
    *this = *this * rhs;
    return *this;
}

inline bool Matrix3::operator==(const Matrix3& rhs) const
{
	return (rows[0] == rhs.rows[0] && rows[1] == rhs.rows[1] && rows[2] == rhs.rows[2]);
}

inline bool Matrix3::operator!=(const Matrix3& rhs) const
{
	return !(*this == rhs);
}

//////////// friends //////////////////////
inline Matrix3 operator-(const Matrix3& rhs)
{
    return Matrix3(-rhs.rows[0], -rhs.rows[1], -rhs.rows[2]);
}

inline Matrix3 operator*(float s, const Matrix3& rhs)
{
	return Matrix3(s*rhs.rows[0], s*rhs.rows[1], s*rhs.rows[2]);
}

inline float3D operator*(const float3D& v, const Matrix3& rhs)
{
	return{ Dot(v, rhs.getCol(0)), Dot(v, rhs.getCol(1)), Dot(v, rhs.getCol(2)) };
}

inline std::ostream& operator<<(std::ostream& os, const Matrix3& m)
{
	os << std::fixed << std::setprecision(5);
	os << "[" << std::setw(10) << m.rows[0][0] << " " << std::setw(10) << m.rows[0][1] << " " << std::setw(10) << m.rows[0][2] << "]\n"
	   << "[" << std::setw(10) << m.rows[1][0] << " " << std::setw(10) << m.rows[1][1] << " " << std::setw(10) << m.rows[1][2] << "]\n"
	   << "[" << std::setw(10) << m.rows[2][0] << " " << std::setw(10) << m.rows[2][1] << " " << std::setw(10) << m.rows[2][2] << "]\n";
	os << std::resetiosflags(std::ios_base::fixed | std::ios_base::floatfield);
	return os;
}
// END OF MATRIX3 INLINE //////////////////////////////////////////////////////


/*
///////////////////////////////////////////////////////////////////////////
// 4x4 matrix
///////////////////////////////////////////////////////////////////////////
class Matrix4
{
public:
// constructors
Matrix4();  // init with identity
Matrix4(const float src[16]);
Matrix4(float m00, float m01, float m02, float m03, // 1st column
float m04, float m05, float m06, float m07, // 2nd column
float m08, float m09, float m10, float m11, // 3rd column
float m12, float m13, float m14, float m15);// 4th column

void        set(const float src[16]);
void        set(float m00, float m01, float m02, float m03, // 1st column
float m04, float m05, float m06, float m07, // 2nd column
float m08, float m09, float m10, float m11, // 3rd column
float m12, float m13, float m14, float m15);// 4th column
void        setRow(int index, const float row[4]);
void        setRow(int index, const Vector4& v);
void        setRow(int index, const Vector3& v);
void        setColumn(int index, const float col[4]);
void        setColumn(int index, const Vector4& v);
void        setColumn(int index, const Vector3& v);

const float* get() const;
const float* getTranspose();                        // return transposed matrix
float        getDeterminant();

Matrix4&    identity();
Matrix4&    transpose();                            // transpose itself and return reference
Matrix4&    invert();                               // check best inverse method before inverse
Matrix4&    invertEuclidean();                      // inverse of Euclidean transform matrix
Matrix4&    invertAffine();                         // inverse of affine transform matrix
Matrix4&    invertProjective();                     // inverse of projective matrix using partitioning
Matrix4&    invertGeneral();                        // inverse of generic matrix

// transform matrix
Matrix4&    translate(float x, float y, float z);   // translation by (x,y,z)
Matrix4&    translate(const Vector3& v);            //
Matrix4&    rotate(float angle, const Vector3& axis); // rotate angle(degree) along the given axix
Matrix4&    rotate(float angle, float x, float y, float z);
Matrix4&    rotateX(float angle);                   // rotate on X-axis with degree
Matrix4&    rotateY(float angle);                   // rotate on Y-axis with degree
Matrix4&    rotateZ(float angle);                   // rotate on Z-axis with degree
Matrix4&    scale(float scale);                     // uniform scale
Matrix4&    scale(float sx, float sy, float sz);    // scale by (sx, sy, sz) on each axis

// operators
Matrix4     operator+(const Matrix4& rhs) const;    // add rhs
Matrix4     operator-(const Matrix4& rhs) const;    // subtract rhs
Matrix4&    operator+=(const Matrix4& rhs);         // add rhs and update this object
Matrix4&    operator-=(const Matrix4& rhs);         // subtract rhs and update this object
Vector4     operator*(const Vector4& rhs) const;    // multiplication: v' = M * v
Vector3     operator*(const Vector3& rhs) const;    // multiplication: v' = M * v
Matrix4     operator*(const Matrix4& rhs) const;    // multiplication: M3 = M1 * M2
Matrix4&    operator*=(const Matrix4& rhs);         // multiplication: M1' = M1 * M2
bool        operator==(const Matrix4& rhs) const;   // exact compare, no epsilon
bool        operator!=(const Matrix4& rhs) const;   // exact compare, no epsilon
float       operator[](int index) const;            // subscript operator v[0], v[1]
float&      operator[](int index);                  // subscript operator v[0], v[1]

friend Matrix4 operator-(const Matrix4& m);                     // unary operator (-)
friend Matrix4 operator*(float scalar, const Matrix4& m);       // pre-multiplication
friend Vector3 operator*(const Vector3& vec, const Matrix4& m); // pre-multiplication
friend Vector4 operator*(const Vector4& vec, const Matrix4& m); // pre-multiplication
friend std::ostream& operator<<(std::ostream& os, const Matrix4& m);

protected:

private:
float       getCofactor(float m0, float m1, float m2,
float m3, float m4, float m5,
float m6, float m7, float m8);

float m[16];
float tm[16];                                       // transpose m

};



///////////////////////////////////////////////////////////////////////////
// inline functions for Matrix4
///////////////////////////////////////////////////////////////////////////
inline Matrix4::Matrix4()
{
    // initially identity matrix
    identity();
}



inline Matrix4::Matrix4(const float src[16])
{
    set(src);
}



inline Matrix4::Matrix4(float m00, float m01, float m02, float m03,
                        float m04, float m05, float m06, float m07,
                        float m08, float m09, float m10, float m11,
                        float m12, float m13, float m14, float m15)
{
    set(m00, m01, m02, m03,  m04, m05, m06, m07,  m08, m09, m10, m11,  m12, m13, m14, m15);
}



inline void Matrix4::set(const float src[16])
{
    m[0] = src[0];  m[1] = src[1];  m[2] = src[2];  m[3] = src[3];
    m[4] = src[4];  m[5] = src[5];  m[6] = src[6];  m[7] = src[7];
    m[8] = src[8];  m[9] = src[9];  m[10]= src[10]; m[11]= src[11];
    m[12]= src[12]; m[13]= src[13]; m[14]= src[14]; m[15]= src[15];
}



inline void Matrix4::set(float m00, float m01, float m02, float m03,
                         float m04, float m05, float m06, float m07,
                         float m08, float m09, float m10, float m11,
                         float m12, float m13, float m14, float m15)
{
    m[0] = m00;  m[1] = m01;  m[2] = m02;  m[3] = m03;
    m[4] = m04;  m[5] = m05;  m[6] = m06;  m[7] = m07;
    m[8] = m08;  m[9] = m09;  m[10]= m10;  m[11]= m11;
    m[12]= m12;  m[13]= m13;  m[14]= m14;  m[15]= m15;
}



inline void Matrix4::setRow(int index, const float row[4])
{
    m[index] = row[0];  m[index + 4] = row[1];  m[index + 8] = row[2];  m[index + 12] = row[3];
}



inline void Matrix4::setRow(int index, const Vector4& v)
{
    m[index] = v.x;  m[index + 4] = v.y;  m[index + 8] = v.z;  m[index + 12] = v.w;
}



inline void Matrix4::setRow(int index, const Vector3& v)
{
    m[index] = v.x;  m[index + 4] = v.y;  m[index + 8] = v.z;
}



inline void Matrix4::setColumn(int index, const float col[4])
{
    m[index*4] = col[0];  m[index*4 + 1] = col[1];  m[index*4 + 2] = col[2];  m[index*4 + 3] = col[3];
}



inline void Matrix4::setColumn(int index, const Vector4& v)
{
    m[index*4] = v.x;  m[index*4 + 1] = v.y;  m[index*4 + 2] = v.z;  m[index*4 + 3] = v.w;
}



inline void Matrix4::setColumn(int index, const Vector3& v)
{
    m[index*4] = v.x;  m[index*4 + 1] = v.y;  m[index*4 + 2] = v.z;
}



inline const float* Matrix4::get() const
{
    return m;
}



inline const float* Matrix4::getTranspose()
{
    tm[0] = m[0];   tm[1] = m[4];   tm[2] = m[8];   tm[3] = m[12];
    tm[4] = m[1];   tm[5] = m[5];   tm[6] = m[9];   tm[7] = m[13];
    tm[8] = m[2];   tm[9] = m[6];   tm[10]= m[10];  tm[11]= m[14];
    tm[12]= m[3];   tm[13]= m[7];   tm[14]= m[11];  tm[15]= m[15];
    return tm;
}



inline Matrix4& Matrix4::identity()
{
    m[0] = m[5] = m[10] = m[15] = 1.0f;
    m[1] = m[2] = m[3] = m[4] = m[6] = m[7] = m[8] = m[9] = m[11] = m[12] = m[13] = m[14] = 0.0f;
    return *this;
}



inline Matrix4 Matrix4::operator+(const Matrix4& rhs) const
{
    return Matrix4(m[0]+rhs[0],   m[1]+rhs[1],   m[2]+rhs[2],   m[3]+rhs[3],
                   m[4]+rhs[4],   m[5]+rhs[5],   m[6]+rhs[6],   m[7]+rhs[7],
                   m[8]+rhs[8],   m[9]+rhs[9],   m[10]+rhs[10], m[11]+rhs[11],
                   m[12]+rhs[12], m[13]+rhs[13], m[14]+rhs[14], m[15]+rhs[15]);
}



inline Matrix4 Matrix4::operator-(const Matrix4& rhs) const
{
    return Matrix4(m[0]-rhs[0],   m[1]-rhs[1],   m[2]-rhs[2],   m[3]-rhs[3],
                   m[4]-rhs[4],   m[5]-rhs[5],   m[6]-rhs[6],   m[7]-rhs[7],
                   m[8]-rhs[8],   m[9]-rhs[9],   m[10]-rhs[10], m[11]-rhs[11],
                   m[12]-rhs[12], m[13]-rhs[13], m[14]-rhs[14], m[15]-rhs[15]);
}



inline Matrix4& Matrix4::operator+=(const Matrix4& rhs)
{
    m[0] += rhs[0];   m[1] += rhs[1];   m[2] += rhs[2];   m[3] += rhs[3];
    m[4] += rhs[4];   m[5] += rhs[5];   m[6] += rhs[6];   m[7] += rhs[7];
    m[8] += rhs[8];   m[9] += rhs[9];   m[10]+= rhs[10];  m[11]+= rhs[11];
    m[12]+= rhs[12];  m[13]+= rhs[13];  m[14]+= rhs[14];  m[15]+= rhs[15];
    return *this;
}



inline Matrix4& Matrix4::operator-=(const Matrix4& rhs)
{
    m[0] -= rhs[0];   m[1] -= rhs[1];   m[2] -= rhs[2];   m[3] -= rhs[3];
    m[4] -= rhs[4];   m[5] -= rhs[5];   m[6] -= rhs[6];   m[7] -= rhs[7];
    m[8] -= rhs[8];   m[9] -= rhs[9];   m[10]-= rhs[10];  m[11]-= rhs[11];
    m[12]-= rhs[12];  m[13]-= rhs[13];  m[14]-= rhs[14];  m[15]-= rhs[15];
    return *this;
}



inline Vector4 Matrix4::operator*(const Vector4& rhs) const
{
    return Vector4(m[0]*rhs.x + m[4]*rhs.y + m[8]*rhs.z  + m[12]*rhs.w,
                   m[1]*rhs.x + m[5]*rhs.y + m[9]*rhs.z  + m[13]*rhs.w,
                   m[2]*rhs.x + m[6]*rhs.y + m[10]*rhs.z + m[14]*rhs.w,
                   m[3]*rhs.x + m[7]*rhs.y + m[11]*rhs.z + m[15]*rhs.w);
}



inline Vector3 Matrix4::operator*(const Vector3& rhs) const
{
    return Vector3(m[0]*rhs.x + m[4]*rhs.y + m[8]*rhs.z,
                   m[1]*rhs.x + m[5]*rhs.y + m[9]*rhs.z,
                   m[2]*rhs.x + m[6]*rhs.y + m[10]*rhs.z);
}



inline Matrix4 Matrix4::operator*(const Matrix4& n) const
{
    return Matrix4(m[0]*n[0]  + m[4]*n[1]  + m[8]*n[2]  + m[12]*n[3],   m[1]*n[0]  + m[5]*n[1]  + m[9]*n[2]  + m[13]*n[3],   m[2]*n[0]  + m[6]*n[1]  + m[10]*n[2]  + m[14]*n[3],   m[3]*n[0]  + m[7]*n[1]  + m[11]*n[2]  + m[15]*n[3],
                   m[0]*n[4]  + m[4]*n[5]  + m[8]*n[6]  + m[12]*n[7],   m[1]*n[4]  + m[5]*n[5]  + m[9]*n[6]  + m[13]*n[7],   m[2]*n[4]  + m[6]*n[5]  + m[10]*n[6]  + m[14]*n[7],   m[3]*n[4]  + m[7]*n[5]  + m[11]*n[6]  + m[15]*n[7],
                   m[0]*n[8]  + m[4]*n[9]  + m[8]*n[10] + m[12]*n[11],  m[1]*n[8]  + m[5]*n[9]  + m[9]*n[10] + m[13]*n[11],  m[2]*n[8]  + m[6]*n[9]  + m[10]*n[10] + m[14]*n[11],  m[3]*n[8]  + m[7]*n[9]  + m[11]*n[10] + m[15]*n[11],
                   m[0]*n[12] + m[4]*n[13] + m[8]*n[14] + m[12]*n[15],  m[1]*n[12] + m[5]*n[13] + m[9]*n[14] + m[13]*n[15],  m[2]*n[12] + m[6]*n[13] + m[10]*n[14] + m[14]*n[15],  m[3]*n[12] + m[7]*n[13] + m[11]*n[14] + m[15]*n[15]);
}



inline Matrix4& Matrix4::operator*=(const Matrix4& rhs)
{
    *this = *this * rhs;
    return *this;
}



inline bool Matrix4::operator==(const Matrix4& n) const
{
    return (m[0] == n[0])  && (m[1] == n[1])  && (m[2] == n[2])  && (m[3] == n[3])  &&
           (m[4] == n[4])  && (m[5] == n[5])  && (m[6] == n[6])  && (m[7] == n[7])  &&
           (m[8] == n[8])  && (m[9] == n[9])  && (m[10]== n[10]) && (m[11]== n[11]) &&
           (m[12]== n[12]) && (m[13]== n[13]) && (m[14]== n[14]) && (m[15]== n[15]);
}



inline bool Matrix4::operator!=(const Matrix4& n) const
{
    return (m[0] != n[0])  || (m[1] != n[1])  || (m[2] != n[2])  || (m[3] != n[3])  ||
           (m[4] != n[4])  || (m[5] != n[5])  || (m[6] != n[6])  || (m[7] != n[7])  ||
           (m[8] != n[8])  || (m[9] != n[9])  || (m[10]!= n[10]) || (m[11]!= n[11]) ||
           (m[12]!= n[12]) || (m[13]!= n[13]) || (m[14]!= n[14]) || (m[15]!= n[15]);
}



inline float Matrix4::operator[](int index) const
{
    return m[index];
}



inline float& Matrix4::operator[](int index)
{
    return m[index];
}



inline Matrix4 operator-(const Matrix4& rhs)
{
    return Matrix4(-rhs[0], -rhs[1], -rhs[2], -rhs[3], -rhs[4], -rhs[5], -rhs[6], -rhs[7], -rhs[8], -rhs[9], -rhs[10], -rhs[11], -rhs[12], -rhs[13], -rhs[14], -rhs[15]);
}



inline Matrix4 operator*(float s, const Matrix4& rhs)
{
    return Matrix4(s*rhs[0], s*rhs[1], s*rhs[2], s*rhs[3], s*rhs[4], s*rhs[5], s*rhs[6], s*rhs[7], s*rhs[8], s*rhs[9], s*rhs[10], s*rhs[11], s*rhs[12], s*rhs[13], s*rhs[14], s*rhs[15]);
}



inline Vector4 operator*(const Vector4& v, const Matrix4& m)
{
    return Vector4(v.x*m[0] + v.y*m[1] + v.z*m[2] + v.w*m[3],  v.x*m[4] + v.y*m[5] + v.z*m[6] + v.w*m[7],  v.x*m[8] + v.y*m[9] + v.z*m[10] + v.w*m[11], v.x*m[12] + v.y*m[13] + v.z*m[14] + v.w*m[15]);
}



inline Vector3 operator*(const Vector3& v, const Matrix4& m)
{
    return Vector3(v.x*m[0] + v.y*m[1] + v.z*m[2],  v.x*m[4] + v.y*m[5] + v.z*m[6],  v.x*m[8] + v.y*m[9] + v.z*m[10]);
}



inline std::ostream& operator<<(std::ostream& os, const Matrix4& m)
{
    os << std::fixed << std::setprecision(5);
    os << "[" << std::setw(10) << m[0] << " " << std::setw(10) << m[4] << " " << std::setw(10) << m[8]  <<  " " << std::setw(10) << m[12] << "]\n"
       << "[" << std::setw(10) << m[1] << " " << std::setw(10) << m[5] << " " << std::setw(10) << m[9]  <<  " " << std::setw(10) << m[13] << "]\n"
       << "[" << std::setw(10) << m[2] << " " << std::setw(10) << m[6] << " " << std::setw(10) << m[10] <<  " " << std::setw(10) << m[14] << "]\n"
       << "[" << std::setw(10) << m[3] << " " << std::setw(10) << m[7] << " " << std::setw(10) << m[11] <<  " " << std::setw(10) << m[15] << "]\n";
    os << std::resetiosflags(std::ios_base::fixed | std::ios_base::floatfield);
    return os;
}
// END OF MATRIX4 INLINE //////////////////////////////////////////////////////
*/
#endif
