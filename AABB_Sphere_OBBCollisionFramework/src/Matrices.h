///////////////////////////////////////////////////////////////////////////////
// Matrice.h
// =========
// NxN Matrix Math classes
// Reza Nezami: It is not row-first matrix

///////////////////////////////////////////////////////////////////////////////
#pragma once

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

	float2D getRow(size_t id) const {
		assert(id < 2); return rows[id];
	}
	float2D getCol(size_t id) const {
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
		rows[0][idx] = v[0];
		rows[1][idx] = v[1];
		rows[2][idx] = v[2];
	}

	float3D getRow(size_t i) const {
		assert(i < 3); return rows[i];
	}
	float3D getCol(size_t i) const
	{
		assert(i < 3);
		return{ rows[0][i], rows[1][i], rows[2][i] };

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
	float3D&      operator[](int index);            // subscript operator v[0], v[1]
	float3D       operator[](int index) const;            // subscript operator v[0], v[1]

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
inline float3D Matrix3::operator[](int index) const
{
	return rows[index];
}

inline float3D& Matrix3::operator[](int index)
{
	return rows[index];
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



///////////////////////////////////////////////////////////////////////////
// 4x4 matrix
///////////////////////////////////////////////////////////////////////////
class Matrix4
{
public:
	// constructors
	Matrix4();  // init with identity matrix
	Matrix4(const float4D& row1, const float4D& row2, const float4D& row3, const float4D& row4)
		: rows{ row1, row2, row3, row4 } {}

	void  setRow(size_t idx, const float4D& row)
	{
		assert(idx <= 3);
		rows[idx] = row;
	}
	void  setCol(size_t idx, float4D& v) {
		rows[0][idx] = v[0];
		rows[1][idx] = v[1];
		rows[2][idx] = v[2];
		rows[3][idx] = v[3];
	}

	float4D getRow(size_t i) const {
		assert(i <= 3); return rows[i];
	}
	float4D getCol(size_t i) const
	{
		assert(i <= 3);
		return{ rows[0][i], rows[1][i], rows[2][i], rows[3][i] };

	}
	static Matrix4  identity() {
		return Matrix4({ 1.f, 0.f, 0.f, 0.f },
						{ 0.f, 1.f, 0.f, 0.f },
						{ 0.f, 0.f, 1.f, 0.f },
						{ 0.f, 0.f, 0.f, 1.f });
	}
	static Matrix4  zero() {
		return Matrix4({ 0.f, 0.f, 0.f },
						{ 0.f, 0.f, 0.f },
						{ 0.f, 0.f, 0.f },
						{ 0.f, 0.f, 0.f, 0.f });
	}
	Matrix4    transpose() { return Matrix4(getCol(0), getCol(1), getCol(2), getCol(3)); }
	
	// stataic transform matrix
	static Matrix4    Translate(const float3D& v)           //translation by (x,y,z)
	{
		Matrix4 m{ identity() };
		m[3] = float4D{ v.x, v.y, v.z, 1.f };
		return m;
	}
	static Matrix4    Rotate(float angle, const float3D& inaxis) // rotate angle(degree) along the given axix
	{
		float3D axis = Normal(inaxis);
		auto DegToRad = [](float a) { return a * 3.1415926f / 180.f; };
		float c = cos(DegToRad(angle));
		float s = sin(DegToRad(angle));
		Matrix4 m{ identity() };
		m[0][0] = (axis.x * axis.x) * (1.0f - c) + c;
		m[0][1] = (axis.y * axis.x) * (1.0f - c) + (axis.z * s);
		m[0][2] = (axis.z * axis.x) * (1.0f - c) - (axis.y * s);

		m[1][0] = (axis.x * axis.y) * (1.0f - c) - (axis.z * s);
		m[1][1] = (axis.y * axis.y) * (1.0f - c) + c;
		m[1][2] = (axis.z * axis.y) * (1.0f - c) + (axis.x * s);

		m[2][0] = (axis.x * axis.z) * (1.0f - c) + (axis.y * s);
		m[2][1] = (axis.y * axis.z) * (1.0f - c) - (axis.x * s);
		m[2][2] = (axis.z * axis.z) * (1.0f - c) + c;

		return m;
	}
	static Matrix4    Scale(float3D scale)                    // uniform scale
	{
		Matrix4 m{ identity() };
		m[0][0] = scale.x;
		m[1][1] = scale.y;
		m[2][2] = scale.z;
		return m;
	}


	float3D		GetPositionVector() const
	{
		return float3D(rows[3][0], rows[3][1], rows[3][2]);
	}

	void SetPosition(float3D const& pos)
	{
		rows[3][0] = pos[0]; rows[3][1] = pos[1]; rows[3][2] = pos[2];
	}

	// operators
	Matrix4     operator+(const Matrix4& rhs) const;    // add rhs
	Matrix4     operator-(const Matrix4& rhs) const;    // subtract rhs
	Matrix4&    operator+=(const Matrix4& rhs);         // add rhs and update this object
	Matrix4&    operator-=(const Matrix4& rhs);         // subtract rhs and update this object
	float4D     operator*(const float4D& rhs) const;    // multiplication: v' = M * v
	Matrix4     operator*(const Matrix4& rhs) const;    // multiplication: M3 = M1 * M2
	Matrix4&    operator*=(const Matrix4& rhs);         // multiplication: M1' = M1 * M2
	bool        operator==(const Matrix4& rhs) const;   // exact compare, no epsilon
	bool        operator!=(const Matrix4& rhs) const;   // exact compare, no epsilon
	float4D       operator[](int index) const;            // subscript operator v[0], v[1]
	float4D&      operator[](int index);                  // subscript operator v[0], v[1]

	friend Matrix4 operator-(const Matrix4& m);                     // unary operator (-)
	friend Matrix4 operator*(float scalar, const Matrix4& m);       // pre-multiplication
	//friend float4D operator*(const float4D& vec, const Matrix4& m); // pre-multiplication
	friend std::ostream& operator<<(std::ostream& os, const Matrix4& m);

protected:

private:
	float4D rows[4];
};

///////////////////////////////////////////////////////////////////////////
// inline functions for Matrix3
///////////////////////////////////////////////////////////////////////////

inline float4D Matrix4::operator*(const float4D& rhs) const
{
	return float4D(Dot(rows[0], rhs), Dot(rows[1], rhs), Dot(rows[2], rhs), Dot(rows[3], rhs));
}

inline Matrix4 Matrix4::operator*(const Matrix4& rhs) const
{
	return Matrix4(	{ Dot(rows[0], rhs.getCol(0)), Dot(rows[0], rhs.getCol(1)), Dot(rows[0], rhs.getCol(2)), Dot(rows[0], rhs.getCol(3)) },
					{ Dot(rows[1], rhs.getCol(0)), Dot(rows[1], rhs.getCol(1)), Dot(rows[1], rhs.getCol(2)), Dot(rows[1], rhs.getCol(3)) },
					{ Dot(rows[2], rhs.getCol(0)), Dot(rows[2], rhs.getCol(1)), Dot(rows[2], rhs.getCol(2)), Dot(rows[2], rhs.getCol(3)) },
					{ Dot(rows[3], rhs.getCol(0)), Dot(rows[3], rhs.getCol(1)), Dot(rows[3], rhs.getCol(2)), Dot(rows[3], rhs.getCol(3)) });
}

///////////////////////////////////////////////////////////////////////////
// inline functions for Matrix4
///////////////////////////////////////////////////////////////////////////
inline Matrix4::Matrix4()
{
    // initially identity matrix
	*this = identity();
}

inline Matrix4 Matrix4::operator+(const Matrix4& rhs) const
{
	return Matrix4(rows[0] + rhs.rows[0], rows[1] + rhs.rows[1], rows[2] + rhs.rows[2], rows[3] + rhs.rows[3]);
}

inline Matrix4 Matrix4::operator-(const Matrix4& rhs) const
{
	return Matrix4(rows[0] - rhs.rows[0], rows[1] - rhs.rows[1], rows[2] - rhs.rows[2], rows[3] - rhs.rows[3]);
}

inline Matrix4& Matrix4::operator+=(const Matrix4& rhs)
{
	rows[0] += rhs.rows[0];
	rows[1] += rhs.rows[1];
	rows[2] += rhs.rows[2];
	rows[3] += rhs.rows[3];
	return *this;
}

inline Matrix4& Matrix4::operator-=(const Matrix4& rhs)
{
	rows[0] -= rhs.rows[0];
	rows[1] -= rhs.rows[1];
	rows[2] -= rhs.rows[2];
	rows[3] -= rhs.rows[3];
	return *this;
}

inline Matrix4& Matrix4::operator*=(const Matrix4& rhs)
{
	*this = *this * rhs;
	return *this;
}

inline bool Matrix4::operator==(const Matrix4& rhs) const
{
	return (rows[0] == rhs.rows[0] && rows[1] == rhs.rows[1] && rows[2] == rhs.rows[2] && rows[3] == rhs.rows[3]);
}

inline bool Matrix4::operator!=(const Matrix4& rhs) const
{
	return !(*this == rhs);
}
inline Matrix4 operator-(const Matrix4& rhs)
{
	return -1.f * rhs;
}

inline Matrix4 operator*(float s, const Matrix4& rhs)
{
	return Matrix4(s*rhs.rows[0], s*rhs.rows[1], s*rhs.rows[2], s*rhs.rows[3]);
}

inline float4D operator*(const float4D& v, const Matrix4& rhs)
{
	return{ Dot(v, rhs.getCol(0)), Dot(v, rhs.getCol(1)), Dot(v, rhs.getCol(2)), Dot(v, rhs.getCol(3)) };
}
inline float4D Matrix4::operator[](int index) const
{
    return rows[index];
}

inline float4D& Matrix4::operator[](int index)
{
    return rows[index];
}

inline std::ostream& operator<<(std::ostream& os, const Matrix4& m)
{
    os << std::fixed << std::setprecision(5);
    os << "[" << std::setw(10) << m.rows[0][0] << " " << std::setw(10) << m.rows[0][1] << " " << std::setw(10) << m.rows[0][2] <<  " " << std::setw(10) << m.rows[0][3] << "]\n"
       << "[" << std::setw(10) << m.rows[1][0] << " " << std::setw(10) << m.rows[1][1] << " " << std::setw(10) << m.rows[1][2] <<  " " << std::setw(10) << m.rows[1][3] << "]\n"
       << "[" << std::setw(10) << m.rows[2][0] << " " << std::setw(10) << m.rows[2][1] << " " << std::setw(10) << m.rows[2][2] <<  " " << std::setw(10) << m.rows[2][3] << "]\n"
       << "[" << std::setw(10) << m.rows[3][0] << " " << std::setw(10) << m.rows[3][1] << " " << std::setw(10) << m.rows[3][2] <<  " " << std::setw(10) << m.rows[3][3] << "]\n";
    os << std::resetiosflags(std::ios_base::fixed | std::ios_base::floatfield);
    return os;
}
// END OF MATRIX4 INLINE //////////////////////////////////////////////////////
