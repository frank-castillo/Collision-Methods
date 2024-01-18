#include <stdio.h>
#include <string.h>
#define _USE_MATH_DEFINES

#include <cmath>
#include <stdlib.h>
#include <ctime>
#include <assert.h>
#include <algorithm>
#include <iostream>
#include <vector>
#include <memory>

#ifdef WIN32
#include <windows.h>
#endif

#ifndef MACOSX
#include <GL/gl.h>
#include <GL/glu.h>
#else
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#endif

#include "wcode/fssimplewindow.h"
#include "wcode/fswin32keymap.h"
#include "bitmapfont\ysglfontdata.h"

#include "vectors.h"
#include "Matrices.h"
#include "eig3.h"

using namespace std;
typedef enum
{
	eIdle = -2,
	eStop = -1,
	eStart = 0,
} changeType;

float iSpeed{ 70.0 };
float radius{ 20. };
size_t num_segments{ 20 };

typedef enum class eCollisionMode
{
	eSphere,
	eAABB,
	eOBB
};

eCollisionMode CollisionType{ eCollisionMode::eSphere };

size_t PolyCount{ 5 };  // number of polygons to launch. 

const unsigned frameRate = 30; // 30 frame per second is the frame rate.
const int timeSpan = 1000 / frameRate; // milliseconds
const double timeInc = (double)timeSpan * 0.001; // time increment in seconds

size_t WinWidth{ 800 }, WinHeight{ 600 };

///////////////////// physics stuff ///////////////////
struct AABB
{// pos is center of AABB, extent is half length extend in each direction
	float3D pos, extend;
};

struct OBB
{
	float3D center, extend;  // contains the legnth of each extent: (a.length(), b.length(), c.length())
	Matrix3 orient;  // 3 half extent directions as rows 1, 2, and 3, [ a direction, b direction, c direction]
	OBB() = default;

	void set(float3D c, float3D e, Matrix3 o)
	{
		center = c;
		extend = e;
		orient = o;
	};
};

struct Sphere
{
	float3D center{};
	float radius{ -1 };
};

struct PhysicsObj
{
	float3D pos, vel; // pos and orientation is the same as OBB's.
	float3D rotationAxis;
	float rotationSpeed;
	Matrix3 orientation;
	bool Colliding;
	OBB obb;
	Sphere mSphere;
	float3D OBB_Extent; // along with orientation and pos, makes up OBB of this object
	float3D AABB_Extent;// along with pos, makes up AABB of this object
	float radius; // along with pos, makes up bounding sphere of this object
	float rotation; // rotation angle around orientation axis to orient the object in WCS.

	PhysicsObj(float3D p, float3D v, float r = 0.f, float rotS = 0.f, float rot = 0.f) :
		pos(p),
		vel(v),
		radius(r),
		Colliding(false),
		rotationAxis({ 0.f, 0.f, 1.f }),
		rotationSpeed(rotS),
		rotation(rot)
	{
		AABB_Extent = { 10., 10., 10. };
	}

	float distSq(PhysicsObj const& obj) const {
		return (pos - obj.pos).LengthSq();
	};

	void SetSphere(std::vector<float3D> const& vertices)
	{
		float nSides = vertices.size();

		// step1: find mean position
		float3D meanPosition{};
		std::for_each(vertices.begin(), vertices.end(), [&](auto p) {meanPosition += p; });
		meanPosition = { meanPosition / nSides };

		//compute covariance matrix
		Matrix3 covarianceMatrix = Matrix3::zero();
		float totalCovSum = 0;
		float covElementMultiplication = 0;
		float3D covarianceTopRow = { 0.0f, 0.0f, 0.0f };
		float3D covarianceMiddleRow = { 0.0f, 0.0f, 0.0f };
		float3D covarianceBottomRow = { 0.0f, 0.0f, 0.0f };

		for (int i = 0; i < nSides; ++i)
		{
			covElementMultiplication = (vertices[i].x - meanPosition.x) * (vertices[i].x - meanPosition.x);
			totalCovSum = totalCovSum + covElementMultiplication;
		}
		covarianceTopRow.x = (1 / nSides) * totalCovSum; // Set up C[1][1]

		totalCovSum = 0;
		for (int i = 0; i < nSides; ++i)
		{
			covElementMultiplication = (vertices[i].x - meanPosition.x) * (vertices[i].y - meanPosition.y);
			totalCovSum = totalCovSum + covElementMultiplication;
		}
		covarianceTopRow.y = (1 / nSides) * totalCovSum; // Set up C[1][2]

		totalCovSum = 0;
		for (int i = 0; i < nSides; ++i)
		{
			covElementMultiplication = (vertices[i].x - meanPosition.x) * (vertices[i].z - meanPosition.z);
			totalCovSum = totalCovSum + covElementMultiplication;
		}
		covarianceTopRow.z = (1 / nSides) * totalCovSum; // Set up C[1][3]
		covarianceMatrix.setRow(0, covarianceTopRow);

		covarianceMiddleRow.x = covarianceTopRow.y; // Set up C[2][1]

		totalCovSum = 0;
		for (int i = 0; i < nSides; ++i)
		{
			covElementMultiplication = (vertices[i].y - meanPosition.y) * (vertices[i].y - meanPosition.y);
			totalCovSum = totalCovSum + covElementMultiplication;
		}
		covarianceMiddleRow.y = (1 / nSides) * totalCovSum; // Set up C[2][2]

		totalCovSum = 0;
		for (int i = 0; i < nSides; ++i)
		{
			covElementMultiplication = (vertices[i].y - meanPosition.y) * (vertices[i].z - meanPosition.z);
			totalCovSum = totalCovSum + covElementMultiplication;
		}
		covarianceMiddleRow.z = (1 / nSides) * totalCovSum; // Set up C[2][3]
		covarianceMatrix.setRow(1, covarianceMiddleRow);

		covarianceBottomRow.x = covarianceTopRow.z; // Set up C[3][1]
		covarianceBottomRow.y = covarianceMiddleRow.z; // Set up C[3][2]

		totalCovSum = 0;
		for (int i = 0; i < nSides; ++i)
		{
			covElementMultiplication = (vertices[i].z - meanPosition.z) * (vertices[i].z - meanPosition.z);
			totalCovSum = totalCovSum + covElementMultiplication;
		}
		covarianceBottomRow.z = (1 / nSides) * totalCovSum; // Set up C[3][3]
		covarianceMatrix.setRow(1, covarianceBottomRow);

		//setup 2d array to be used in eigen_decomposition()
		double covarianceMatrixArray[3][3] =
		{
			{ covarianceMatrix.getRow(0).x, covarianceMatrix.getRow(0).y, covarianceMatrix.getRow(0).z },
			{ covarianceMatrix.getRow(1).x, covarianceMatrix.getRow(1).y, covarianceMatrix.getRow(1).z },
			{ covarianceMatrix.getRow(2).x, covarianceMatrix.getRow(2).y, covarianceMatrix.getRow(2).z }
		};

		double eigenVecs[3][3]{};
		double eigenValues[3]{};
		eigen_decomposition(covarianceMatrixArray, eigenVecs, eigenValues);

		// setup orient matrix 
		Matrix3 orient{};
		Matrix3 eigenVectors{};  // use eigenVectors.setRow() to set the 3 rows of orient to 3 eigenVecs
		for (uint32_t i = 0; i < 3; ++i)
			eigenVectors.setRow(i, float3D(eigenVecs[i][0], eigenVecs[i][1], eigenVecs[i][2]));

		float3D T = eigenVectors.getCol(0); //T
		float3D S = eigenVectors.getCol(1); //S
		float3D R = eigenVectors.getCol(2); //R
		assert(eigenValues[2] >= eigenValues[1]);
		assert(eigenValues[2] >= eigenValues[0]);
		assert(eigenValues[1] >= eigenValues[0]);
		orient.setRow(0, R);
		orient.setRow(1, S);
		orient.setRow(2, T);

		// compute min/max extends along R, S, and T:
		float minExtendR{}, maxExtendR{};
		float minExtendS{}, maxExtendS{};
		float minExtendT{}, maxExtendT{};
		float findR{}, findS{}, findT{};
		float minIterator = 0;
		float maxIterator = 0;

		for (auto i = 0; i < nSides; ++i)
		{
			// update minExt* and maxExt* here
			// We find the dot product of each point with the eigen vector and compare to find min and max value for each vertex and eigen vector

			findR = Dot(vertices[i], R);
			if (findR < minExtendR)
			{
				minIterator = i;
				minExtendR = findR;
			}
			else if (findR > maxExtendR)
			{
				maxIterator = i;
				maxExtendR = findR;
			}
		}

		minExtendR = abs(minExtendR);

		mSphere.center = (vertices[maxIterator] + vertices[minIterator]) / 2; 
		float3D diff = vertices[maxIterator] - mSphere.center;
		float dist = diff.Length();
		mSphere.radius = dist;
		float3D g;

		for (int i = 0; i < nSides; ++i)
		{
			diff = vertices[i] - mSphere.center;
			dist = diff.Length();

			if ((dist * dist) > (mSphere.radius * mSphere.radius))
			{
				g = mSphere.center - (mSphere.radius * ((vertices[i] - mSphere.center) / dist));
				mSphere.center = (g + vertices[i]) / 2;
				diff = vertices[i] - mSphere.center;
				dist = diff.Length();
				mSphere.radius = dist;
			}
		}
	}
};
////////////////////////////////////////////////////////

// all 3-8 sided polygons. It has a center and n vertices around it at average distance d from the center.
struct MyPolygon : PhysicsObj
{ // center position for a polygon is center of its OBB 
	size_t nSides;
	Vector3d<unsigned> color;
	std::vector<float3D> vertices;

	void setVertex(unsigned n, float3D v) {
		assert(n < nSides);
		vertices.push_back(v);
	}

	MyPolygon(float3D p, float3D v, unsigned n) : PhysicsObj(p, v)
	{
		vertices.reserve(n);
		nSides = n;
	}

	~MyPolygon()
	{
		vertices.clear();
	}

	void SetSphere()
	{
		PhysicsObj::SetSphere(vertices);
	}

	// fill up steps bellow with proper code using Principle component analysis (PCA)
	void computePrinicipleAxis()
	{
		// first set the rough radius as the distance of the farthest vertex from pos
		radius = 0.f;
		std::for_each(vertices.begin(), vertices.end(), [&](float3D& n) {radius = max(n.Length(), radius); });

		// step1: find mean position
		float3D meanPosition{};
		std::for_each(vertices.begin(), vertices.end(), [&](float3D& p) {meanPosition += p; });
		meanPosition = { meanPosition / nSides };

		//compute covariance matrix
		Matrix3 covarianceMatrix = Matrix3::zero();
		float totalCovSum = 0;
		float covElementMultiplication = 0;
		float3D covarianceTopRow = { 0.0f, 0.0f, 0.0f };
		float3D covarianceMiddleRow = { 0.0f, 0.0f, 0.0f };
		float3D covarianceBottomRow = { 0.0f, 0.0f, 0.0f };

		for (int i = 0; i < nSides; ++i)
		{
			covElementMultiplication = (vertices[i].x - meanPosition.x) * (vertices[i].x - meanPosition.x);
			totalCovSum = totalCovSum + covElementMultiplication;
		}
		covarianceTopRow.x = (1 / nSides) * totalCovSum; // Set up C[1][1]

		totalCovSum = 0;
		for (int i = 0; i < nSides; ++i)
		{
			covElementMultiplication = (vertices[i].x - meanPosition.x) * (vertices[i].y - meanPosition.y);
			totalCovSum = totalCovSum + covElementMultiplication;
		}
		covarianceTopRow.y = (1 / nSides) * totalCovSum; // Set up C[1][2]

		totalCovSum = 0;
		for (int i = 0; i < nSides; ++i)
		{
			covElementMultiplication = (vertices[i].x - meanPosition.x) * (vertices[i].z - meanPosition.z);
			totalCovSum = totalCovSum + covElementMultiplication;
		}
		covarianceTopRow.z = (1 / nSides) * totalCovSum; // Set up C[1][3]
		covarianceMatrix.setRow(0, covarianceTopRow);

		covarianceMiddleRow.x = covarianceTopRow.y; // Set up C[2][1]

		totalCovSum = 0;
		for (int i = 0; i < nSides; ++i)
		{
			covElementMultiplication = (vertices[i].y - meanPosition.y) * (vertices[i].y - meanPosition.y);
			totalCovSum = totalCovSum + covElementMultiplication;
		}
		covarianceMiddleRow.y = (1 / nSides) * totalCovSum; // Set up C[2][2]

		totalCovSum = 0;
		for (int i = 0; i < nSides; ++i)
		{
			covElementMultiplication = (vertices[i].y - meanPosition.y) * (vertices[i].z - meanPosition.z);
			totalCovSum = totalCovSum + covElementMultiplication;
		}
		covarianceMiddleRow.z = (1 / nSides) * totalCovSum; // Set up C[2][3]
		covarianceMatrix.setRow(1, covarianceMiddleRow);

		covarianceBottomRow.x = covarianceTopRow.z; // Set up C[3][1]
		covarianceBottomRow.y = covarianceMiddleRow.z; // Set up C[3][2]

		totalCovSum = 0;
		for (int i = 0; i < nSides; ++i)
		{
			covElementMultiplication = (vertices[i].z - meanPosition.z) * (vertices[i].z - meanPosition.z);
			totalCovSum = totalCovSum + covElementMultiplication;
		}
		covarianceBottomRow.z = (1 / nSides) * totalCovSum; // Set up C[3][3]
		covarianceMatrix.setRow(1, covarianceBottomRow);

		//setup 2d array to be used in eigen_decomposition()
		double covarianceMatrixArray[3][3] =
		{
			{ covarianceMatrix.getRow(0).x, covarianceMatrix.getRow(0).y, covarianceMatrix.getRow(0).z },
			{ covarianceMatrix.getRow(1).x, covarianceMatrix.getRow(1).y, covarianceMatrix.getRow(1).z },
			{ covarianceMatrix.getRow(2).x, covarianceMatrix.getRow(2).y, covarianceMatrix.getRow(2).z }
		};

		double eigenVecs[3][3]{};
		double eigenValues[3]{};
		eigen_decomposition(covarianceMatrixArray, eigenVecs, eigenValues);

		// setup orient matrix 
		Matrix3 orient{};
		Matrix3 eigenVectors{};  // use eigenVectors.setRow() to set the 3 rows of orient to 3 eigenVecs
		for (uint32_t i = 0; i < 3; ++i)
			eigenVectors.setRow(i, float3D(eigenVecs[i][0], eigenVecs[i][1], eigenVecs[i][2]));

		float3D T = eigenVectors.getCol(0); //T
		float3D S = eigenVectors.getCol(1); //S
		float3D R = eigenVectors.getCol(2); //R
		assert(eigenValues[2] >= eigenValues[1]);
		assert(eigenValues[2] >= eigenValues[0]);
		assert(eigenValues[1] >= eigenValues[0]);
		orient.setRow(0, R);
		orient.setRow(1, S);
		orient.setRow(2, T);

		// compute min/max extends along R, S, and T:
		float minExtendR{}, maxExtendR{};
		float minExtendS{}, maxExtendS{};
		float minExtendT{}, maxExtendT{};
		float findR{}, findS{}, findT{};

		for (auto i = 0; i < nSides; ++i)
		{
			// update minExt* and maxExt* here
			// We find the dot product of each point with the eigen vector and compare to find min and max value for each vertex and eigen vector

			findR = Dot(vertices[i], R);
			if (findR < minExtendR)
			{
				minExtendR = findR;
			}
			else if (findR > maxExtendR)
			{
				maxExtendR = findR;
			}

			findS = Dot(vertices[i], S);
			if (findS < minExtendS)
			{
				minExtendS = findS;
			}
			else if (findS > maxExtendS)
			{
				maxExtendS = findS;
			}

			findT = Dot(vertices[i], T);
			if (findS < minExtendT)
			{
				minExtendT = findS;
			}
			else if (findS > maxExtendT)
			{
				maxExtendT = findS;
			}
		}

		// Get absolute value of minimum eigen values
		minExtendR = abs(minExtendR);
		minExtendS = abs(minExtendS);
		minExtendT = abs(minExtendT);

		float3D extend = { (maxExtendR - minExtendR) * 0.5f , (maxExtendS - minExtendS) * 0.5f, (maxExtendT - minExtendT) * 0.5f };
		float3D covMean{ (maxExtendR + minExtendR) * 0.5f , (maxExtendS + minExtendS) * 0.5f , (maxExtendT + minExtendT) * 0.5f };
		float3D covCenter = (covMean.x * R) + (covMean.y * S) + (covMean.z * T);
		obb.set(covCenter, extend, orient);
	}

};
vector<shared_ptr<MyPolygon>> sPolygons;


///////////////////// collision stuff ////////////////////
struct CollisionData
{
	CollisionData(shared_ptr<PhysicsObj> a, shared_ptr<PhysicsObj> b) {
		pair[0] = a;
		pair[1] = b;
	}
	float3D pt, MTD;  // MTD is minimum translation distance vector. pt is point of contact.
	shared_ptr<PhysicsObj> pair[2];
};
vector<shared_ptr<CollisionData>> CollisionDataList;

//////////////////////////////////////////////////////////////////////////////////////
void DrawCircle(Sphere const& sphere, float3D color, float3D pos = float3D{})
{
	float3D center{ pos + sphere.center };
	float cx{ center.x };
	float cy{ center.y };
	float theta = 2.f * M_PI / float(num_segments);
	float c = cos(theta); //precalculate the sine and cosine
	float s = sin(theta);
	float x = sphere.radius;
	float y = 0.0f;
	float t = x;
	//	glMatrixMode(GL_MODELVIEW);
	//	glPushMatrix();
	//	glLoadIdentity();
	//	glTranslatef(ball.pos.x, ball.pos.y, ball.pos.z);
	//	glRotatef(ball.rotation, ball.rotationAxis.x, ball.rotationAxis.y, ball.rotationAxis.z);
	glColor3ub(color.x, color.y, color.z);
	glBegin(GL_LINE_LOOP);
	for (size_t i = 0; i < num_segments; i++)
	{
		if (i % 2)
			glColor3ub(color.x, color.y, color.z);
		else
			glColor3ub(10, 250, 250);
		glVertex2f(x + cx, y + cy);
		//apply the rotation matrix
		t = x;
		x = c * x - s * y;
		y = s * t + c * y;
	}
	glEnd();
	//	glPopMatrix();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void DrawSolidCircle(Sphere const& sphere, float3D color, float3D pos = float3D{})
{
	float theta = 2.f * M_PI / float(num_segments);
	float c{ cos(theta) };//precalculate the sine and cosine
	float s{ sin(theta) };
	float3D center{ pos + sphere.center };
	float cx{ center.x };
	float cy{ center.y };
	glColor3ub(color.x, color.y, color.z);
	float x = sphere.radius;
	float y = 0.0f;
	float t{ x };
	glBegin(GL_TRIANGLE_FAN);
	glVertex2d(cx, cy);
	for (size_t i = 0; i < num_segments; i++)
	{
		glColor3ub(10, 10, 250);
		glVertex2f(x + cx, y + cy);//output vertex 
		//apply the rotation matrix
		t = x;
		x = c * x - s * y;
		y = s * t + c * y;
	}

	glVertex2f(sphere.radius + cx, cy);
	glEnd();
}
//////////////////////////////////////////////////////////////
void DrawAABB(MyPolygon& poly)
{
	glLineWidth(3);
	glPointSize(7);
	glColor3ub(0, 0, 123);
	float3D bottomLeft = poly.pos - poly.AABB_Extent;
	float3D topRight = poly.pos + poly.AABB_Extent;
	float3D bottomRight = { poly.pos.x + poly.AABB_Extent.x,
						   poly.pos.y - poly.AABB_Extent.y,
							poly.pos.z };
	float3D topLeft = { poly.pos.x - poly.AABB_Extent.x,
						   poly.pos.y + poly.AABB_Extent.y,
							poly.pos.z };
	glBegin(GL_LINE_LOOP);
	glVertex2d(bottomLeft.x, bottomLeft.y);
	glVertex2d(bottomRight.x, bottomRight.y);
	glVertex2d(topRight.x, topRight.y);
	glVertex2d(topLeft.x, topLeft.y);
	glEnd();

	glPointSize(7);
	glColor3ub(0, 255, 0);
	glBegin(GL_POINTS);
	float3D pt = poly.pos;
	glVertex2d(pt.x, pt.y);
	glEnd();
}

void DrawOBB(OBB const& obb, float3D const& pos)
{
	glLineWidth(3);
	glPointSize(7);
	glColor3ub(255, 255, 0);
	float3D ext{ obb.extend };
	float3D offset{ pos + obb.center };
	const float3D& a{ obb.orient.getRow(0) };
	const float3D& b{ obb.orient.getRow(1) };
	float3D v1{ offset + a * ext.x + b * ext.y };
	float3D v2{ offset - a * ext.x + b * ext.y };
	float3D v3{ offset - a * ext.x - b * ext.y };
	float3D v4{ offset + a * ext.x - b * ext.y };

	//glBegin(GL_QUADS);
	glBegin(GL_LINE_LOOP);

	glVertex2f(v1.x, v1.y);
	glVertex2f(v2.x, v2.y);
	glVertex2f(v3.x, v3.y);
	glVertex2f(v4.x, v4.y);

	glEnd();
}

void DrawPolygon(MyPolygon& poly)
{
	glColor3ub(poly.color.x, poly.color.y, poly.color.z);
	glLineWidth(3);
	glPointSize(7);
	glColor3ub(0, 0, 255);
	glBegin(GL_LINE_LOOP);
	for (int j = 0; j < poly.nSides; j++)
	{
		float3D pt = poly.pos + poly.vertices[j];
		glVertex2d(pt.x, pt.y);
	}
	glEnd();
	glPointSize(7);
	glColor3ub(0, 255, 0);
	glBegin(GL_POINTS);
	float3D pt = poly.pos;
	glVertex2d(pt.x, pt.y);
	glEnd();
}

//////////////////////////////////////////////////////////
void DrawCollisionGeo(MyPolygon& poly)
{
	glColor3ub(poly.color.x, poly.color.y, poly.color.z);
	switch (CollisionType)
	{
	case eCollisionMode::eSphere:
		DrawCircle(poly.mSphere, poly.pos, poly.pos);
		break;
	case eCollisionMode::eAABB:
		DrawAABB(poly);
		break;
	case eCollisionMode::eOBB:
		DrawOBB(poly.obb, poly.pos);
		break;
	}

	// draw the poly itself
	glBegin(GL_POLYGON);
	//glVertex2d(sPolygons[i].pos.x, sPolygons[i].pos.y);
	for (size_t j = 0; j < poly.nSides; j++)
	{
		float3D pt = poly.pos + poly.vertices[j];
		glVertex2d(pt.x, pt.y);
	}
	glEnd();

}
//////////////////////////////////////////////////////////////////////////////////////////////

int Menu(void)
{
	int r = eIdle, key;
	unsigned oldPolyCount = PolyCount;
	while (r != eStop && r != eStart)
	{
		FsPollDevice();
		key = FsInkey();
		switch (key)
		{
		case FSKEY_G:
			r = eStart;
			break;
		case FSKEY_ESC:
			r = eStop;
			break;
		case FSKEY_UP:
			iSpeed += 20.f;
			break;
		case FSKEY_DOWN:
			iSpeed = max(5.f, iSpeed - 20.f);
			break;
		case FSKEY_RIGHT:
			++PolyCount;
			break;
		case FSKEY_LEFT:
			PolyCount = max(0, PolyCount - 1);
			break;
		case FSKEY_S:
			CollisionType = eCollisionMode::eSphere;
			break;
		case FSKEY_A:
			CollisionType = eCollisionMode::eAABB;
			break;
		case FSKEY_O:
			CollisionType = eCollisionMode::eOBB;
			break;
		}

		int wid, hei;
		FsGetWindowSize(wid, hei);

		glViewport(0, 0, wid, hei);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glOrtho(-0.5, (GLdouble)wid - 0.5, (GLdouble)hei - 0.5, -0.5, -1, 1);

		glClearColor(0.0, 0.0, 0.0, 0.0);
		glClear(GL_COLOR_BUFFER_BIT);
		char sSpeed[128];
		sprintf(sSpeed, "Average speed is %f m/s. Use Up/Down keys to change it!\n", iSpeed);
		char sPolyCnt[128];
		sprintf(sPolyCnt, "Polygon count is %d. Use Right/Left keys to change it!\n", PolyCount);
		char sCollision[255];
		sprintf(sCollision, "Collision detection is %s: A for AABB, S for Sphere, O for OBB\n",
			CollisionType == eCollisionMode::eSphere ? "Sphere Collision" :
			CollisionType == eCollisionMode::eAABB ? "AABB Colliion" : "OBB Collision");

		glColor3ub(255, 255, 255);

		glRasterPos2i(32, 32);
		glCallLists(strlen(sSpeed), GL_UNSIGNED_BYTE, sSpeed);
		glRasterPos2i(32, 64);
		glCallLists(strlen(sCollision), GL_UNSIGNED_BYTE, sCollision);
		glRasterPos2i(32, 128);
		glCallLists(strlen(sPolyCnt), GL_UNSIGNED_BYTE, sPolyCnt);

		const char* msg1 = "G.....Start Game\n";
		const char* msg2 = "ESC...Exit";
		glRasterPos2i(32, 192);
		glCallLists(strlen(msg1), GL_UNSIGNED_BYTE, msg1);
		glRasterPos2i(32, 224);
		glCallLists(strlen(msg2), GL_UNSIGNED_BYTE, msg2);

		FsSwapBuffers();
		FsSleep(10);
	}

	// create balls and polys
	if (r == eStart)
	{
		CollisionDataList.clear();
		CollisionDataList.reserve(PolyCount);

		sPolygons.clear();
		sPolygons.reserve(PolyCount);
	}
	return r;
}


//////////////////////////////////////////////////////////////
void spheresCollisionCheck()
{
	for_each(sPolygons.begin(), sPolygons.end(), [&](auto& p) {p->Colliding = false; });
	CollisionDataList.clear();
	for (int i = 0; i < sPolygons.size(); i++)
	{
		MyPolygon& poly1 = *sPolygons[i];
		for (int j = i + 1; j < sPolygons.size(); j++)
		{
			MyPolygon& poly2 = *sPolygons[j];
			if (poly1.distSq(poly2) < (poly1.mSphere.radius + poly2.mSphere.radius) * (poly1.mSphere.radius + poly2.mSphere.radius))
			{
				CollisionDataList.push_back(make_shared<CollisionData>(sPolygons[i], sPolygons[j]));
				float3D mtd{ poly2.mSphere.center + poly2.pos - (poly1.mSphere.center + poly1.pos) }; // Calculate distance between the two spheres based on their center position and external points
				mtd = mtd / mtd.Length();
				CollisionDataList.back()->MTD = (poly1.mSphere.radius + poly2.mSphere.radius - (float)((poly1.mSphere.center - poly2.mSphere.center)).Length()) * mtd;
				poly1.Colliding = poly2.Colliding = true;
			}
		}
	}
}

float GetSAToverlap(float3D const& direction, MyPolygon const& poly1, MyPolygon const& poly2)
{
	// Implementation of SAT to compute overlapping length between projections in the referenced direction (axis)

	// Project and store the minimum and maximum values of all the vertices of the first polygon in the current axis
	// This is the calculation of A . Pi where i = vertices of the polygon
	float low1 = Dot(direction, poly1.vertices[0] + poly1.pos); // Initial projection of the first vertex. We assign the first values we will then compare
	float high1 = low1;
	size_t size{ poly1.nSides };
	for (int i = 1; i < size; ++i)
	{
		float proj{ Dot(direction, poly1.vertices[i] + poly1.pos) }; // Project each vertex and then compare the result to store the max and min values
		low1 = std::min<float>(low1, proj);   // Store minimum value
		high1 = std::max<float>(high1, proj); // Store maximum value
	}

	// Project and store the minimum and maximum values of all the vertices of the first polygon in the current axis
	float  low2 = Dot(direction, poly2.vertices[0] + poly2.pos);
	float high2 = low2;
	size = poly2.nSides;
	for (int i = 1; i < size; ++i)
	{
		float proj{ Dot(direction, poly2.vertices[i] + poly2.pos) };
		low2 = std::min<float>(low2, proj);   // Store minimum value
		high2 = std::max<float>(high2, proj); // Store maximum value
	}

	// After computing and obtaining the maximum and minimum values of the projections in the current axis, we check for overlaps
	if (high1 <= low2 || high2 <= low1)
		return 0.f; // no overlap
	if (high2 > low1)
		return high2 - low1; // Returns overlap
	return high1 - low2; // Returns overlap

}

float3D OBBSATCollisionCheck(MyPolygon const& poly1, MyPolygon const& poly2)
{
	// Helper function we will use to see if there is any overlap using the SAT method and return the MTD
	// If there is overlap, we return the MTD vector, otherwise we return a zero length MTD
	// https://dyn4j.org/2010/01/sat/ used as reference for this code
	// Pulled from previous assignment where we were using self generated OBB to make checks

	// First we will find the 4 directions available to check our projections against when looking for overlaps
	float3D directions[4]{};
	directions[0] = { poly1.vertices[1] - poly1.vertices[0] }; // Get 2 edge vectors per shape as this will suffice to obtain perpendicular vectors used to project
	directions[1] = { poly1.vertices[2] - poly1.vertices[1] }; // Get 2 edge vectors per shape as this will suffice to obtain perpendicular vectors used to project
	directions[2] = { poly2.vertices[1] - poly2.vertices[0] }; // Get 2 edge vectors per shape as this will suffice to obtain perpendicular vectors used to project
	directions[3] = { poly2.vertices[2] - poly2.vertices[0] }; // Get 2 edge vectors per shape as this will suffice to obtain perpendicular vectors used to project

	// Normalize vectors to obtain perpendicular ones and use those for projecting our shapes and test for collision
	directions[0].normalize(); directions[1].normalize(); directions[2].normalize(); directions[3].normalize();

	// Now we project polys on each perpendicular vector, one by one, and check if they overlap. If they do, return true
	float overlap{ FLT_MAX }; // initialize overlap to max value so we can compare for the lowest one
	int index{}; // Keep track of which direction has the smallest gap
	for (int i = 0; i < 4; ++i)
	{
		float cur{ GetSAToverlap(directions[i], poly1, poly2) }; // Offset value we obtain based on the overlap
		if (cur < FLT_EPSILON)
			return float3D{}; // There was no overlap
		if (overlap > cur)
		{
			index = i;
			overlap = cur; // We found a smaller gap, so we assign it as the new overlap
		}
	}

	return -directions[index] * overlap; // This is the MTD we obtain 
}

// This is called when collision type is OBB
void obbCollisionCheck()
{
	for_each(sPolygons.begin(), sPolygons.end(), [&](auto& p) {p->Colliding = false; });
	CollisionDataList.clear();
	for (int i = 0; i < sPolygons.size(); i++)
	{
		MyPolygon& poly1 = *sPolygons[i];
		for (int j = i + 1; j < sPolygons.size(); j++)
		{
			MyPolygon& poly2 = *sPolygons[j];
			float3D mtd{ OBBSATCollisionCheck(poly1,poly2) };
			if (mtd.LengthSq() > FLT_EPSILON) // 
			{
				CollisionDataList.push_back(make_shared<CollisionData>(sPolygons[i], sPolygons[j]));
				CollisionDataList.back()->MTD = mtd;
				poly1.Colliding = poly2.Colliding = true;
			}
		}
	}
}

//////////////////////////////////////////////////////////////////
void checkEdgeCollision()
{
	/////////////////////check polygons edge collision ////////////////////////////////////////
	for (auto& poly : sPolygons)
	{
		if (poly->pos.x < poly->radius && poly->vel.x < 0) //checking left wall
		{
			poly->vel.x = -poly->vel.x;
		}
		if (poly->pos.y < poly->radius && poly->vel.y < 0) // checking top wall
		{
			poly->vel.y = -poly->vel.y;
		}
		if (poly->pos.x > (WinWidth - poly->radius) && poly->vel.x > 0) // checking right wall
		{
			poly->vel.x = -poly->vel.x;
		}
		if (poly->pos.y > (WinHeight - poly->radius) && poly->vel.y > 0) // check bottom wall
		{
			poly->vel.y = -poly->vel.y;
		}
	}

	////////////// check polygon collision /////////////////////////

}

/////////////////////////////////////////////////////////////////////
void updatePhysics(double timeInc)
{
	////////////first update Polys positions //////////////////
	for_each(sPolygons.begin(), sPolygons.end(), [timeInc](shared_ptr<MyPolygon>& obj) {
		obj->pos += obj->vel * timeInc;
		//obj->rotation += obj->rotationSpeed * timeInc;
		});

	///// next check collisions ///////////////////////
	switch (CollisionType)
	{
	case eCollisionMode::eSphere:
		spheresCollisionCheck();
		break;
	case eCollisionMode::eAABB:
		//aabbCollisionCheck();
		break;
	case eCollisionMode::eOBB:
		obbCollisionCheck();
		break;
	}
}

//////////////////////////////////////////////////////
void renderScene()
{
	////// render polygons ///////////////
	for (auto& poly : sPolygons)
	{
		if ((*poly).Colliding)
			DrawCollisionGeo(*poly);

		DrawPolygon(*poly);
	}
	////  swap //////////
	FsSwapBuffers();
}

//////////////////////////////////////////////////////////////
void initPhysics()
{
	int width = 0, height = 0;
	std::srand(std::time(0)); /* seed random number generator */

	FsGetWindowSize(width, height);

	int xdist = width / PolyCount;
	int ydist = height / PolyCount;

	// code to generate general polygons
	for (int i = 0; i < PolyCount; ++i)
	{
		float rad = radius * (1. + float(std::rand() % PolyCount) / float(PolyCount));
		float cX = width / 2 + (i - PolyCount / 2) * std::rand() % xdist;
		float cY = height / 2 + (i - PolyCount / 2) * std::rand() % ydist;
		unsigned nSides = std::rand() % 5 + 3;
		float angle = float(std::rand() % 360) / 180.f * M_PI;
		float speed = iSpeed * (1.f + float(std::rand() % PolyCount) / float(PolyCount));
		float vx = speed * cos(angle);
		float vy = speed * sin(angle);
		sPolygons.push_back(make_shared<MyPolygon>(float3D(cX, cY), float3D(vx, vy), nSides));
		printf("Polygon: nSides=%d. Center(%f, %f)\n", nSides, cX, cY);
		float curRadius{ 2.f * rad };
		for (int j = 0; j < nSides; j++)
		{
			float3D side(curRadius * cos(angle), curRadius * sin(angle));
			sPolygons[i]->setVertex(j, side);
			float angleInc = std::rand() % (360 / (sPolygons.back()->nSides - 1));
			angle += angleInc * M_PI / 180.f;
		}
		sPolygons.back()->color = { (unsigned)std::rand() % 250, (unsigned)std::rand() % 250, (unsigned)std::rand() % 250 };
		sPolygons.back()->radius = curRadius;
		sPolygons.back()->SetSphere();
		sPolygons.back()->computePrinicipleAxis();
	}
}

//////////////////////////////////////////////////////////////////////////////
int Game(void)
{
	DWORD passedTime = 0;
	FsPassedTime(true);

	int width = 0, height = 0;
	FsGetWindowSize(width, height);
	WinWidth = width;
	WinHeight = height;
	//////////// setting up the scene ////////////////////////////////////////	
	initPhysics();

	glViewport(0, 0, WinWidth, WinHeight);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-0.5, (GLdouble)WinWidth - 0.5, (GLdouble)WinHeight - 0.5, -0.5, -1, 1);

	glClearColor(1.0, 1.0, 1.0, 1.0);
	glClear(GL_COLOR_BUFFER_BIT);

	////////////////////// main simulation loop //////////////////////////
	while (1)
	{
		glClear(GL_COLOR_BUFFER_BIT);
		int lb, mb, rb, mx, my;

		FsPollDevice();
		FsGetMouseState(lb, mb, rb, mx, my);
		int key = FsInkey();
		if (key == FSKEY_ESC)
			break;

		renderScene();

		checkEdgeCollision();

		////// update time lapse /////////////////
		passedTime = FsPassedTime();
		int timediff = timeSpan - passedTime;

		/////////// update physics /////////////////
		int maxPossible_dt = 2;
		int numOfIterations = timediff / maxPossible_dt + 1;	// Calculate Number Of Iterations To Be Made At This Update Depending On maxPossible_dt And dt
		double inc = (double)(timediff) / (double)numOfIterations * 0.001;
		for (int i = 0; i < numOfIterations; i++)
			updatePhysics(inc);

		//	printf("\inc=%f, numOfIterations=%d, timediff=%d", inc, numOfIterations, timediff);
		while (timediff >= timeSpan / 3)
		{
			FsSleep(1);
			passedTime = FsPassedTime();
			timediff = timeSpan - passedTime;
			//	printf("--passedTime=%d, timediff=%d", passedTime, timediff);
		}
		passedTime = FsPassedTime(true);
	}
	return 0;
}

////////////////////////////////////////////////////////////////
void GameOver(int score)
{
	int r = 0;

	FsPollDevice();
	while (FsInkey() != 0)
	{
		FsPollDevice();
	}

	while (FsInkey() == 0)
	{
		FsPollDevice();

		int wid, hei;
		FsGetWindowSize(wid, hei);

		glViewport(0, 0, wid, hei);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glOrtho(0, (float)wid - 1, (float)hei - 1, 0, -1, 1);

		glClearColor(0.0, 0.0, 0.0, 0.0);
		glClear(GL_COLOR_BUFFER_BIT);

		const char* msg1 = "Game Over";
		char msg2[256];
		glColor3ub(255, 255, 255);
		glRasterPos2i(32, 32);
		glCallLists(strlen(msg1), GL_UNSIGNED_BYTE, msg1);

		sprintf(msg2, "Your score is %d", score);

		glRasterPos2i(32, 48);
		glCallLists(strlen(msg2), GL_UNSIGNED_BYTE, msg2);

		FsSwapBuffers();
		FsSleep(10);
	}
}

/////////////////////////////////////////////////////////////////////////////////////////
int main(void)
{
	int menu;
	FsOpenWindow(32, 32, WinWidth, WinHeight, 1); // 800x600 pixels, useDoubleBuffer=1

	int listBase;
	listBase = glGenLists(256);
	YsGlUseFontBitmap8x12(listBase);
	glListBase(listBase);

	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);
	glDepthFunc(GL_ALWAYS);

	while (1)
	{
		menu = Menu();
		if (menu == eStart)
		{
			int score;
			score = Game();
			GameOver(score);
		}
		else if (menu == eStop)
		{
			break;
		}
	}

	return 0;
}