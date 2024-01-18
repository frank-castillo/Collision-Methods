#include <stdio.h>
#include <string.h>

#define _USE_MATH_DEFINES
#ifdef WIN32
#include <windows.h>
#endif

#include <math.h>
#include <stdlib.h>
#include <ctime>
#include <assert.h>
#include <algorithm>
#include <iostream>
#include <vector>
#include <memory>

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
namespace {
typedef enum
{
	eIdle = -2,
	eStop = -1,
	eStart = 0,
	eSpeedUp,
	eSpeedDown,
} changeType;


float coeff{ 1 }; // coefficient of restitution

constexpr float PI{ 3.1415926f };
float iSpeed{ 70.0 };
float radius{ 20. };
size_t num_segments{ 20 };
size_t PolyCount{ 5 }; // number of polygons to launch.

const unsigned frameRate = 30; // 30 frame per second is the frame rate.
const int timeSpan = 1000 / frameRate; // milliseconds
const double timeInc = (double)timeSpan * 0.001; // time increment in seconds

size_t WinWidth{ 900 }, WinHeight{ 700 };

///////////////////// physics stuff ///////////////////

typedef enum class eCollisionMode
{
	eSphere,
	eAABB,
	eOBB
};

eCollisionMode ColligionGeo = eCollisionMode::eSphere;

struct AABB
{// pos is center of AABB in local coordinate, extent is half length extend in each direction
	float3D center{}, extend{};
	float3D LL() const { return center - extend; }
	float3D UR() const { return center + extend; }
};

struct OBB
{    // center is in local coordinates
	float3D center{}, extend{};  // contains the legnth of each half extent: (a.length(), b.length(), c.length())
	Matrix3 orient{};  // 3 main directions as rows 1, 2, and 3, [ a direction, b direction, c direction]
	bool Colliding{ false };
	OBB() = default;

	void set(float3D c, float3D e, Matrix3 o)
	{
		center = c;
		extend = e;
		orient = o;
	};

	void render(float3D const& pos)
	{
		glLineWidth(3);
		glPointSize(7);
		glColor3ub(255, 255, 0);
		float3D ext{ extend };
		float3D offset{ pos + center };
		const float3D& a{ orient.getRow(0) };
		const float3D& b{ orient.getRow(1) };
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
};

struct Sphere
{
	float3D center{}; // sphere center in local coordinate
	float radius{ -1.f };
};

struct Gun
{
	enum eAim{ left, right, up, down, notSet};
	AABB cannon{};
	float3D dir{};
	Sphere ball{};
	float3D vel{};
	bool triggered{ false };
	eAim aim{ notSet };

	Gun() = default;

	void reset() { vel = {}; aim = Gun::notSet; }

	void set(float3D cen, eAim d, float w=8.f)
	{
		cannon.center = cen;
		aim = d;
		switch (d)
		{
		case left:
			dir.set( -1.f, 0.f, 0.f );
			cannon.extend.set(3.f * w, w, 0.f);
			break;
		case right:
			dir.set(1.f, 0.f, 0.f);
			cannon.extend.set(3.f * w, w, 0.f);
			break;
		case up:
			dir.set(0.f, -1.f, 0.f);
			cannon.extend.set(w, 3.f * w, 0.f);
			break;
		case down:
			dir.set(0.f, 1.f, 0.f);
			cannon.extend.set(w, 3.f*w, 0.f);
			break;
		default:
			cout << "Gun: ERROR!! wrong direction\n";
			exit(1);
		}
		vel = 30.f * dir;
		ball.center = cannon.center;
		ball.radius = w;
	}

	void trigger() {
		ball.center = cannon.center;
		triggered = true;
	}

	void increaseBallRadius() {
		auto v{ vel };
		ball.radius += 3.f;
		set(cannon.center, aim, ball.radius);
		vel = v;
	}
	void decreaseBallRadius() {
		auto v{ vel };
		ball.radius = std::max<float>((ball.radius - 3.f), 8.f);
		set(cannon.center, aim, ball.radius);
		vel = v;
	}

	void update(float timeInc)
	{
		if(triggered)
			ball.center += vel * timeInc;
	}
} projectile;

int minRVert{ -1 }, maxRVert{ -1 }; // to be used for effective sphere using PCA later

struct PhysicsObj
{
	float3D pos, vel; // pos and velocity of the Object in World Coordinates.
	float3D rotationAxis;
	float rotationSpeed;
	//Matrix3 orientation;
	bool colliding;
	AABB mAABB{};
	OBB mOBB{};
	Sphere mSphere{};
	float slantAngle; // rotation angle around orientation axis to orient the object in WCS.
	PhysicsObj(float3D p, float3D v, float rotS = 0.f, float rot = 0.f) :
		pos(p),
		vel(v),
		colliding(false),
		rotationAxis({ 0.f, 0.f, 1.f }),
		rotationSpeed(rotS),
		slantAngle(rot)
	{
	}

	// should be called after PCA already performed by calling computePrinicipleAxis()
	// computing effective sphere collision geo using PCA method
	void setSphere(std::vector<float3D> const& vers)
	{
		//std::cout << "setSphere: to be implemented by students using PCA method from Chapte 8\n";
		assert(minRVert >= 0 && maxRVert >= 0);
		//1-initial setting of center and radius:
		auto cent{ (vers[minRVert] + vers[maxRVert]) / 2.f };
		float rad{ cent.dist(vers[minRVert])};
		//2-refining cent and rad by processing the vertices
		for (int i = 0; i < vers.size(); ++i)
		{
			if (cent.dist(vers[i]) > rad)
			{
				float3D D{ cent - Normal(vers[i] - cent) * rad };
				rad = D.dist(vers[i]) / 2.f;
				cent = (D + vers[i]) / 2.f;
			}
		}

		mSphere.center = cent;
		mSphere.radius = rad;

		// debug code bellow: verify all vertices are inside the sphere:
		for (auto& ver : vers)
		{
			if (mSphere.center.dist(ver) > mSphere.radius)
			{
				cout << "Error: "<< mSphere.center.dist(ver) <<", "<< mSphere.radius << "\n";
			}
		}
	}

	void setAABB(vector<float3D> const& vert, size_t n)
	{
		float3D LL{ FLT_MAX, FLT_MAX }, UR{ FLT_MIN, FLT_MIN };
		for (int i = 0; i < n; ++i)
		{
			LL = minimise(LL, vert[i]);
			UR = maximise(UR, vert[i]);
		}
		mAABB.center = (LL + UR) / 2.f;
		mAABB.extend = (UR - LL) / 2.f;
	}
};


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
////////////////////////////////////////////////////////

// n sided polygons. It has a center and n vertices around it at average distance d from the center.
struct Polygon : PhysicsObj
{ // center position for a polygon is center of its OBB 
	size_t nSides;
	Vector3d<unsigned> color;
	std::vector<float3D> vertices;
	void setVertex(unsigned n, float3D v) {
		assert(n < nSides);
		vertices.push_back(v);
	}

	void setAABB()
	{
		PhysicsObj::setAABB(vertices, nSides);
	}

	Polygon(float3D c, float3D v, unsigned n, float rotS = 0.f, float rot = 0.f) : PhysicsObj(c, v, 0., rotS), nSides(n)
	{
		slantAngle = { rot };
		vertices.clear();
		vertices.reserve(nSides);
	}

	// used to construct OBB for our polygon
	void computePrinicipleAxis()
	{
		// calculate the average:
		float3D meanPosition{};
		std::for_each(vertices.begin(), vertices.end(), [&](auto& p) {meanPosition += p; });
		std::cout << "m:(" << meanPosition.x << ", " << meanPosition.y << std::endl;
		meanPosition = { meanPosition / nSides };
			
		//calculate covariance matrix:
		Matrix3 covarianceMatrix{ Matrix3::zero() };
		// compute covariance matrix here:
		float m11{}, m12{}, m13{}, m22{}, m23{}, m33{};
		for (auto const& pt : vertices)
		{
			m11 += (pt.x - meanPosition.x) * (pt.x - meanPosition.x);
			m12 += (pt.x - meanPosition.x) * (pt.y - meanPosition.y);
			m13 += (pt.x - meanPosition.x) * (pt.z - meanPosition.z);

			m22 += (pt.y - meanPosition.y) * (pt.y - meanPosition.y);
			m23 += (pt.y - meanPosition.y) * (pt.z - meanPosition.z);

			m33 += (pt.z - meanPosition.z) * (pt.z - meanPosition.z);
		}
		const float nSides{ static_cast<float>(vertices.size()) };
		float3D row1 = float3D(m11, m12, m13) / nSides;
		covarianceMatrix.setRow(0, row1);
		float3D row2 = float3D(m12, m22, m23) / nSides;
		covarianceMatrix.setRow(1, row2);
		float3D row3 = float3D(m13, m23, m33) / nSides;
		covarianceMatrix.setRow(2, row3);


		double covMatrixArray[3][3] =
		{
			{covarianceMatrix.getRow(0).x, covarianceMatrix.getRow(0).y, covarianceMatrix.getRow(0).z },
			{covarianceMatrix.getRow(1).x, covarianceMatrix.getRow(1).y, covarianceMatrix.getRow(1).z },
			{covarianceMatrix.getRow(2).x, covarianceMatrix.getRow(2).y, covarianceMatrix.getRow(2).z },
		};

		double eigenVecs[3][3]{};
		double eigenValues[3]{};
		eigen_decomposition(covMatrixArray, eigenVecs, eigenValues);
		Matrix3 orient{};
		Matrix3 eigenVectors{};
		for (uint32_t i = 0; i < 3; ++i)
			eigenVectors.setRow(i, float3D(eigenVecs[i][0], eigenVecs[i][1], eigenVecs[i][2]));

		{
			float3D T = eigenVectors.getCol(0); //T
			float3D S = eigenVectors.getCol(1); //S
			float3D R = eigenVectors.getCol(2); //R
			assert(eigenValues[2] >= eigenValues[1]);
			assert(eigenValues[2] >= eigenValues[0]);
			assert(eigenValues[1] >= eigenValues[0]);
			orient.setRow(0, R);
			orient.setRow(1, S);
			orient.setRow(2, T);
			float3D testR = covarianceMatrix * R;
			float3D vR = R * eigenValues[2];
			assert((vR - testR).LengthSq() < 0.001);
			float3D testS = covarianceMatrix * S;
			float3D vS = S * eigenValues[1];
			assert((vS - testS).LengthSq() < 0.001);
			float3D testT = covarianceMatrix * T;
			float3D vT = T * eigenValues[0];
			//assert((vT - testT).LengthSq() < 0.001);
		}
		float minExtendR{}, maxExtendR{};
		float minExtendS{}, maxExtendS{};
		float minExtendT{}, maxExtendT{};
		minExtendR = maxExtendR = Dot(vertices[0], orient.getRow(0));
		minExtendS = maxExtendS = Dot(vertices[0], orient.getRow(1));
		minExtendT = maxExtendT = Dot(vertices[0], orient.getRow(2));
		minRVert = maxRVert = 0;
		for (uint32_t i = 1; i < nSides; ++i)
		{
			// the following is needed to record the extreme vertex index along R for effective sphere computation later
			auto minExtendR1 = min(minExtendR, Dot(vertices[i], orient.getRow(0)));
			auto maxExtendR1 = max(maxExtendR, Dot(vertices[i], orient.getRow(0)));
			if (minExtendR1 < minExtendR)
			{
				minExtendR = minExtendR1;
				minRVert = i;
			}
			if (maxExtendR1 < maxExtendR)
			{
				maxExtendR = maxExtendR1;
				maxRVert = i;
			}

			minExtendS = min(minExtendS, Dot(vertices[i], orient.getRow(1)));
			maxExtendS = max(maxExtendS, Dot(vertices[i], orient.getRow(1)));

			minExtendT = min(minExtendT, Dot(vertices[i], orient.getRow(2)));
			maxExtendT = max(maxExtendT, Dot(vertices[i], orient.getRow(2)));
		}

		float3D extend = { (maxExtendR - minExtendR) * 0.5f , (maxExtendS - minExtendS) * 0.5f, (maxExtendT - minExtendT) * 0.5f };
		float3D covMean{ (maxExtendR + minExtendR) * 0.5f , (maxExtendS + minExtendS) * 0.5f , (maxExtendT + minExtendT) * 0.5f };
		float3D covCenter = (covMean.x * orient.getRow(0)) + (covMean.y * orient.getRow(1)) + (covMean.z * orient.getRow(2));
		mOBB.set(covCenter, extend, orient);
	}
	~Polygon()
	{
		vertices.clear();
	}
};
vector<shared_ptr<Polygon> > sPolygons;

//////////////////////////////////////////////////////////////////////////////////////
void DrawCircle(Sphere const& sp, float3D pos)
{
	float3D cen{ pos + sp.center }; // cen in global coordinate system
	float cx{ cen.x};
	float cy{ cen.y };
	float theta = 2.f * M_PI / float(num_segments);
	float c = cos(theta);//precalculate the sine and cosine
	float s = sin(theta);
	float x = sp.radius;
	float y = 0.f;
	float t = x;

	glColor3ub(0, 0, 123);
	glBegin(GL_LINE_LOOP);
	for (size_t i = 0; i < num_segments; i++)
	{
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
void DrawSolidCircle(Sphere const& sp, float3D col, float3D pos = float3D{})
{
	float3D cen{ pos + sp.center }; // cen in global coordinate system
	float cx{ cen.x };
	float cy{ cen.y };
	float theta = 2.f * M_PI / float(num_segments);
	float c{ cos(theta) };//precalculate the sine and cosine
	float s{ sin(theta) };
	glColor3ub(col.x, col.y, col.z);
	float x = sp.radius;
	float y = 0.f;
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

	glVertex2f(sp.radius + cx, cy);
	glEnd();
}
//////////////////////////////////////////////////////////////
void DrawAABB(AABB const& aabb, float3D pos)
{
	glLineWidth(3);
	glPointSize(7);
	glColor3ub(0, 0, 123);
	float3D ll = pos + aabb.LL();
	float3D ur = pos + aabb.UR();

	glBegin(GL_LINE_LOOP);
		glVertex2d(ll.x, ll.y);
		glVertex2d(ur.x, ll.y);
		glVertex2d(ur.x, ur.y);
		glVertex2d(ll.x, ur.y);
	glEnd();

	glPointSize(7);
	glColor3ub(0, 255, 0);
	// now render the center of AABB
	glBegin(GL_POINTS);
	float3D pt = pos + aabb.center; 
	glVertex2d(pt.x, pt.y);
	glEnd();

}

void DrawOBB(OBB const& obb, float3D pos)
{
	glLineWidth(3);
	glPointSize(7);
	glColor3ub(0, 0, 123);

	float3D offset{ pos + obb.center };
	float3D ext{ obb.extend };
	const float3D& a{ obb.orient.getRow(0) };
	const float3D& b{ obb.orient.getRow(1) };
		
	float3D v1{ offset + a * ext.x + b * ext.y };
	float3D v2{ offset - a * ext.x + b * ext.y };
	float3D v3{ offset - a * ext.x - b * ext.y };
	float3D v4{ offset + a * ext.x - b * ext.y };

	glBegin(GL_LINE_LOOP);
	glVertex2f(v1.x, v1.y);
	glVertex2f(v2.x, v2.y);
	glVertex2f(v3.x, v3.y);
	glVertex2f(v4.x, v4.y);
	glEnd();
}

void DrawPolygon(Polygon& poly)
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
void DrawCollisionGeo(Polygon& poly)
{
	glColor3ub(poly.color.x, poly.color.y, poly.color.z);
	switch (ColligionGeo)
	{
	case eCollisionMode::eSphere:
		DrawCircle(poly.mSphere, poly.pos);
		break;
	case eCollisionMode::eAABB:
		DrawAABB(poly.mAABB, poly.pos);
		break;
	case eCollisionMode::eOBB:
		DrawOBB(poly.mOBB, poly.pos);
		break;
	}

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
int cannonId = -1; // index of the active cannon

int Menu(void)
{
	int r = eIdle, key;
	unsigned oldPolyCount = PolyCount;
	projectile.reset();
	char* msg{};
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
			coeff = min(coeff + 0.1f, 1.0f);
			break;
		case FSKEY_LEFT:
			coeff = max(coeff - 0.1f, 0.000001f);
			break;
		case FSKEY_PAGEUP:
			PolyCount = min(25, PolyCount + 1);
			break;
		case FSKEY_PAGEDOWN:
			PolyCount = max(0, PolyCount - 1);
			break;
		case FSKEY_C:
			ColligionGeo = eCollisionMode::eSphere;
			break;
		case FSKEY_A:
			ColligionGeo = eCollisionMode::eAABB;
			break;
		case FSKEY_O:
			ColligionGeo = eCollisionMode::eOBB;
			break;
		case FSKEY_W: // up gun
				projectile.set({ WinWidth * 0.5f, WinHeight - 10.f, 0.f }, Gun::eAim::up);
				msg = " :Bottom gun set";
			
			break;
		case FSKEY_X: // down gun
				projectile.set({ WinWidth * 0.5f, 10.f, 0.f }, Gun::eAim::down);
				msg = " :Top gun set";
			
			break;
		case FSKEY_F: // right gun
				projectile.set({ 10.f, WinHeight * 0.5f, 0.f }, Gun::eAim::right);
				msg = " :Right gun set";
			break;
		case FSKEY_S: // left gun
				projectile.set({ WinWidth - 10.f, WinHeight * 0.5f, 0.f }, Gun::eAim::left);
				msg = " :Left gun set";
			break;
		case FSKEY_PLUS: // increase ball speed
			if(projectile.aim != Gun::notSet) 
				projectile.vel += 2.5f;
			break;
		case FSKEY_MINUS: // decrease ball speed
			if (projectile.aim != Gun::notSet)
				projectile.vel -= 1.5f;
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
		sprintf(sCollision, "Collision detection is %s: A for AABB, C for Sphere, O for OBB\n",
			ColligionGeo == eCollisionMode::eSphere ? "Sphere Collision" :
			ColligionGeo == eCollisionMode::eAABB ? "AABB Colliion" : "OBB Collision");

		glColor3ub(255, 255, 255);

		glRasterPos2i(32, 32);
		glCallLists(strlen(sSpeed), GL_UNSIGNED_BYTE, sSpeed);
		glRasterPos2i(32, 64);
		glCallLists(strlen(sCollision), GL_UNSIGNED_BYTE, sCollision);
		glRasterPos2i(32, 128);
		glCallLists(strlen(sPolyCnt), GL_UNSIGNED_BYTE, sPolyCnt);

		const char* gunW{ "W... add a gun shooting upward" };
		const char* gunX{ "X... add a gun shooting downward" };
		const char* gunF{ "F... add a gun shooting right" };
		const char* gunS{ "S... add a gun shooting " };
		glRasterPos2i(32, 216);
		glCallLists(strlen(gunW), GL_UNSIGNED_BYTE, gunW);
		glRasterPos2i(32, 240);
		glCallLists(strlen(gunX), GL_UNSIGNED_BYTE, gunX);
		glRasterPos2i(32, 264);
		glCallLists(strlen(gunF), GL_UNSIGNED_BYTE, gunF);
		glRasterPos2i(32, 288);
		glCallLists(strlen(gunS), GL_UNSIGNED_BYTE, gunS);

		glRasterPos2i(32, 310);
		if(msg)
			glCallLists(strlen(msg), GL_UNSIGNED_BYTE, msg);


		const char* gSpeedUp{ "+ / - ... increase bullet speed or decrease(Plus or Minus)" };
		glRasterPos2i(32, 340);
		glCallLists(strlen(gSpeedUp), GL_UNSIGNED_BYTE, gSpeedUp);
		const char* gGunRadius{ "P / O ... increase bullet radius or decrease(P or O)" };
		glRasterPos2i(32, 360);
		glCallLists(strlen(gGunRadius), GL_UNSIGNED_BYTE, gGunRadius);

		const char* msg1 = "G.....Start Game. ESC...Exit";
		glRasterPos2i(32, 400);
		glCallLists(strlen(msg1), GL_UNSIGNED_BYTE, msg1);

		FsSwapBuffers();
		FsSleep(10);
	}

	// create polys
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
	for_each(sPolygons.begin(), sPolygons.end(), [&](auto& p) {p->colliding = false; });
	CollisionDataList.clear();
	for (int i = 0; i < sPolygons.size(); i++)
	{
		Polygon& poly1 = *sPolygons[i];
		for (int j = i + 1; j < sPolygons.size(); j++)
		{
			Polygon& poly2 = *sPolygons[j];
			auto totalRad{ (poly1.mSphere.radius + poly2.mSphere.radius) };
			if (poly1.pos.distSq(poly2.pos) < (totalRad  * totalRad))
			{
				CollisionDataList.push_back(make_shared<CollisionData>(sPolygons[i], sPolygons[j]));
				float3D mtd{ poly2.mSphere.center + poly2.pos - (poly1.mSphere.center + poly1.pos) };
				mtd = mtd / mtd.Length();
				CollisionDataList.back()->MTD = (totalRad - (float)((poly1.mSphere.center - poly2.mSphere.center)).Length() )* mtd;
				poly1.colliding = poly2.colliding = true;
			}
		}
	}
}
	
bool aabbCollide(Polygon const& poly1, Polygon const& poly2)
{
	// check if a and b projections on x or on y axis do not overlap:
	auto ll_a{ poly1.mAABB.LL() + poly1.pos };
	auto ll_b{ poly2.mAABB.LL() + poly2.pos };
	auto ur_a{ poly1.mAABB.UR() + poly1.pos };
	auto ur_b{ poly2.mAABB.UR() + poly2.pos };
	return !(ur_a.x < ll_b.x || ur_b.x < ll_a.x || ur_a.y < ll_b.y || ur_b.y < ll_a.y);
}

void aabbCollisionCheck()
{
	for_each(sPolygons.begin(), sPolygons.end(), [&](auto& p) {p->colliding = false; });
	CollisionDataList.clear();
	for (int i = 0; i < sPolygons.size(); i++)
	{
		Polygon& poly1 = *sPolygons[i];
		for (int j = i + 1; j < sPolygons.size(); j++)
		{
			Polygon& poly2 = *sPolygons[j];
			if (aabbCollide(poly1, poly2))
			{
				CollisionDataList.push_back(make_shared<CollisionData>(sPolygons[i], sPolygons[j]));
				// find overlapping AABB:
				AABB ab1{ poly1.mAABB.center + poly1.pos, poly1.mAABB.extend };
				AABB ab2{ poly2.mAABB.center + poly2.pos, poly2.mAABB.extend };
				float3D ll{ maximise(ab1.LL(), ab2.LL()) };
				float3D ur{ minimise(ab1.UR(), ab2.UR()) };
				AABB overlap{ (ll + ur) * 0.5f, (ur - ll) * 0.5f };
				float wid{ overlap.UR().x - overlap.LL().x };
				float high{ overlap.UR().y - overlap.LL().y };
				float3D mtd = (wid < high) ? float3D{wid, 0.f, 0.f} : float3D{0.0f, high, 0.f};
				CollisionDataList.back()->MTD = mtd;
				poly1.colliding = poly2.colliding = true;
			}
		}
	}
}

// usese SAT to compute the overlapping length between the projection of poly1 and poly2 over direction dir
float getSAToverlap(float3D dir, Polygon const& poly1, Polygon const& poly2)
{
	float low1 = Dot(dir, poly1.vertices[0] + poly1.pos);
	float high1 =  low1;
	size_t sz{ poly1.nSides };
	for (int i=1; i<sz; ++i)
	{
		float proj{ Dot(dir, poly1.vertices[i] + poly1.pos) };
		low1 = std::min<float>(low1, proj);
		high1 = std::max<float>(high1, proj);
	}

	float  low2 =  Dot(dir, poly2.vertices[0] + poly2.pos);
	float high2 = low2;
	sz = poly2.nSides;
	for (int i = 1; i < sz; ++i)
	{
		float proj{ Dot(dir, poly2.vertices[i] + poly2.pos) };
		low2 = std::min<float>(low2, proj);
		high2 = std::max<float>(high2, proj);
	}
	if (high1 <= low2 || high2 <= low1)
		return 0.f; // no overlap
	if (high2 > low1)
		return high2 - low1;
	return high1 - low2;
}

// returns MTD for the 2 polys. If no overlap, then it returns zero length MTD
float3D oBBCollideSAT(Polygon const& poly1, Polygon const& poly2)
{
	// get 4 directions to check projections against:
	float4D dirs[4]{};
	dirs[0] = poly1.mOBB.orient.getRow(0);
	dirs[1] = { poly1.vertices[2] - poly1.vertices[1] };
	dirs[2] = { poly2.vertices[1] - poly2.vertices[0] };
	dirs[3] = { poly2.vertices[2] - poly2.vertices[1] };
	dirs[0].normalize(); dirs[1].normalize(); dirs[2].normalize(); dirs[3].normalize();

	// project polys on dirs, one by one, and check if they don't overlap then no collision ==> return false
	float overlap{FLT_MAX};
	int idx{};  
	for (int i = 0; i < 4; ++i)
	{
		float cur{ getSAToverlap(dirs[i], poly1, poly2) };
		if (cur < FLT_EPSILON)
			return float3D{};
		if (overlap > cur)
		{
			idx = i;
			overlap = cur;
		}
	}

	return -dirs[idx] * overlap;  // means they collide
}

// This is called when CollisionType is OBB
void obbCollisionCheck()
{
	for_each(sPolygons.begin(), sPolygons.end(), [&](auto& p) {p->colliding = false; });
	CollisionDataList.clear();
	for (int i = 0; i < sPolygons.size(); i++)
	{
		Polygon& poly1 = *sPolygons[i];
		for (int j = i + 1; j < sPolygons.size(); j++)
		{
			Polygon& poly2 = *sPolygons[j];
			float3D mtd{ oBBCollideSAT(poly1, poly2) };
			if (mtd.LengthSq() > FLT_EPSILON)
			{
				CollisionDataList.push_back(make_shared<CollisionData>(sPolygons[i], sPolygons[j]));
				CollisionDataList.back()->MTD = mtd;
				poly1.colliding = poly2.colliding = true;
			}
		}
	}
}

/*
* Static collision resolution: 1-Separate the colliding pairs; 2-apply resolution method
*/
void collisionResolve()
{
	// NOTE: USE FLAG SET UP PREVOUSLY
	// First separate the colliding pairs, then fix the velocities:
	for (auto &col : CollisionDataList)
	{
		PhysicsObj& obj1 = *(col->pair[0]);
		PhysicsObj& obj2 = *(col->pair[1]);
			
		// separate the 2 obj by half of MTD:
		float3D mtdh{ col->MTD * 0.5f };
		obj1.pos -= mtdh;  
		obj2.pos += mtdh;

		// now fix velocities with respect to normal to MTD.
		// REMEMBER this method is not really correct way of resolving
		float3D normal{ mtdh.x, mtdh.y, 0.f };
		normal = normal / normal.Length();
		obj1.vel -= 2.f * Dot(obj1.vel, normal) * normal;   // using reflection formula vel = vel - 2(vel.n)n;
		obj2.vel -= 2.f * Dot(obj2.vel, normal) * normal;
	}
}

bool spheresCollide(Sphere s1, Sphere s2)
{
	if (s1.center.dist(s2.center) < (s1.radius + s2.radius))
		return true;
	return false;
}
//// 
// apply Continous collision detection and resolution between a Gun's ball and all the flying Polys
// Collision detection and resoultion must be with Poly's sphere.
// For collision detection use the method we covered Continuous Collision detection in class, meaning:
// 1- make the ball stand still by computing relative velocity of a poly A with respect to the ball: rel_vel = A.vel - Ball.vel.
// 2- from here on treat the ball as stationary and poly A moving with velocity rel_vel.
// 3- apply the 3 steps explained in class to check if A is moving towards ball ---> If so, does it get close enough at its closest distance
// on its trajectory to the ball, --> If so, does it actually get close enough to collide in this frame with the ball.
// 4- If the answer of all questions above in 3) are positive, then set colliding flag for both to true.
// We will take care of "Resolution" step next week.
// PAY Attention that the code bellow is doing static collision at the moment, just as we have learnt so far. This method is faulty for 
// fast moving object like the ball. You can test this by increasing the ball speed (use + while ball is in move) to see when the ball vel is
// too high sometimes the collision detection with static method bellow fails to detect the collision.
////
void ContiniousCollisionDetectionAndResolution()
{
	projectile.ball.center += projectile.vel * timeInc;
	/// update cannon balls:
	for (auto& poly : sPolygons)
	{
		Vector3d<float> velocity = projectile.vel - poly->vel;
		float3D tempVelocity = projectile.vel;
		float3D center = poly->pos + poly->mSphere.center;
		Vector3d<float> stationaryObject = projectile.ball.center - center;
		float angle = Dot(stationaryObject, velocity);

		if (angle < 0.0f)
		{
			poly->colliding = false;
			continue;
		}

		float stationaryObjectSize = stationaryObject.Length();
		float dotProductResult = Dot((velocity / velocity.Length()), stationaryObject);
		float thirdSide = stationaryObjectSize * stationaryObjectSize - dotProductResult * dotProductResult;
		float radii = projectile.ball.radius + poly->mSphere.radius;

		if (thirdSide > radii)
		{
			poly->colliding = false;
			continue;
		}

		float hypotenuse = sqrt(radii * radii - thirdSide * thirdSide);
		float collisionPoint = dotProductResult - hypotenuse;

		if (velocity.Length() * timeInc < collisionPoint)
		{
			poly->colliding = false;
			continue;
		}
		else
		{
			poly->colliding = true;
		}
	}
}

/////////////////////////////////////////////////////////////////////
void updatePhysics(double timeInc)
{
	//////////// update Polys positions //////////////////
	for_each(sPolygons.begin(), sPolygons.end(), [timeInc](shared_ptr<Polygon>& obj) {
		obj->pos += obj->vel * timeInc;
		//obj->rotation += obj->rotationSpeed * timeInc;
		});

	///// next check collisions ///////////////////////
	switch (ColligionGeo)
	{
	case eCollisionMode::eSphere:
		spheresCollisionCheck();
		break;
	case eCollisionMode::eAABB:
		aabbCollisionCheck();
		break;
	case eCollisionMode::eOBB:
		obbCollisionCheck();
		break;
	}

	/// do continuous collision detection ball and Polys
	ContiniousCollisionDetectionAndResolution();

	////// here you do collision resolution for polys. /////////////
	//std::cout << "Resolving not done!\n";
	collisionResolve();
}

//////////////////////////////////////////////////////////////////
void checkEdgeCollision()
{
	/////////////////////check polygons edge collision ////////////////////////////////////////
	for (auto& poly : sPolygons)
	{
		if (poly->pos.x < poly->mSphere.radius && poly->vel.x < 0) //checking left wall
		{
			poly->vel.x = -poly->vel.x;
		}
		if (poly->pos.y < poly->mSphere.radius && poly->vel.y < 0) // checking top wall
		{
			poly->vel.y = -poly->vel.y;
		}
		if (poly->pos.x > (WinWidth - poly->mSphere.radius) && poly->vel.x > 0) // checking right wall
		{
			poly->vel.x = -poly->vel.x;
		}
		if (poly->pos.y > (WinHeight - poly->mSphere.radius) && poly->vel.y > 0) // check bottom wall
		{
			poly->vel.y = -poly->vel.y;
		}
	}
}

//////////////////////////////////////////////////////
void renderScene()
{
	////// render polygons ///////////////
	for (auto& poly : sPolygons)
	{
		//		DrawCircle(Ball(poly->pos, poly->vel, poly->radius, poly->rotationSpeed, poly->rotation));
		if (!(*poly).colliding)
			DrawPolygon(*poly);
		else
			DrawCollisionGeo(*poly);
	}

	/// render cannon and the ball
	float3D col{10, 250, 250};
	if (projectile.aim != Gun::notSet)
	{
		DrawAABB(projectile.cannon, float3D{});
		DrawSolidCircle(projectile.ball, col);
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

	for (int i = 0; i < PolyCount; ++i)
	{
		float rad = radius * (1. + float(std::rand() % PolyCount) / float(PolyCount));
		float cX = width / 2 + (i - PolyCount / 2) * std::rand() % xdist;
		float cY = height / 2 + (i - PolyCount / 2) * std::rand() % ydist;
		unsigned nSides = std::rand() % 3 + 3;
		float angle = float(std::rand() % 360) / 180.f * M_PI;
		float speed = iSpeed * (1.f + float(std::rand() % PolyCount) / float(PolyCount));
		float vx = speed * cos(angle);
		float vy = speed * sin(angle);
		sPolygons.push_back(make_shared<Polygon>(float3D(cX, cY), float3D(vx, vy), nSides, 2. * angle / frameRate));
		printf("Polygon: nSides=%d. Center(%f, %f)\n", nSides, cX, cY);
		float curRadius{ 2.f * rad };
		for (int j = 0; j < nSides; j++)
		{
			float3D side(curRadius * cos(angle), curRadius * sin(angle));
			sPolygons[i]->setVertex(j, side);
			float angleInc = std::rand() % (360 / (sPolygons[i]->nSides - 1));
			angle += angleInc * M_PI / 180.f;
		}
		sPolygons[i]->color = { (unsigned)std::rand() % 250, (unsigned)std::rand() % 250, (unsigned)std::rand() % 250 };
		sPolygons[i]->setAABB();
		sPolygons[i]->computePrinicipleAxis(); // use PCA to compute OBB
		sPolygons[i]->setSphere(sPolygons[i]->vertices); // use PCA to compute effective Sphere
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
		switch (key)
		{
		case FSKEY_PLUS:
			if (projectile.aim != Gun::notSet)
				projectile.vel *= 1.5f;
			break;
		case FSKEY_MINUS:
			if (projectile.aim != Gun::notSet)
				projectile.vel /= 1.5f;
			break;
		case FSKEY_TAB: 
			if (projectile.aim != Gun::notSet)
				projectile.trigger();
			break;
		case FSKEY_P:
			if (projectile.aim != Gun::notSet)
				projectile.increaseBallRadius();
			break;
		case FSKEY_O:
			if (projectile.aim != Gun::notSet)
				projectile.decreaseBallRadius();
			break;
		}

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

