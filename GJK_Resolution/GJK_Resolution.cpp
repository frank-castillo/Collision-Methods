#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <ctime>
#include <assert.h>
#include <algorithm>
#include <iostream>
#include <vector>
#include <array>
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

float e{ 1 }; // coefficient of restitution

using namespace std;
using std::min;

typedef enum
{
	eIdle = -2,
	eStop = -1,
	eStart = 0,
	eSpeedUp,
	eSpeedDown,
} changeType;

constexpr float PI{ 3.1415926f };
float iSpeed{ 70.0 };
float radius{ 20. };
size_t num_segments{ 20 };
int PolyCount{ 5 };  // number of polygons to launch.
bool doResolve{ false };
const unsigned frameRate = 30; // 30 frame per second is the frame rate.
const int timeSpan = 1000 / frameRate; // milliseconds
const double timeInc = (double)timeSpan * 0.001; // time increment in seconds

size_t WinWidth{ 800 }, WinHeight{ 600 };

////////////////////////////////////////////////////////

// basically a n-sided polygons. 
struct MyPolygon
{
	float3D pos, vel;
	float3D centre;
	size_t nVertices;
	float m; // used as replacement for mass
	float radius{}; // a rough radius used for rough collision with edges
	bool colliding{};
	Vector3d<unsigned> color;
	std::vector<float3D> vertices;

	void setVertex(unsigned n, float3D v)
	{
		assert(n < nVertices);
		vertices.push_back(v);
	}

	MyPolygon(float3D p, float3D v, unsigned n) : pos(p), vel(v), nVertices(n)
	{
		vertices.reserve(nVertices);
	}

	~MyPolygon()
	{
		vertices.clear();
	}
	// use triangle fan method, or any other method of your choice, to compute the area of the polygon
	void computeArea()
	{
		// Code taken from: https://www.geeksforgeeks.org/area-of-a-polygon-with-given-n-ordered-vertices/
		float area = 0.0;

		// Calculate value of shoelace formula
		// j is previous vertex to i
		int j = nVertices - 1;

		for (int i = 0; i < nVertices; i++)
		{
			area += (vertices[j].x + vertices[i].x) * (vertices[j].y - vertices[i].y);
			j = i;
		}

		m = abs(area / 2.0);
	}

	float3D FindFurthestPoint(float3D direction)
	{
		float3D maxPoint;
		float maxDistance = -FLT_MAX;

		for (auto i = 0; i < nVertices; ++i)
		{
			float distance = Dot(GetVertexWorldPos(i), direction);
			if (distance > maxDistance)
			{
				maxDistance = distance;
				maxPoint = GetVertexWorldPos(i);
			}
		}

		return maxPoint;
	}

	void computePolygonCentroid()
	{
		float3D centroid = { 0, 0 };
		double signedArea = 0.0;
		double x0 = 0.0; // Current vertex X
		double y0 = 0.0; // Current vertex Y
		double x1 = 0.0; // Next vertex X
		double y1 = 0.0; // Next vertex Y
		double a = 0.0;  // Partial signed area

		int lastdex = nVertices - 1;
		const float3D* prev = &(GetVertexWorldPos(lastdex));
		const float3D* next;

		// For all vertices in a loop
		for (int i = 0; i < nVertices; ++i)
		{
			next = &(GetVertexWorldPos(i));
			x0 = prev->x;
			y0 = prev->y;
			x1 = next->x;
			y1 = next->y;
			a = x0 * y1 - x1 * y0;
			signedArea += a;
			centroid.x += (x0 + x1) * a;
			centroid.y += (y0 + y1) * a;
			prev = next;
		}

		signedArea *= 0.5;
		centroid.x /= (6.0 * signedArea);
		centroid.y /= (6.0 * signedArea);

		centre = centroid;
	}

	float3D GetVertexWorldPos(int index)
	{
		return vertices[index] + pos;
	}
};
vector<shared_ptr<MyPolygon>> sPolygons;

///////////////////// collision stuff ////////////////////
struct CollisionData
{
	CollisionData(shared_ptr<MyPolygon> a, shared_ptr<MyPolygon> b) {
		pair[0] = a;
		pair[1] = b;
	}
	float3D pt, normal;
	float penetration;
	shared_ptr<MyPolygon> pair[2];
};
vector<shared_ptr<CollisionData>> CollisionDataList;

//////////////////////////////////////////////////////////////
struct Simplex
{
private:
	array<float3D, 4> m_points;
	unsigned m_size;

public:
	Simplex()
		:m_points({})
		, m_size(0)
	{}

	Simplex& operator=(std::initializer_list<float3D> list)
	{
		for (auto v = list.begin(); v != list.end(); v++)
		{
			m_points[distance(list.begin(), v)] = *v;
		}

		m_size = list.size();
		return *this;
	}

	void push_front(float3D point)
	{
		m_points = { point, m_points[0], m_points[1], m_points[2] };
		m_size = min(m_size + 1, 4u);
	}

	float3D& operator[](unsigned i) { return m_points[i]; }
	unsigned size() const { return m_size; }

	auto begin() const { return m_points.begin(); }
	auto end() const { return m_points.end() - (4 - m_size); }

	vector<float3D> pointsVector;
};
//////////////////////////////////////////////////////////////

void DrawPolygon(MyPolygon& poly)
{
	//	glColor3ub(sPolygons[i].colorx, sBalls[i].colory, sBalls[i].colorz);
	glLineWidth(3);
	glPointSize(7);
	glColor3ub(0, 0, 255);
	glBegin(GL_LINE_LOOP);
	for (int j = 0; j < poly.nVertices; j++)
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
void DrawFilledPolygon(MyPolygon const& poly)
{
	glColor3ub(poly.color.x, poly.color.y, poly.color.z);
	glBegin(GL_POLYGON);
	//glVertex2d(sPolygons[i].pos.x, sPolygons[i].pos.y);
	for (size_t j = 0; j < poly.nVertices; j++)
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
			iSpeed++;
			break;
		case FSKEY_DOWN:
			iSpeed = max(5.f, iSpeed - 1.f);
			break;
		case FSKEY_LEFT:
			e = max(e - 0.1f, 0.000001f);
			break;
		case FSKEY_RIGHT:
			e = min(e + 0.1f, 1.0f);
			break;
		case FSKEY_PAGEUP:
			PolyCount = min(25, PolyCount + 1);
			break;
		case FSKEY_PAGEDOWN:
			PolyCount = max(0, PolyCount - 1);
			break;
		case FSKEY_R:
			doResolve = !doResolve;
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
		sprintf(sSpeed, "Average Polygon's speed is %f m/s. Use Up/Down keys to change it!\n", iSpeed);
		char sPolyCnt[128];
		sprintf(sPolyCnt, "Polygons count is %d. Use Page Up/Page Down keys to change it!\n", PolyCount);
		char sCoefficientE[128];
		sprintf(sCoefficientE, "Coefficient of Restitution is %f. Use Left / Right keys to change it!\n", e);

		char resolveMsg[128];
		sprintf_s(resolveMsg, "Resolving is enabled?(Press 'R' to switch on or off) %s.", doResolve ? "Yes" : "No");

		glColor3ub(255, 255, 255);

		glRasterPos2i(32, 32);
		glCallLists(strlen(sSpeed), GL_UNSIGNED_BYTE, sSpeed);
		glRasterPos2i(32, 64);
		glCallLists(strlen(sPolyCnt), GL_UNSIGNED_BYTE, sPolyCnt);
		glRasterPos2i(32, 96);
		glCallLists(strlen(sCoefficientE), GL_UNSIGNED_BYTE, sCoefficientE);
		glRasterPos2i(32, 128);
		glCallLists(strlen(resolveMsg), GL_UNSIGNED_BYTE, resolveMsg);

		const char* msg1 = "G.....Start Game\n";
		const char* msg2 = "ESC...Exit";
		glRasterPos2i(32, 240);
		glCallLists(strlen(msg1), GL_UNSIGNED_BYTE, msg1);
		glRasterPos2i(32, 260);
		glCallLists(strlen(msg2), GL_UNSIGNED_BYTE, msg2);

		FsSwapBuffers();
		FsSleep(10);
	}

	// create  polys
	if (r == eStart)
	{
		CollisionDataList.clear();
		CollisionDataList.reserve(PolyCount);

		sPolygons.clear();
		sPolygons.reserve(PolyCount);
	}
	return r;
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
}
//////////////////////////////////////////////////////////////////

float3D Support(MyPolygon& poly01, MyPolygon& poly02, float3D direction)
{
	return poly01.FindFurthestPoint(direction) - poly02.FindFurthestPoint(-direction);
}

bool SameDirection(const float3D& direction, const float3D& ao)
{
	return Dot(direction, ao) > 0;
}

float3D TripleProduct(float3D& a, float3D& b, float3D& c)
{
	float3D doubleCross = Cross(Cross(a, b), c);
	return float3D({ doubleCross.x, doubleCross.y, 0.0f });
}

bool ContainsOrigin(Simplex& simplex, float3D& direction)
{
	if (simplex.pointsVector.size() == 3)
	{
		float3D a = simplex.pointsVector[0];
		float3D b = simplex.pointsVector[1];
		float3D c = simplex.pointsVector[2];

		// Compute the edges
		float3D ab = b - a;
		float3D ac = c - a;
		float3D ao = -a;

		// Compute normals
		float3D abf = TripleProduct(ac, ab, ab);
		float3D acf = TripleProduct(ab, ac, ac);

		// is the origin in R4
		if (SameDirection(abf, ao)) {
			// remove point c
			simplex.pointsVector.pop_back();
			// set the new direction to abPerp
			direction = abf;
		}
		else
		{
			// is the origin in R3
			if (SameDirection(acf, ao))
			{
				// remove point b
				simplex.pointsVector.erase(simplex.pointsVector.begin() + 1);
				// set the new direction to acPerp
				direction = acf;
			}
			else
			{
				// otherwise we know its in R5 so we can return true
				return true;
			}
		}
	}
	else
	{
		float3D a = simplex.pointsVector[0];
		float3D b = simplex.pointsVector[1];
		float3D ab = b - a;
		float3D ao = -a;

		float3D abPerp = TripleProduct(ab, ao, ab);
		// set the direction to abPerp
		direction = abPerp;
	}

	return false;
}

void gjk_CollisionCheck()
{
	for_each(sPolygons.begin(), sPolygons.end(), [&](auto& p) {p->colliding = false; });
	CollisionDataList.clear();
	for (int i = 0; i < sPolygons.size(); i++)
	{
		MyPolygon& poly1 = *sPolygons[i];

		for (int j = i + 1; j < sPolygons.size(); j++)
		{
			MyPolygon& poly2 = *sPolygons[j];

			// Get initial support point in any direction
			float3D initDist = float3D{ 1,0,0 };
			auto supportPoint = Support(poly1, poly2, initDist);

			// Simplex is an array of points, max count is 4
			Simplex points;
			points.push_front(supportPoint);
			points.pointsVector.push_back(supportPoint);

			// New direction is towards the origin
			float3D direction = -supportPoint;
			float3D vectorDirection = -initDist;

			while (true)
			{
				supportPoint = Support(poly1, poly2, direction);
				points.pointsVector.push_back(supportPoint);

				if (Dot(supportPoint, direction) <= 0)
				{
					break;
				}
				else
				{
					if (ContainsOrigin(points, direction))
					{
						if (sPolygons[i]->colliding == false && sPolygons[j]->colliding == false)
						{
							sPolygons[i]->colliding = true;
							sPolygons[j]->colliding = true;
							CollisionDataList.push_back(make_shared<CollisionData>(sPolygons[i], sPolygons[j]));
						}
						else if (sPolygons[i]->colliding == true && sPolygons[j]->colliding == false)
						{
							sPolygons[j]->colliding = true;
							CollisionDataList.push_back(make_shared<CollisionData>(sPolygons[j], sPolygons[i]));
						}
						else if (sPolygons[i]->colliding == false && sPolygons[j]->colliding == true)
						{
							sPolygons[i]->colliding = true;
							CollisionDataList.push_back(make_shared<CollisionData>(sPolygons[i], sPolygons[j]));
						}
						else if (sPolygons[i]->colliding == true && sPolygons[j]->colliding == true)
						{
							// Do nothing
						}
						break;
					}
				}
			}

			points.pointsVector.clear();
		}
	}
}

void resolve()
{
	for (auto& col : CollisionDataList)
	{
		MyPolygon& obj1 = *(col->pair[0]);
		MyPolygon& obj2 = *(col->pair[1]);

		float3D obj1_InitialVelocity = obj1.vel;
		float3D obj2_InitialVelocity = obj2.vel;
		float obj1_Mass = obj1.m;
		float obj2_Mass = obj2.m;

		float3D v1prime = (((obj1_Mass - (e * obj2_Mass)) * obj1_InitialVelocity) + ((1 + e) * obj2_Mass * obj2_InitialVelocity)) / (obj1_Mass + obj2_Mass);
		float3D v2prime = ((((1 + e) * obj1_Mass * obj1_InitialVelocity) + (obj2_Mass - (e * obj1_Mass)) * obj2_InitialVelocity)) / (obj1_Mass + obj2_Mass);

		obj1.vel = v1prime;
		obj2.vel = v2prime;
	}
}

/////////////////////////////////////////////////////////////////////

void updatePhysics(double timeInc)
{
	////////////first update Polys positions //////////////////
	for (auto i = 0; i < sPolygons.size(); ++i)
	{
		auto displacement{ sPolygons[i]->vel * timeInc };
		sPolygons[i]->pos += displacement;
		sPolygons[i]->colliding = false;
	}

	///// next check collisions ///////////////////////
	// clear collision data first:
	for_each(sPolygons.begin(), sPolygons.end(), [&](auto& p) {p->colliding = false; });
	CollisionDataList.clear();

	gjk_CollisionCheck();
	if (doResolve)
		resolve();
}

//////////////////////////////////////////////////////
void renderScene()
{
	for (uint32_t i = 0; i < sPolygons.size(); ++i)
	{
		if (sPolygons[i]->colliding)
		{
			DrawFilledPolygon(*sPolygons[i]);
		}
		else
		{
			DrawPolygon(*sPolygons[i]);
		}
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

	// bellow we generate OBB's only, so polygon has 4 sides and angle is 90
	int xdist = width / PolyCount;
	int ydist = height / PolyCount;
	for (int i = 0; i < PolyCount; i++)
	{
		float rad = radius * (1. + float(std::rand() % PolyCount) / float(PolyCount));
		float cX = width / 2 + (i - PolyCount / 2) * std::rand() % xdist;
		float cY = height / 2 + (i - PolyCount / 2) * std::rand() % ydist;
		unsigned nSides = std::rand() % 5 + 3;
		float angle = float(std::rand() % 360) / 180. * PI;
		float speed = iSpeed * (1.f + float(std::rand() % (2 * PolyCount)) / float(PolyCount));
		float vx = speed * cos(angle);
		float vy = speed * sin(angle);

		bool isColliding = false;
		do
		{
			isColliding = false;

			auto temp = make_shared<MyPolygon>(float3D(cX, cY), float3D(vx, vy), nSides);
			sPolygons.push_back(temp);

			for (int j = 0; j < nSides; j++)
			{
				float3D side(2. * rad * cos(angle), 2. * rad * sin(angle));
				temp->setVertex(j, side);
				float angleInc = std::rand() % (360 / (sPolygons.back()->nVertices - 1));
				angle += angleInc * PI / 180.f; // forcing internal angles to be 90'. This makes a rectangle
			}
			temp->color = { (unsigned)std::rand() % 250, (unsigned)std::rand() % 250, (unsigned)std::rand() % 250 };
			temp->computeArea();
			temp->computePolygonCentroid();

			gjk_CollisionCheck();

			if (temp->colliding)
			{
				sPolygons.pop_back();
				--i;
			}

		} while (isColliding == true);
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