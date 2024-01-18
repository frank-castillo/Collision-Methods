#pragma once

#define _USE_MATH_DEFINES
#include <math.h>
#include <algorithm>
#include <random>
#include <assert.h>
#include <map>
#include <iostream>

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

#include "vectors.h"

GLuint theTorus{};
static double timePassed = 0;

typedef enum
{
	eStop = -1,
	eIdle = 0,
	eStart = 1,
	eLinear,
	eLagrange,
	eHermite,
	eCatmulRom,
	eQuadraticBezier,
	eCubicBezier
} changeType;

changeType eInterpolation{ eLinear };
bool		keys[256];			// Array Used For The Keyboard Routine
float timeRange = 10.;    // total flight time in seconds
int WinWidth = 860;
int WinHeight = 720;
float ratio = (float)WinHeight / (float)WinWidth;
float WorldWidth = 50.0; // 50 meter wide
float WorldHeight = WorldWidth * ratio; // 

const float PointSize{ 1.f / 3.f };

struct Point
{
	float3D p;
	float color[3];
	float timeTag;

	Point(float a = 0.f, float b = 0.f, float t = 0.f) :p({ a, b, 0.f }), timeTag(t) {
		color[0] = color[1] = color[2] = 0;
	}

	Point(float3D position, float3D _color, float _timeT)
	{
		color[0] = _color.x;
		color[1] = _color.y;
		color[2] = _color.z;
		timeTag = _timeT;
		p = position;

	}

	Point(const Point& p) = default;

	~Point() = default;

	void toString() { std::cout << "(" << p.x << ", " << p.y << ")--"; }
	void set(float a, float b, float c = 0) { p.x = a; p.y = b; p.z = c; }
	Point  operator+(const Point& rhs)
	{

		Point tmp = Point(*this);
		tmp.p += rhs.p;
		return tmp;
	}
	Point operator-(const Point& rhs)
	{
		Point tmp = Point(*this);
		tmp.p -= rhs.p;
		return tmp;
	}
	Point operator *(float mul)
	{
		Point tmp = Point(*this);
		tmp.p *= mul;
		return tmp;
	}
	Point& operator=(const Point& rhs) = default;

	float length() { return p.Length(); }
	float dist(const Point& pt) { return (p - pt.p).Length(); }
	void normalize() { p /= p.Length(); }
};

struct ControlPnt {
	Point vertices[4]{};
	Point center;
	int winx, winy; // screen coordinates needed for picking
	unsigned char color[3]{};
	float width;
	double timeStamp;  // the time at which the current point reaches this control point.

	ControlPnt() {
		set(0, 0, 0);
		color[0] = rand() % 255; color[1] = rand() % 255; color[2] = rand() % 255;
	}
	ControlPnt(float x, float y, float z) {
		set(x, y, z);
		color[0] = rand() % 255; color[1] = rand() % 255; color[2] = rand() % 255;
	}

	void set(float x, float y, float z) {
		width = PointSize;
		vertices[0].set(x - width, y - width, z);
		vertices[1].set(x + width, y - width, z);
		vertices[2].set(x + width, y + width, z);
		vertices[3].set(x - width, y + width, z);
		center.set(x, y, z);

		winx = (x * (float)WinWidth) / WorldWidth;
		winy = WinHeight - (int)((y * (float)WinHeight) / WorldHeight);
	}

	void Render() {
		glColor3ubv(color);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glBegin(GL_QUADS);
		glVertex2f(vertices[0].p.x, vertices[0].p.y);
		glVertex2f(vertices[1].p.x, vertices[1].p.y);
		glVertex2f(vertices[2].p.x, vertices[2].p.y);
		glVertex2f(vertices[3].p.x, vertices[3].p.y);
		glEnd();
	}

	void RenderToScreen() {
		glColor3ubv(color);
		glPointSize(6);
		glBegin(GL_POINTS);
		glVertex2i(winx, winy);
		glEnd();
	}
};

Point lerp(Point& start, Point& end, float t, float timespan) // timespan = t_2- t_1
{
	Point tmp = (start * ((timespan - t) / timespan));
	tmp = tmp + (end * (t / timespan));
	return tmp;
}

Point LerpCat(Point& start, Point& end, float t, float timespan) // timespan = t_2- t_1
{
	Point tmp = start * (t / timespan);
	tmp = tmp + end * (t / timespan);
	return tmp;
}

void FillTorus(float rc, int numc, float rt, int numt)
{
	int i, j, k;
	double s, t;
	double x, y, z;
	double pi, twopi;

	pi = M_PI;
	twopi = 2. * pi;

	for (i = 0; i < numc; i++) {
		glBegin(GL_QUAD_STRIP);
		for (j = 0; j <= numt; j++) {
			for (k = 1; k >= 0; k--) {
				s = (i + k) % numc + 0.5;
				t = j % numt;

				x = cos(t * twopi / numt) * cos(s * twopi / numc);
				y = sin(t * twopi / numt) * cos(s * twopi / numc);
				z = sin(s * twopi / numc);
				glNormal3f(x, y, z);

				x = (rt + rc * cos(s * twopi / numc)) * cos(t * twopi / numt);
				y = (rt + rc * cos(s * twopi / numc)) * sin(t * twopi / numt);
				z = rc * sin(s * twopi / numc);
				glVertex3f(x, y, z);
			}
		}
		glEnd();
	}
}

void initLighting()
{
	static float lmodel_ambient[] = { 0.5, 0.0, 0.0, 0.0 };
	static float lmodel_twoside[] = { GL_FALSE };
	static float lmodel_local[] = { GL_FALSE };
	static float light0_ambient[] = { 0.0, 0.1, 0.5, 1.0 };
	static float light0_diffuse[] = { 0.0, 0.5, 1.0, 0.0 };
	static float light0_position[] = { 10.0, 10.0, 10.0, 0 };
	static float light0_specular[] = { 1.0, 1.0, 1.0, 0.0 };
	static float bevel_mat_ambient[] = { 0.0, 0.5, 0.0, 1.0 };
	static float bevel_mat_shininess[] = { 40.0 };
	static float bevel_mat_specular[] = { 1.0, 1.0, 1.0, 0.0 };
	static float bevel_mat_diffuse[] = { 0.0, 0.0, 1.0, 0.0 };


	glClearColor(0.5, 0.5, 0.5, 1.0);
	glLightfv(GL_LIGHT0, GL_AMBIENT, light0_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light0_diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light0_specular);
	glLightfv(GL_LIGHT0, GL_POSITION, light0_position);
	glEnable(GL_LIGHT0);

	glLightModelfv(GL_LIGHT_MODEL_LOCAL_VIEWER, lmodel_local);
	glLightModelfv(GL_LIGHT_MODEL_TWO_SIDE, lmodel_twoside);
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);
	//glEnable(GL_LIGHTING);

	glMaterialfv(GL_FRONT, GL_AMBIENT, bevel_mat_ambient);
	glMaterialfv(GL_FRONT, GL_SHININESS, bevel_mat_shininess);
	glMaterialfv(GL_FRONT, GL_SPECULAR, bevel_mat_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, bevel_mat_diffuse);

	glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);
	glShadeModel(GL_SMOOTH);

}

void initGLObjects()
{
	theTorus = glGenLists(1);
	glNewList(theTorus, GL_COMPILE);
	FillTorus(0.3, 8, 1.0, 25);
	glEndList();

	// init lighting
	initLighting();
}

void FsSwapBuffers(void);

void clearScreen()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();									// Reset The Current Modelview Matrix
	glColor3ub(0, 0, 0);
	glBegin(GL_QUADS);
	{
		glVertex2f(0, 0);
		glVertex2f(WorldWidth, 0);
		glVertex2f(WorldWidth, WorldHeight);
		glVertex2f(0, WorldHeight);
		glEnd();
	}
	FsSwapBuffers();
}
