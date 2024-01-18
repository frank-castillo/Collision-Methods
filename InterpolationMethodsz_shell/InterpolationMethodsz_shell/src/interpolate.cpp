#include <stdio.h>
#include <string.h>

#include <iostream>

#include "graphicsDefs.h"
#include "CubicBezier.h"
#include "CutmulRom.h"
#include "Hermite.h"
#include "Lagrange.h"
#include "QuadraticBezier.h"

#include "wcode/fssimplewindow.h"
#include "bitmapfont/ysglfontdata.h"
#include <format>

const unsigned maxControlPoints{ 15 };

float pieceLength[maxControlPoints - 1];
Point currentPoint; //  this is position of the moving point.
unsigned controlPntCnt = 0;  // current number of control points. 

ControlPnt contrlPnts[maxControlPoints];
CubicBezier mCubicBezier;
CutMulRom mCutmulRom;
HermiteInterp mHermitePoly;
LagrangeInterp mLagrangePoly;
QuadBezier mQuadraticBezier;

bool isSpline = false;
bool usingPath = false;

void init();

void Draw_Axes(void)
{
	static float ORG[3] = { 0, 0, 0 };
	static float XP[3] = { 3, 0, 0 }, YP[3] = { 0, 3, 0 }, ZP[3] = { 0, 0, 3 };

	//glPushMatrix();

//	glTranslatef(-2.4, -1.5, -5);
//	glRotatef(tip, 1, 0, 0);
//	glRotatef(turn, 0, 1, 0);
//	glScalef(0.25, 0.25, 0.25);

	//glLineWidth(3.0);

	glBegin(GL_LINES);
	glColor3b(125, 0, 0); // X axis is red.
	glVertex3fv(ORG);
	glVertex3fv(XP);
	glColor3b(0, 125, 0); // Y axis is green.
	glVertex3fv(ORG);
	glVertex3fv(YP);
	glColor3b(0, 0, 125); // z axis is blue.
	glVertex3fv(ORG);
	glVertex3fv(ZP);
	glEnd();

	//	glPopMatrix();
}

int width = 0, height = 0;

int UI_Process()
{
	FsPollDevice();
	int key = FsInkey();
	keys[key] = true;
	int ret = key;
	switch (key)
	{
	case FSKEY_S:
		ret = eStart;
		break;
	case FSKEY_ESC:
		ret = eStop;
		break;
	case FSKEY_L:
		eInterpolation = eLinear;
		break;
	case FSKEY_H:
		eInterpolation = eHermite;
		break;
	case FSKEY_G:
		eInterpolation = eLagrange;
		break;
	case FSKEY_B:
		eInterpolation = eCubicBezier;
		break;
	case FSKEY_Q:
		eInterpolation = eQuadraticBezier;
		break;
	case FSKEY_C:
		eInterpolation = eCatmulRom;
		break;
	case FSKEY_1:
		usingPath = !usingPath;
		break;
	}
	return ret;
}

//////////////////////////////////////////////////////////////////////////////////////////////
int Menu()
{
	int r = 0;

	while (r != eStart && r != eStop)
	{
		r = UI_Process();

		if (r == eStop)
			return eStop;
		int wid, hei;
		FsGetWindowSize(wid, hei);

		glViewport(0, 0, wid, hei);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glOrtho(-0.5, (GLdouble)wid - 0.5, (GLdouble)hei - 0.5, -0.5, -1, 1);

		glClearColor(0.5, 0.5, 0.5, 1.0);
		glClear(GL_COLOR_BUFFER_BIT);
		glColor3ub(127, 127, 127);

		char sSpeed[200];
		sprintf(sSpeed, "Starting point (%f, %f), End point (%f, %f)\n",
			contrlPnts[0].center.p.x, contrlPnts[0].center.p.y, contrlPnts[maxControlPoints - 1].center.p.x, contrlPnts[maxControlPoints - 1].center.p.y);
		char* message = "Use L, H, B, Q, G, C to cycle among interpolation schemes.\n";
		char interpolstyle[180];
		sprintf(interpolstyle, "interpolation method is %s %s %s!\n",
			(eInterpolation == eLinear ? "Linear" :
				eInterpolation == eCubicBezier ? "Cubic Bezier" :
				eInterpolation == eQuadraticBezier ? "Quadratic Bezier" :
				eInterpolation == eHermite ? "Hermite" :
				eInterpolation == eLagrange ? "Lagrange. Press 1 to turn path on and off" : "CatMulRom"),
			(isSpline ? "spline" : ""),
			(usingPath ? "Using path" : "Not using path"));

		glColor3ub(255, 255, 255);
		glRasterPos2i(32, 32);
		glCallLists(strlen(sSpeed), GL_UNSIGNED_BYTE, sSpeed);
		glRasterPos2i(32, 64);
		glCallLists(strlen(message), GL_UNSIGNED_BYTE, message);
		glRasterPos2i(32, 96);
		glCallLists(strlen(interpolstyle), GL_UNSIGNED_BYTE, interpolstyle);

		const char* msg1 = "S.....Start Game";
		const char* msg2 = "ESC...Exit";
		glRasterPos2i(32, 128);
		glCallLists(strlen(msg1), GL_UNSIGNED_BYTE, msg1);
		glRasterPos2i(32, 160);
		glCallLists(strlen(msg2), GL_UNSIGNED_BYTE, msg2);

		FsSwapBuffers();
		FsSleep(10);
	}

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	FsSwapBuffers();
	init();

	return r;
}

///////////////////////////////////////////////////////////////
void updatePath(double timeInc)
{
	switch (eInterpolation)
	{
	case eLinear:
	{
		int pieceIndex = 0;
		while ((contrlPnts[pieceIndex].timeStamp <= timePassed))// || fabs(contrlPnts[pieceIndex].timeStamp - time) < 0.0001)
			pieceIndex++;

		if (pieceIndex <= 0)
			std::cout << "ERROR!!!updatePath(): error! pieceIndex<=0 ?\n";

		Point startPnt = contrlPnts[pieceIndex - 1].center;
		Point endPnt = contrlPnts[pieceIndex].center;

		float pieceTime = timePassed - contrlPnts[pieceIndex - 1].timeStamp;
		float pieceTimeSpan = contrlPnts[pieceIndex].timeStamp - contrlPnts[pieceIndex - 1].timeStamp;
		currentPoint = lerp(startPnt, endPnt, pieceTime, pieceTimeSpan);
		break;
	}
	case eLagrange:
	{
		if (mLagrangePoly.isSet && usingPath)
			currentPoint = mLagrangePoly.next(timePassed);
		else if (mLagrangePoly.isSet && !usingPath)
			currentPoint = mLagrangePoly.nextCurrent(timePassed);
		break;
	}
	case eHermite:
	{
		if (mHermitePoly.isSet)
			currentPoint = mHermitePoly.next(timePassed);
		break;
	}
	case eCubicBezier:
	{
		if (mCubicBezier.isSet)
			currentPoint = mCubicBezier.next(timePassed);
		break;
	}
	case eCatmulRom:
	{
		if (mCutmulRom.isSet)
			//currentPoint = mCutmulRom.next(timePassed);
			currentPoint = mCutmulRom.NextMultipoint(timePassed);
			//currentPoint = mCutmulRom.CalculateCatmullPoint(timePassed);
		break;
	}
	case eQuadraticBezier:
	{
		if (mQuadraticBezier.isSet)
			currentPoint = mQuadraticBezier.next(timePassed);
		break;
	}
	default:
		std::cout << "Out of bound selection! Please select a valid one. " << std::endl;
	}

	timePassed += timeInc;
}

void init()
{
	isSpline = false;
	timePassed = 0.;
	controlPntCnt = 0;
	contrlPnts[controlPntCnt++].set(PointSize + 1., WorldHeight / 3.0, 0.);
	contrlPnts[controlPntCnt++].set(WorldWidth - PointSize - 1.0, WorldHeight / 2.0, 0);
	contrlPnts[0].timeStamp = 0.f;
	contrlPnts[1].timeStamp = timeRange;

	pieceLength[0] = contrlPnts[1].center.dist(contrlPnts[0].center);
}

/////////////////////////////////////////////////////////////////////////////////////////////////
void reset(bool clear)
{
	timePassed = 0.0;

	currentPoint = contrlPnts[0].center;

	glClearDepth(2.0f);									// Depth Buffer Setup
	glEnable(GL_DEPTH_TEST);							// Enables Depth Testing
	glDepthFunc(GL_LEQUAL);								// The Type Of Depth Testing To Do
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);	// Really Nice Perspective Calculations

	/// clear the window:
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0., (GLdouble)WorldWidth, 0., (GLdouble)WorldHeight, -10, 10);

	glMatrixMode(GL_MODELVIEW);							// Select The Modelview Matrix
	glLoadIdentity();									// Reset The Modelview Matrix

	if (clear)
	{
		clearScreen();
	}
}

void renderControlPoints(GLenum mode)
{
	if (mode == GL_RENDER) {
		for (int i = 0; i < controlPntCnt; i++) {
			contrlPnts[i].Render();
		}

		if (eInterpolation == eLagrange && usingPath)
			mLagrangePoly.displayPath();
	}
	else if (mode == GL_SELECT) {
		for (int i = 0; i < controlPntCnt; i++) {
			glLoadName(i);
			contrlPnts[i].RenderToScreen();
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////
void renderScene()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();									// Reset The Current Modelview Matrix

	///////////// draw the end or control points /////////////
	renderControlPoints(GL_RENDER);

	//////////////////////////// draw the path  //////////////////////////////////
	glPointSize(2);
	glColor3ub(28, 10, 255);
	glBegin(GL_POINTS);
	glVertex2f(currentPoint.p.x, currentPoint.p.y);
	glEnd();

	glPushMatrix();
	glEnable(GL_LIGHTING);
	glLoadIdentity();
	glTranslatef(currentPoint.p.x, currentPoint.p.y, currentPoint.p.z);
	static float angle{};

	angle++;
	float angleRad = angle * M_PI / 180.f;
	//glRotatef(angle, 1, 0, 0);
	float m[16] = { 1.0, 0.0, 0.0, 0.0,                   // rotation matrix to rotate around x by increment of 1 degree each frame.
		0.0, cosf(angleRad), sinf(angleRad), 0.0,
		0.0, -sinf(angleRad), cosf(angleRad), 0.0,
		0.0, 0.0, 0.0, 1.0 };
	glMultMatrixf(m);
	glCallList(theTorus);
	glDisable(GL_LIGHTING);
	Draw_Axes();
	glPopMatrix();

	FsSwapBuffers();
}

// support for picking
#define MAXSELECT 100   
#define MAXFEED 300
GLuint selectBuf[MAXSELECT];
GLfloat feedBuf[MAXFEED];
int vp[4];

void recalculateTimeStamps();
void updateControlPoint(int mx, int my, float xWorld, float yWorld)
{
	glSelectBuffer(MAXSELECT, selectBuf);
	glRenderMode(GL_SELECT);
	glInitNames();
	glPushName(~0);

	glPushMatrix();

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPickMatrix(mx, my, 30, 30, vp);
	gluOrtho2D(0, WinWidth, 0, WinHeight);
	glMatrixMode(GL_MODELVIEW);

	glClearColor(0.5, 0.5, 0.5, 1.0);
	glClear(GL_COLOR_BUFFER_BIT);

	renderControlPoints(GL_SELECT);

	glPopMatrix();
	int hits = glRenderMode(GL_RENDER);
	if (hits > 0)
	{
		int hitIdx = selectBuf[(hits - 1) * 4 + 3];
		if (hitIdx != -1)
		{
			contrlPnts[hitIdx].set(xWorld, yWorld, 0.);
		}

		if (eInterpolation == eLagrange && controlPntCnt == 3) // we only choose first 3 cntrl points for Lagrange
			mLagrangePoly.set(contrlPnts[0].center, contrlPnts[1].center, contrlPnts[2].center, timeRange, usingPath);

		else if (eInterpolation == eHermite && controlPntCnt == 4)
			mHermitePoly.set(contrlPnts[0].center.p, contrlPnts[1].center.p, contrlPnts[2].center.p, contrlPnts[3].center.p, timeRange);

		else if (eInterpolation == eLinear)
			recalculateTimeStamps();

		else if (eInterpolation == eCubicBezier && controlPntCnt == 4)
			mCubicBezier.set(contrlPnts[0].center.p, contrlPnts[1].center.p, contrlPnts[2].center.p, contrlPnts[3].center.p, timeRange);

		else if (eInterpolation == eCatmulRom && controlPntCnt >= 4)
			//mCutmulRom.set(contrlPnts[0].center, contrlPnts[1].center, contrlPnts[2].center, contrlPnts[3].center, timeRange);
			mCutmulRom.set(contrlPnts, controlPntCnt, timeRange);

		else if (eInterpolation == eQuadraticBezier && controlPntCnt == 3)
			mQuadraticBezier.set(contrlPnts[0].center.p, contrlPnts[1].center.p, contrlPnts[2].center.p, timeRange);
	}
}

void recalculateTimeStamps()
{
	// first calculate piecelength for each piece, i.e. distance between consecutive control pnts
	float totalLength = 0.f;
	for (int i = 1; i < controlPntCnt; i++)
	{
		pieceLength[i - 1] = contrlPnts[i].center.dist(contrlPnts[i - 1].center);
		totalLength += pieceLength[i - 1];
	}

	// now compute timestamp for each control point
	for (int i = 1; i < controlPntCnt; i++)
	{
		contrlPnts[i].timeStamp = contrlPnts[i - 1].timeStamp +
			(pieceLength[i - 1] / totalLength) * timeRange;
	}

	if (eInterpolation == eHermite && controlPntCnt == 4)
		mHermitePoly.set(contrlPnts[0].center.p, contrlPnts[1].center.p, contrlPnts[2].center.p, contrlPnts[3].center.p, timeRange);

	else if (eInterpolation == eLagrange && controlPntCnt == 3)
		mLagrangePoly.set(contrlPnts[0].center, contrlPnts[1].center, contrlPnts[2].center, timeRange, usingPath);

	else if (eInterpolation == eCubicBezier && controlPntCnt == 4)
		mCubicBezier.set(contrlPnts[0].center.p, contrlPnts[1].center.p, contrlPnts[2].center.p, contrlPnts[3].center.p, timeRange);

	else if (eInterpolation == eCatmulRom && controlPntCnt >= 4)
		//mCutmulRom.set(contrlPnts[0].center, contrlPnts[1].center, contrlPnts[2].center, contrlPnts[3].center, timeRange);
		mCutmulRom.set(contrlPnts, controlPntCnt, timeRange);

	else if (eInterpolation == eQuadraticBezier && controlPntCnt == 3)
		mQuadraticBezier.set(contrlPnts[0].center.p, contrlPnts[1].center.p, contrlPnts[2].center.p, timeRange);
}

///////////////////////////////////////////////////////////////////
int Game(void)
{
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glDepthFunc(GL_LEQUAL);

	initGLObjects();

	DWORD passedTime = 0;
	FsPassedTime(true);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//////////// initial setting up the scene ////////////////////////////////////////
	reset(true);
	int timeSpan = 33; // milliseconds
	double timeInc = (double)timeSpan * 0.001; // time increment in seconds

	FsGetWindowSize(WinWidth, WinHeight);

	int lb, mb, rb, mx, my;

	glViewport(0, 0, WinWidth, WinHeight);
	glGetIntegerv(GL_VIEWPORT, vp);
	////////////////////// main simulation loop //////////////////////////
	while (1)
	{
		int key = UI_Process();
		if (key == eStop)
		{
			init();
			break;
		}
		int mouseEventType = FsGetMouseEvent(lb, mb, rb, mx, my);
		float xWorld = ((float)mx / (float)WinWidth) * WorldWidth;
		float yWorld = ((float)(WinHeight - my) / (float)WinHeight) * WorldHeight;
		// right mouse button clicked. So create a new control point here:
		if (rb == 1 && mouseEventType == FSMOUSEEVENT_RBUTTONDOWN &&
			(controlPntCnt != maxControlPoints - 1))
		{
			if ((eInterpolation != eLagrange || controlPntCnt < 3) &&
				(eInterpolation != eHermite || controlPntCnt < 4) &&
				(eInterpolation != eCatmulRom || controlPntCnt <= 4 || controlPntCnt >= 4) &&
				(eInterpolation != eCubicBezier || controlPntCnt < 4) &&
				(eInterpolation != eQuadraticBezier || controlPntCnt < 3)
				)
			{
				contrlPnts[controlPntCnt] = contrlPnts[controlPntCnt - 1];
				contrlPnts[controlPntCnt - 1].set(xWorld, yWorld, 0);
				++controlPntCnt;
				recalculateTimeStamps();
				reset(false);
				glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			}
		}
		// start dragging operation on a control point:
		else if (lb == 1 && mouseEventType == FSMOUSEEVENT_MOVE)
		{
			updateControlPoint(mx, my, xWorld, yWorld);
			recalculateTimeStamps();
			reset(false);
		}

		//specify if it is a spline or not!
		if (eInterpolation == eLinear)
			isSpline = false;
		else if (eInterpolation == eLagrange)
		{
			isSpline = false;
		}
		else if (eInterpolation == eCubicBezier && controlPntCnt > 4)
			isSpline = true;
		else if (controlPntCnt > 3)
			isSpline = true;

		timeInc = (double)(passedTime) * 0.001;

		/////////// update path /////////////////
		updatePath(timeInc);
		if (timePassed >= timeRange)
		{
			reset(false);
		}

		renderScene();

		////// update time lapse /////////////////
		passedTime = FsPassedTime(); // Making it up to 50fps
		int timediff = timeSpan - passedTime;
		//	printf("\ntimeInc=%f, passedTime=%d, timediff=%d", timeInc, passedTime, timediff);
		while (timediff >= timeSpan / 3)
		{
			FsSleep(5);
			passedTime = FsPassedTime(); // Making it up to 50fps
			timediff = timeSpan - passedTime;
			//		printf("--passedTime=%d, timediff=%d", passedTime, timediff);
		}
		passedTime = FsPassedTime(true); // Making it up to 50fps
	}
	return 0;
}

/////////////////////////////////////////////////////////////////
void GameOver(int score)
{
	int r = 0;
	init();

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

		glClearColor(0.5, 0.5, 0.5, 0.0);
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

//////////////////////////////////////////////////////////////////////////////////////
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
		reset(false);
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
