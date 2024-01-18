#pragma once

#include "graphicsDefs.h"

// generate Lagrange Polynomial interpolation between 3 control points
struct LagrangeInterp
{
	Point currPoint;
	float averageSpeed;
	Point* p[3];
	std::map<float, Point> pathMap;
	std::map<float, Point>::const_iterator currIt;
	bool isSet{ false };
	bool usePath{ false };

	LagrangeInterp() :averageSpeed(0.0f), isSet(false), usePath(false), p{ nullptr, nullptr, nullptr }
	{};

	void set(Point& p1, Point& p2, Point& p3, float timerange, bool usingPath = true)
	{
		float dist1 = p2.dist(p1);
		float dist2 = p3.dist(p2);
		p1.timeTag = 0.f;
		p2.timeTag = dist1 / (dist1 + dist2) * timerange;
		p3.timeTag = timerange;
		p[0] = &p1;
		p[1] = &p2;
		p[2] = &p3;
		currPoint = p1;
		buildPathMap(timerange);
		isSet = true;
		usePath = usingPath;
	}

	void buildPathMap(float timerange)
	{
		const float inc{ 1.f / 30.f };
		size_t totalCount = static_cast<size_t>(timerange / inc);
		float pathLen = 0.0f;
		float timeElapsed = 0.0f;
		pathMap.clear();
		pathMap[0] = currPoint;
		float currentTime = 0.0f;
		for (int i = 1; i < totalCount; i++)
		{
			currentTime += inc;
			Point pos = computeLagrange(currentTime);
			pathLen += pos.dist(currPoint);
			pathMap[pathLen] = pos;
			currPoint = pos;
		}
		currIt = pathMap.cbegin();
		averageSpeed = pathLen / timerange;
	}

	Point computeLagrange(float t)
	{
		float mult1 = (t - p[1]->timeTag) * (t - p[2]->timeTag) / ((p[0]->timeTag - p[1]->timeTag) * (p[0]->timeTag - p[2]->timeTag));
		float mult2 = (t - p[0]->timeTag) * (t - p[2]->timeTag) / ((p[1]->timeTag - p[0]->timeTag) * (p[1]->timeTag - p[2]->timeTag));
		float mult3 = (t - p[0]->timeTag) * (t - p[1]->timeTag) / ((p[2]->timeTag - p[0]->timeTag) * (p[2]->timeTag - p[1]->timeTag));
		float3D pnt{ (p[0]->p * mult1) + (p[1]->p * mult2) + (p[2]->p * mult3) };
		return Point{ pnt.x, pnt.y, t };
	}

	void displayPath() const
	{
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glLoadIdentity();									// Reset The Current Modelview Matrix
		glColor3ub(28, 10, 255);
		glPushMatrix();
		glBegin(GL_LINE_STRIP);
		for (auto& mapItem : pathMap)
		{
			glVertex3f(mapItem.second.p.x, mapItem.second.p.y, mapItem.second.p.z);
		}
		glEnd();
		glPopMatrix();
	}

	// given current path lenght along the curve, find next position along the curve based on physics.
	Point next(float tInc)
	{
		float pathAdvancement = averageSpeed * tInc; // find how much we should advance on the path.

		if (currIt == pathMap.cend())
		{
			currIt = pathMap.cbegin();
		}

		const float nextPathLength = (*currIt).first + pathAdvancement;
		float diff = pathAdvancement;

		do
		{
			diff = fabs(nextPathLength - currIt->first);
			++currIt;
		} while (currIt != pathMap.cend() && fabs(nextPathLength - (*currIt).first) <= diff);

		if (currIt != pathMap.cend())
			--currIt;
		else
		{
			return pathMap.crbegin()->second;
		}

		return currIt->second;
	}

	Point nextCurrent(float tm)
	{
		return computeLagrange(tm);
	}
};
