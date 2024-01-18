#pragma once
#include "graphicsDefs.h"
#include <vector>

struct Segment
{
	float3D a;
	float3D b;
	float3D c;
	float3D d;
};

struct CutMulRom
{
	Point currPoint;// current point in flight path.
	Point p0, p1, p2, p3{};
	bool isSet{ false };
	Segment splineSegment;
	float totalLength{};

	void set(Point pnt1, Point pnt2, Point pnt3, Point pnt4, float tr)
	{
		p0 = pnt1;
		p1 = pnt2;
		p2 = pnt3;
		p3 = pnt4;
		currPoint = p0;
		timeRange = tr;
		isSet = true;
		float alpha = 0.5f;
		float tension = 0.0f;

		totalLength += p0.dist(p1);
		totalLength += (p1.dist(p2));
		totalLength += (p2.dist(p3));
		float speed = totalLength / tr;

		float t0 = 0.0f;
		float t1 = (p0.dist(p1) / speed) + t0;
		float t2 = (p1.dist(p2) / speed) + t1;
		float t3 = (p2.dist(p3) / speed) + t2;

		float3D m1 = (1.0f - tension) * (t2 - t1) * ((p1.p - p0.p) / (t1 - t0) - (p2.p - p0.p) / (t2 - t0) + (p2.p - p1.p) / (t2 - t1));
		float3D m2 = (1.0f - tension) * (t2 - t1) * ((p2.p - p1.p) / (t2 - t1) - (p3.p - p1.p) / (t3 - t1) + (p3.p - p2.p) / (t3 - t2));

		splineSegment.a = 2.0f * (p1.p - p2.p) + m1 + m2;
		splineSegment.b = -3.0f * (p1.p - p2.p) - m1 - m1 - m2;
		splineSegment.c = m1;
		splineSegment.d = p1.p;
	}

	Point next(float tm)
	{
		float t{ tm / timeRange };
		currPoint.p = (splineSegment.a * t * t * t) + (splineSegment.b * t * t) + (splineSegment.c * t) + (splineSegment.d);
		return currPoint;
	}

	// Multipoint Implementation
	std::vector<Point> catmulPoints;
	int iteration = 1;
	int currentPointIndex = 0;

	void set(ControlPnt points[], int pointCount, float tr)
	{
		catmulPoints.clear();

		iteration = pointCount;

		for (int i = 0; i < pointCount - 1; ++i)
		{
			float distance = points[i].center.dist(points[i + 1].center);
			std::cout << distance << std::endl;
			totalLength += distance;
		}
		std::cout << totalLength << std::endl;

		float speed = totalLength / tr;

		float d1 = points[0].center.dist(points[1].center);
		float d2 = points[iteration - 2].center.dist(points[iteration - 1].center);
		float averageDistance = (d1 + d2) / 2;

		float3D dir1 = points[1].center.p - points[0].center.p;
		float3D dir2 = points[iteration - 2].center.p - points[iteration - 1].center.p;

		// Add ghost points for correct traversal of the whole path
		catmulPoints.push_back(Point(dir1 * averageDistance, { 0,1,0 }, 0));
		for (int i = 0; i < iteration; ++i)
		{
			catmulPoints.push_back(points[i].center);
		}
		catmulPoints.push_back(Point(dir2 * averageDistance, { 0,1,0 }, 0));

		catmulPoints[0].timeTag = 0.0f;
		catmulPoints[1].timeTag = 0.0f;
		for (int i = 2; i < iteration; ++i)
		{
			float distance = catmulPoints[i].dist(catmulPoints[i - 1]);
			catmulPoints[i].timeTag = (distance / speed) + catmulPoints[i - 1].timeTag;
			std::cout << distance << "      " << catmulPoints[i].timeTag << std::endl;
		}
		catmulPoints[iteration].timeTag = tr;
		catmulPoints[iteration + 1].timeTag = 0.0f;

		/*for (int i = 1; i < iteration + 1; i++)
		{
			catmulPoints[i].timeTag = timeRange / (iteration - 1) * (i - 1);
		}*/

		timeRange = tr;
		isSet = true;
	}

	Point NextMultipoint(float tm)
	{
		for (int i = 1; i < iteration; ++i)
		{
			if (tm >= catmulPoints[i].timeTag && tm < catmulPoints[i + 1].timeTag)
			{
				p0 = catmulPoints[i - 1];
				p1 = catmulPoints[i];
				p2 = catmulPoints[i + 1];
				p3 = catmulPoints[i + 2];
			}
		}
		
		float par = (tm - p1.timeTag) / (p2.timeTag - p1.timeTag);

		float alpha = 0.5f;
		float tension = 0.0f;

		float t0 = 0.0f;
		float t1 = t0 + pow(p0.dist(p1), alpha);
		float t2 = t1 + pow(p1.dist(p2), alpha);
		float t3 = t2 + pow(p2.dist(p3), alpha);

		float3D m1 = (1.0f - tension) * (t2 - t1) * ((p1.p - p0.p) / (t1 - t0) - (p2.p - p0.p) / (t2 - t0) + (p2.p - p1.p) / (t2 - t1));
		float3D m2 = (1.0f - tension) * (t2 - t1) * ((p2.p - p1.p) / (t2 - t1) - (p3.p - p1.p) / (t3 - t1) + (p3.p - p2.p) / (t3 - t2));

		splineSegment.a = 2.0f * (p1.p - p2.p) + m1 + m2;
		splineSegment.b = -3.0f * (p1.p - p2.p) - m1 - m1 - m2;
		splineSegment.c = m1;
		splineSegment.d = p1.p;

		currPoint.p = (splineSegment.a * par * par * par) + (splineSegment.b * par * par) + (splineSegment.c * par) + (splineSegment.d);
		return currPoint;

		//float t{ tm / timeRange };
		/*currentPointIndex == iteration ? currentPointIndex = 0 : ++currentPointIndex;
		currPoint = LerpCat(pointsArray[currentPointIndex].center, pointsArray[currentPointIndex + 1].center, tm, pointsArray[currentPointIndex + 1].timeStamp);
		return currPoint;*/
	}
};
