#pragma once
#include "graphicsDefs.h"

struct CubicBezier
{
	Point currPoint;// current point in flight path.
	float3D p[4];
	float timeRange{ 1.f };
	bool isSet{false};

	void set(float3D p1, float3D p2, float3D p3, float3D p4, float tr)
	{
		p[0] = p1;
		p[3] = p4;
		p[1] = p2;
		p[2] = p3;
		currPoint.p = p[0];
		timeRange = tr;
		isSet = true;
	}

	Point next(float tm) // giving ct it interpolates using Hermite interpolation.
	{
		float t{ tm / timeRange };
		float3D res = ((1.0f - t) * (1.0f - t) * (1.0f - t) * p[0]) + (3.0f * t * (1.0f - t) * (1.0f - t) * p[1]) + (3.0f * (t * t) * (1.0f - t) * p[2]) + ((t * t * t) * p[3]);
		currPoint.p = res;
		return currPoint;
	}
};