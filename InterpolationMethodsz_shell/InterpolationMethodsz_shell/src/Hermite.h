#pragma once

#include "graphicsDefs.h"
#include "vectors.h"

struct HermiteInterp
{
	Point currPoint;// current point in flight path.
	// add data you need to implement Hermite
	float3D p[2];
	float3D dir[2];
	float timeRange{ 1.f };

	bool isSet{ false };

	void set(float3D p1, float3D p2, float3D p3, float3D p4, float tr)
	{
		p[0] = p1;
		p[1] = p4;
		dir[0] = 3.f * (p2 - p1);
		dir[1] = 3.f * (p4 - p3);
		currPoint.p = p[0];
		timeRange = tr;
		isSet = true;
	}
	
	Point next(float tm) // giving ct it interpolates using Hermite interpolation.
	{
		float t{ tm / timeRange };
		float3D res = (1.f - 3.f * t * t + 2.f * t * t * t) * p[0] +
			t * t * (3.f - 2.f * t) * p[1] + t * (t - 1.f) * (t - 1.f) * dir[0]
			+ t * t * (t - 1.f) * dir[1];
		currPoint.p = res;
		return currPoint;
	}

};