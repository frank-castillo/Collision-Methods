#pragma once
#include "graphicsDefs.h"

// This is shell structure to support Lagrange interpolation. You must
// provide the rest of functinality and implemenation as needed.
struct QuadBezier
{
	Point currPoint;// current point in flight path.
	float3D p[3];
	bool isSet{false};

	void set(float3D p1, float3D p2, float3D p3, float tr)
	{
		p[0] = p1;
		p[2] = p3;
		p[1] = p2;
		currPoint.p = p[0];
		timeRange = tr;
		isSet = true;
	}

	Point next(float tm) // giving ct it interpolates using Hermite interpolation.
	{
		float t{ tm / timeRange };
		float3D res = ((1.0f - t) * (1.0f - t) * p[0]) + (2.0f * t * (1.0f - t) * p[1]) + ((t * t) * p[2]);
		currPoint.p = res;
		return currPoint;
	}
};