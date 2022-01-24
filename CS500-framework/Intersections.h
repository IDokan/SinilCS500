/*
Sinil.Kang
DigiPen cs500s22
1/19/2022
*/

#pragma once

#include <../geom.h>

struct Ray;
class Shape;
struct Intersection;
class Interval;
class Material;

bool FloatEqual(float val, float target);

Interval MySlabIntersection(const Ray& r, vec3 n, float d0, float d1);

struct Ray
{
public:
	vec3 Eval(float t);

	vec3 start;
	vec3 dir;
};

class Shape
{
public:
	virtual bool Intersect(Ray r, Intersection& i);
};

class Sphere : public Shape
{
public:
	Sphere(vec3 position, float radius);

	virtual bool Intersect(Ray r, Intersection& i);

	vec3 BoundingBoxMax();
	vec3 BoundingBoxMin();

private:
	vec3 center;
	float radius;
};

class Box : public Shape
{
public:
	Box(vec3 position, vec3 diagonal);

	virtual bool Intersect(Ray r, Intersection& i);

	vec3 BoundingBoxMax();
	vec3 BoundingBoxMin();

private:
	vec3 corner;
	vec3 diagonalVector;
};

class Cylinder : public Shape
{
public:
	Cylinder(vec3 position, vec3 axis, float radius);

	virtual bool Intersect(Ray r, Intersection& i);


	vec3 BoundingBoxMax();
	vec3 BoundingBoxMin();

private:
	vec3 base;
	vec3 axis;
	float radius;
};

class Triangle : public Shape
{
public:
	virtual bool Intersect(Ray r, Intersection& i);

	vec3 BoundingBoxMax();
	vec3 BoundingBoxMin();

private:
	vec3 v0, v1, v2;
	vec3 n0, n1, n2;
};

struct Intersection
{
public:
	float t;								// Parameter value on ray of the point of intersection
	Shape* object;					// A pointer to the Shape intersected
	vec3 pointIntersection;	// Point of intersection (in world coordinates)
	vec3 normal;					// Normal of surface at intersection point (in world coordinates)
	vec2 texCoord;				// for instance 2D or 3D texture coordinates
};

class Interval
{
public:
	Interval();
	Interval(float _t0, float _t1);
	Interval(float _t0, float _t1, vec3 n0, vec3 n1);		// Reorders t0,t1 (and N0,N1) so t0 <= t1, then stores all 4.
	void Empty();		// Sets t0, t1 to 0, -1 to represent an empty interval
	void Intersect(Interval& i);
	void Intersect(Ray r);

public:
	float t0, t1;		// beginning and ending point along a ray
	vec3 normal0, normal1;		// Surface normals at t0 and t1 respectively.
};