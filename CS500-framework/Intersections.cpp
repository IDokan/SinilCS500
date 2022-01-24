#include "Intersections.h"

bool FloatEqual(float val, float target)
{
	const float e = glm::epsilon<float>();

	return (val <= target + e) && (val >= target - e);
}

Interval MySlabIntersection(const Ray& r, vec3 n, float d0, float d1)
{
	Interval i;

	const float NdotStart = dot(n, r.start);
	const float NdotDir = dot(n, r.dir);

	// Ray intersects both slab planes
	if (!FloatEqual(NdotDir, 0.f))
	{
		float t0 = -(d0 + NdotStart) / NdotDir;
		float t1 = -(d1 + NdotStart) / NdotDir;

		if (t0 > t1)
		{
			std::swap(t0, t1);
		}

		return Interval(t0, t1);
	}
	else
		// ray is parallel to slab planes
	{
		float s0 = NdotStart + d0;
		float s1 = NdotStart + d1;

		if ((s0 < 0) ? (s1 >= 0) : (s1 < 0))
		{
			// ray is between planes
			return Interval(0.f, INFINITY);
		}
		else
		{
			// ray is outside planes, return empty interval
			return Interval(1.f, 0.f);
		}
	}
}

vec3 Ray::Eval(float t)
{
	return start + t*dir;
}

bool Shape::Intersect(Ray r, Intersection& i)
{
	// returns true (and fills Intersection structure) if ray intersects shape
	return false;
}

bool Sphere::Intersect(Ray r, Intersection& i)
{
	
	vec3 q = r.start - center;

	float dotQD = dot(q, r.dir);
	float discriminant = (dotQD * dotQD) - dot(q, q) + (radius * radius);

	if (discriminant < 0)
	{
		return false;
	}

	float sqrtDiscriminant = sqrt(discriminant);

	float tNegative = -dotQD - sqrtDiscriminant;
	float tPositive = -dotQD + sqrtDiscriminant;


	// Return the smallest positive value

	// If both are negative -> No intersections
	if (tNegative < 0 && tPositive < 0)
	{
		return false;
	}


	// Otherwise return the smallest positive value of the two tNegative and tPositive
	if (tNegative >= 0 && tNegative < tPositive)
	{
		i.t = tNegative;
		i.pointIntersection = r.Eval(tNegative);
		i.normal = normalize(i.pointIntersection - center);
		i.object = this;
		float theta = atan2(i.normal.y, i.normal.x);
		float phi = acos(i.normal.z);
		i.texCoord = vec2(theta / (2 * glm::pi<float>()), phi / glm::pi<float>());
	}
	else if (tPositive >= 0 && tNegative > tPositive)
	{
		i.t = tPositive;
		i.pointIntersection = r.Eval(tPositive);
		i.normal = normalize(i.pointIntersection - center);
		i.object = this;
		float theta = atan2(i.normal.y, i.normal.x);
		float phi = acos(i.normal.z);
		i.texCoord = vec2(theta / (2 * glm::pi<float>()), phi / glm::pi<float>());
	}
	
	return true;
}

vec3 Sphere::BoundingBoxMax()
{
	return center + vec3(radius);
}

vec3 Sphere::BoundingBoxMin()
{
	return center - vec3(radius);
}

bool Box::Intersect(Ray r, Intersection& i)
{
	// Intersection slabs
	// (1, 0, 0), -x, -x-dx
	// (0, 1, 0), -y, -y-dy
	// (0, 0, 1), -z, -z-dz

	const Interval xInterval = MySlabIntersection(r, vec3(1, 0, 0), -corner.x, -corner.x - diagonalVector.x);
	const Interval yInterval = MySlabIntersection(r, vec3(0, 1, 0), -corner.y, -corner.y - diagonalVector.y);
	const Interval zInterval = MySlabIntersection(r, vec3(0, 0, 1), -corner.z, -corner.z - diagonalVector.z);

	Interval interval(0, INFINITY);

	interval.t0 = std::max(std::max(std::max(interval.t0, xInterval.t0), yInterval.t0), zInterval.t0);
	interval.t1 = std::min(std::min(std::min(interval.t1, xInterval.t1), yInterval.t1), zInterval.t1);

	if (interval.t0 > interval.t1)
	{
		// NO Intersection
		return false;
	}
	else
	{
		if (interval.t0 < 0 && interval.t1 < 0)
		{
			return false;
		}
		else 
		{
			if (interval.t0 < interval.t1 && interval.t0 > 0)
			{
				i.pointIntersection = r.Eval(interval.t0);
				i.object = this;

				if (FloatEqual(interval.t0, xInterval.t0))
				{
					i.normal = vec3(-1.f, 0.f, 0.f);
				}
				else if (FloatEqual(interval.t0, yInterval.t0))
				{
					i.normal = vec3(0.f, -1.f, 0.f);
				}
				else if (FloatEqual(interval.t0, zInterval.t0))
				{
					i.normal = vec3(0.f, 0.f, -1.f);
				}
			}
			else if (interval.t1 < interval.t0 && interval.t1 > 0)
			{
				i.pointIntersection = r.Eval(interval.t1);
				i.object = this;

				if (FloatEqual(interval.t1, xInterval.t1))
				{
					i.normal = vec3(1.f, 0.f, 0.f);
				}
				else if (FloatEqual(interval.t1, yInterval.t1))
				{
					i.normal = vec3(0.f, 1.f, 0.f);
				}
				else if (FloatEqual(interval.t1, zInterval.t1))
				{
					i.normal = vec3(0.f, 0.f, 1.f);
				}
			}
			
			return true; 
		}
	}

	return true;
}

vec3 Box::BoundingBoxMax()
{
	return corner + diagonalVector;
}

vec3 Box::BoundingBoxMin()
{
	return corner;
}

Interval::Interval()
{
}

Interval::Interval(float _t0, float _t1)
{
	t0 = _t0;
	t1 = _t1;

	normal0 = vec3(0);
	normal1 = vec3(0);
}

Interval::Interval(float _t0, float _t1, vec3 n0, vec3 n1)
{
	t0 = _t0;
	t1 = _t1;
	normal0 = n0;
	normal1 = n1;
}

void Interval::Empty()
{
	t0 = 0;
	t1 = -1;
}

bool Triangle::Intersect(Ray r, Intersection& i)
{
	vec3 e1 = v1 - v0;
	vec3 e2 = v2 - v0;
	vec3 s = r.start - v0;

	// det(-D, E1, E2) ????
	//	or
	//			(-D)
	//	det	(E1)			???
	//			(E2)

	vec3 de2 = cross(r.dir, e2);
	vec3 se1 = cross(s, e1);

	float d = dot(de2, e1);

	if (d == 0)
	{
		// Ray is parallel to triangle
		return false;
	}

	float u = dot(de2, s) / d;
	if (u < 0 || u > 1)
	{
		// Ray intersects plane, but outside E2 edge
		return false;
	}

	float v = dot(se1, r.dir) / d;
	if (v < 0 || (u + v) > 1)
	{
		// Ray intersects plane, but outside other edges
		return false;
	}

	float t = dot(se1, e2) / d;
	if (t < 0)
	{
		// Ray's negative half intersects triangle
		return false;
	}

	i.t = t;
	i.object = this;
	i.pointIntersection = r.start + t * r.dir;

	i.normal = (1 - u - v) * n0 + u * n1 + v * n2;
	//i.texCoord = (i - u - v)*t0 + u*t1 + v*t2;			// if vertices texture coordinates t0, t1, t2 are known

	return true;
}

vec3 Triangle::BoundingBoxMax()
{
	return vec3(
		std::max(std::max(v0.x, v1.x), v2.x),
		std::max(std::max(v0.y, v1.y), v2.y),
		std::max(std::max(v0.z, v1.z), v2.z)
	);
}

vec3 Triangle::BoundingBoxMin()
{
	return vec3(
		std::min(std::min(v0.x, v1.x), v2.x),
		std::min(std::min(v0.y, v1.y), v2.y),
		std::min(std::min(v0.z, v1.z), v2.z)
	);
}

bool Cylinder::Intersect(Ray r, Intersection& i)
{

	vec3 a(0, 0, 1);
	vec3 b = normalize(cross(vec3(1, 0, 0), vec3(0, 0, 1)));
	vec3 c = cross(a, b);
	mat3 rInverse = mat3(b, c, a);
	mat3 BaseToZMatrix = glm::transpose(rInverse);

	vec3 newStart = BaseToZMatrix * (r.start - base);
	vec3 newDir = BaseToZMatrix * (r.dir - base);

	Interval firstInterval = MySlabIntersection(r, vec3(0, 0, 1), 0, -length(a));


	float polyB = dot(newDir.x, newStart.x) + dot(newDir.y, newStart.y);
	float polyA = (dot(newDir.x, newDir.x) + dot(newDir.y, newDir.y));
	float discriminant = pow(polyB, 2) - ((dot(newDir.x, newDir.x)+dot(newDir.y, newDir.y)) * (dot(newStart.x, newStart.x) + dot(newStart.y, newStart.y) - pow(radius, 2)));

	if (discriminant < 0)
	{
		return false;
	}
	else
	{
		float sqrtedDiscriminant = sqrt(discriminant);
		float secondInterval0 = (-polyB - sqrtedDiscriminant) / polyA;
		float secondInterval1	= (-polyB + sqrtedDiscriminant) / polyA;


		float ultT0 = std::max(firstInterval.t0, secondInterval0);
		float ultT1 = std::min(firstInterval.t1, secondInterval1);

		if (ultT0 > ultT1)
		{
			// NO-INTERSECTION
			// The "off the corner" case
			return false;
		}

		if (ultT0 < 0 && ultT1 < 0)
		{
			return false;
		}

		if (ultT0 > 0 && ultT0 < ultT1)
		{
			i.t = ultT0;
			i.pointIntersection = r.Eval(ultT0);
			i.object = this;

			// Choose normal depends on which interval.
			if (FloatEqual(ultT0, firstInterval.t0))
			{
				if (newStart.z < 0)
				{
					i.normal = vec3(0, 0, -1);
				}
				else
				{
					i.normal = vec3(0, 0, 1);
				}
			}
			else
			{
				i.normal = vec3(
					newStart.x + ultT0 * newDir.x,
					newStart.y + ultT0 * newDir.y,
					0.f);
			}
		}
		else if (ultT1 > 0 && ultT1 < ultT0)
		{
			i.t = ultT1;
			i.pointIntersection = r.Eval(ultT1);
			i.object = this;

			// Choose normal depends on which interval.
			if (FloatEqual(ultT1, firstInterval.t1))
			{
				if (newStart.z < 0)
				{
					i.normal = vec3(0, 0, -1);
				}
				else
				{
					i.normal = vec3(0, 0, 1);
				}
			}
			else
			{
				i.normal = vec3(
					newStart.x + ultT0 * newDir.x,
					newStart.y + ultT0 * newDir.y,
					0.f);
			}
		}
	}

	i.normal = rInverse * i.normal;

	return true;
}

vec3 Cylinder::BoundingBoxMax()
{
	const vec3 p0 = base - vec3(radius);
	const vec3 p1 = base + vec3(radius);
	const vec3 p2 = axis + base - vec3(radius);
	const vec3 p3 = axis + base + vec3(radius);

	vec3 result;
	result.x = std::max(std::max(p0.x, p1.x), std::max(p2.x, p3.x));
	result.y = std::max(std::max(p0.y, p1.y), std::max(p2.y, p3.y));
	result.z = std::max(std::max(p0.z, p1.z), std::max(p2.z, p3.z));

	return result;
}

vec3 Cylinder::BoundingBoxMin()
{
	const vec3 p0 = base - vec3(radius);
	const vec3 p1 = base + vec3(radius);
	const vec3 p2 = axis + base - vec3(radius);
	const vec3 p3 = axis + base + vec3(radius);

	vec3 result;
	result.x = std::min(std::min(p0.x, p1.x), std::min(p2.x, p3.x));
	result.y = std::min(std::min(p0.y, p1.y), std::min(p2.y, p3.y));
	result.z = std::min(std::min(p0.z, p1.z), std::min(p2.z, p3.z));

	return result;
}
