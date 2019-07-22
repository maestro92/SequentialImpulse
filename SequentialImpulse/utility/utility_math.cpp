#include "utility_math.h"

// const float utl::MATH_EPSILON = 1e-5;
string utl::intToStr(int value)
{
	stringstream ss;
	ss << value; // add number to string
	return ss.str();
}

string utl::floatToStr(float value)
{
    ostringstream buff;
    buff << std::setprecision(4) << value;
    return buff.str();
}

glm::mat4 utl::axes2GLMMat4(glm::vec3 x, glm::vec3 y, glm::vec3 z)
{
    float temp[16] = {x[0], x[1], x[2], 0.0,
                      y[0], y[1], y[2], 0.0,
                      z[0], z[1], z[2], 0.0,
                      0.0,  0.0,  0.0,  1.0 };
    return glm::make_mat4(temp);
}

glm::mat3 utl::axes2GLMMat3(glm::vec3 x, glm::vec3 y, glm::vec3 z)
{
    float temp[16] = { x[0], x[1], x[2],
                      y[0], y[1], y[2],
                      z[0], z[1], z[2] };
    return glm::make_mat3(temp);
}


// solving A x = b, but only for the first two equation
glm::vec3 utl::solve22(glm::mat3 A, glm::vec3 b)
{
    /*
    glm has the layout of 
    float temp[16] = {x[0], x[1], x[2], 0.0,
                      y[0], y[1], y[2], 0.0,
                      z[0], z[1], z[2], 0.0,
                      0.0,  0.0,  0.0,  1.0 };
    */

    float a00 = A[0][0];    // x[0]
    float a01 = A[1][0];    // y[0]

    float a10 = A[0][1];    // x[1]
    float a11 = A[1][1];    // y[1]

    float det = a00 * a11 - a01 * a10;

    if (det != 0.0f)
    {
        det = 1.0f / det;
    }

    glm::vec3 x;
    x.x = det * (a11 *  b.x - a01 * b.y);
    x.y = det * (a00 *  b.y - a10 * b.x);
    return x;
}


string utl::vec2ToStr(glm::vec2 value)
{
	ostringstream buff;
	buff << std::setprecision(4) << value.x << " " << std::setprecision(4) << value.y;
	return buff.str();
}

int utl::isPointLeftOfVector(glm::vec2 v0, glm::vec2 v1, glm::vec2 point)
{
	return ((v1.x - v0.x) * (point.y - v0.y) - (point.x - v0.x) * (v1.y - v0.y));
}


int utl::randInt(int min, int max)
{
	float result = randFloat((float)min, (float)max);
	return (int)(floor(result));
}


float utl::randFloat(float min, float max)
{
    float num = (float)rand() / (float) RAND_MAX;
    return min + (max - min) * num;
}

float utl::barycentricInterpolation(glm::vec3 p1, glm::vec3 p2, glm::vec3 p3, glm::vec2 pos)
{
	float det = (p2.z - p3.z) * (p1.x - p3.x) + (p3.x - p2.x) * (p1.z - p3.z);
	float l1 = ((p2.z - p3.z) * (pos.x - p3.x) + (p3.x - p2.x) * (pos.y - p3.z)) / det;
	float l2 = ((p3.z - p1.z) * (pos.x - p3.x) + (p1.x - p3.x) * (pos.y - p3.z)) / det;
	float l3 = 1.0f - l1 - l2;
	return l1 * p1.y + l2 * p2.y + l3 * p3.y;
}


float utl::crossProduct2D(glm::vec2 a, glm::vec2 b)
{
	return a.x * b.y - a.y * b.x;
}

bool utl::chance(float percent)
{
	const float value = rand() / (float)RAND_MAX * 100;
	return value <= percent;
}

glm::vec3 utl::interpolateEntityPosition(glm::vec3 pos0, glm::vec3 pos1, float interpFactor)
{
	return pos0 + (pos1 - pos0) * interpFactor;
}


bool utl::equals(glm::vec2 a, glm::vec2 b)
{
	float MATH_EPSILON = 1e-5;
	return (abs(a.x - b.x) < MATH_EPSILON && abs(a.y - b.y) < MATH_EPSILON);
}


glm::vec3 utl::interpolateEntityAngles(glm::vec3 pos0, glm::vec3 pos1, float interpFactor)
{
	glm::vec3 diff = pos1 - pos0;

	for (int i = 0; i < 3; i++)
	{
		if (diff[i] > 180)
		{
			diff[i] -= 360;
		}

		if (diff[i] < -180)
		{
			diff[i] += 360;
		}
	}

	return pos0 + diff * interpFactor;
}

float utl::interpolateAngle(float f0, float f1, float interpFactor)
{
	return 0;
}

bool utl::sameSign(float a, float b)
{
	if (a > 0 && b > 0)
		return true;
	if (a < 0 && b < 0)
		return true;
	if (a == 0 && b == 0)
		return true;
	return false;
}


// RealTime Collision Detection 5.4.2
bool utl::isPointInTriangle(glm::vec2 point, glm::vec2 v0, glm::vec2 v1, glm::vec2 v2)
{
	float p01 = crossProduct2D(point - v0, v1 - v0);
	float p12 = crossProduct2D(point - v1, v2 - v1);

	if (!sameSign(p01, p12))
		return false;

	float p20 = crossProduct2D(point - v2, v0 - v2);
	if (!sameSign(p01, p20))
		return false;
	
	return true;
}



bool utl::isPointInTriangle(glm::vec3 point, glm::vec3 v0, glm::vec3 v1, glm::vec3 v2)
{
	return true;
}

float utl::sqDistBetweenPointAndLineSetment(glm::vec2 l0, glm::vec2 l1, glm::vec2 p)
{
	glm::vec2 ab = l1 - l0;
	glm::vec2 ac = p - l0;
	glm::vec2 bc = p - l1;

	float e = glm::dot(ac, ab);

	// handles cases where c projects outside ab
	if (e <= 0.0f)
		return glm::dot(ac, ac);

	float f = glm::dot(ab, ab);

	if (e >= f)
		return glm::dot(bc, bc);

	return glm::dot(ac, ac) - e * e / f;

}

float utl::sqDistBetweenPointAndLineSetment(glm::vec3 l0, glm::vec3 l1, glm::vec3 p)
{
	return 0;
}

