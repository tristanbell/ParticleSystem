#include "DeviceVec.h"

DeviceVec::DeviceVec(float x, float y, float z) : x(x), y(y), z(z)
{ }

DeviceVec::DeviceVec(DeviceVec *v) : x(v->x), y(v->y), z(v->z)
{ }

DeviceVec::DeviceVec(Vec3 *v) : x(v->x), y(v->y), z(v->z)
{ }

Vec3 DeviceVec::toVec3()
{
	return Vec3(x, y, z);
}

float DeviceVec::dot(DeviceVec *other)
{
	return x * other->x + y * other->y + z * other->z;
}

DeviceVec DeviceVec::normalised()
{
	float len = length();

	DeviceVec normalised(x / len, y / len, z / len);
	return normalised;
}

DeviceVec *DeviceVec::reflection(DeviceVec *normal)
{
	DeviceVec tmp(x, y, z);
	tmp = tmp - (normal * 2 * this->dot(normal));

	return new DeviceVec(tmp);
}
