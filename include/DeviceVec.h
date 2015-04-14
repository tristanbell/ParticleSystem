#include "vec.h"

class DeviceVec {

public: // Member variables
	float x;
	float y;
	float z;

public: // Methods
	__device__ DeviceVec(float x, float y, float z);
	__device__ DeviceVec(DeviceVec *v);
	__device__ DeviceVec(Vec3 *v);

	__device__ Vec3 toVec3();

	__device__ float dot(DeviceVec *other);

	__device__ float length() {
		float sum = 0;
		sum += x * x;
		sum += y * y;
		sum += z * z;

		return sqrt(sum);
	}

	__device__ DeviceVec normalised();

	__device__ DeviceVec *reflection(DeviceVec *normal);

	// Operators
	__device__ DeviceVec operator+(const DeviceVec& other)
	{
		DeviceVec newVec(x + other.x, y + other.y, z + other.z);
		return newVec;
	}

	__device__ DeviceVec operator-(const DeviceVec& other)
	{
		DeviceVec newVec(x - other.x, y - other.y, z - other.z);
		return newVec;
	}

	__device__ DeviceVec operator*(const DeviceVec& other)
	{
		DeviceVec newVec(x * other.x, y * other.y, z * other.z);
		return newVec;
	}
};
