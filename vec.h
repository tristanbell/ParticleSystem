#ifndef __vec_h_
#define __vec_h_

class Vec3
{
public: // Member variables
	float x;
	float y;
	float z;

public: // Methods
	Vec3(float x, float y, float z);
	
	float dot(Vec3 *other);
	float length();
	
	Vec3 normalised();
	
	// Operators
	Vec3 operator+(const Vec3& other)
	{
		Vec3 newVec(x + other.x, y + other.y, z + other.z);
		return newVec;
	}
	
	Vec3 operator-(const Vec3& other)
	{
		Vec3 newVec(x - other.x, y - other.y, z - other.z);
		return newVec;
	}
	
	Vec3 operator*(const Vec3& other)
	{
		Vec3 newVec(x * other.x, y * other.y, z * other.z);
		return newVec;
	}
};

#endif
