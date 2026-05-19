#include "Vmath.h"
#include "quickMath.h"

Vec3::Vec3(float x, float y, float z)
{
    this->x = x;
    this->y = y;
    this->z = z;
}

Vec3 Vec3::operator+(const Vec3 &other)
{
    return Vec3(x + other.x, y + other.y, z + other.z);
}
Vec3 Vec3::operator-(const Vec3 &other)
{
    return Vec3(x - other.x, y - other.y, z - other.z);
}
Vec3 Vec3::operator*(const float &other)
{
    return Vec3(x * other, y * other, z * other);
}
Vec3 Vec3::operator/(const float &other)
{
    return Vec3(x / other, y / other, z / other);
}
float Vec3::norm()
{
    return 1 / invSqrt(x * x + y * y + z * z);
}
float Vec3::normRecip()
{
    return invSqrt(x * x + y * y + z * z);
}
Vec3 Vec3::normalize()
{
    return *this * this->normRecip();
}
float dot(Vec3 a, Vec3 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
Vec3 cross(Vec3 a, Vec3 b)
{
    Vec3 result;
    result.x = a.y * b.z - a.z * b.y;
    result.y = a.z * b.x - a.x * b.z;
    result.z = a.x * b.y - a.y * b.x;
    return result;
}
Quat Vec3::euler_to_quat(){
    return Quat(0.0f, x, y, z);
}