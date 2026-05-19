#include "Qmath.h"
#include "Vmath.h"
#include "quickMath.h"
#include <cmath>

Quat::Quat(float q0, float q1, float q2, float q3)
{
    this->q0 = q0;
    this->q1 = q1;
    this->q2 = q2;
    this->q3 = q3;
}

Quat Quat::operator*(const Quat &other)
{
    Quat result;
    result.q0 = q0 * other.q0 - q1 * other.q1 - q2 * other.q2 - q3 * other.q3;
    result.q1 = q0 * other.q1 + q1 * other.q0 + q2 * other.q3 - q3 * other.q2;
    result.q2 = q0 * other.q2 - q1 * other.q3 + q2 * other.q0 + q3 * other.q1;
    result.q3 = q0 * other.q3 + q1 * other.q2 - q2 * other.q1 + q3 * other.q0;
    return result;
}
Quat Quat::operator+(const Quat &other)
{
    return Quat(
        q0 + other.q0,
        q1 + other.q1,
        q2 + other.q2,
        q3 + other.q3);
}
Quat Quat::operator-(const Quat &other)
{
    return Quat(
        q0 - other.q0,
        q1 - other.q1,
        q2 - other.q2,
        q3 - other.q3);
}
Quat Quat::operator*(const float s)
{
    return Quat(q0 * s, q1 * s, q2 * s, q3 * s);
}
Quat Quat::conj()
{
    return Quat(q0, -q1, -q2, -q3);
}
float Quat::norm()
{
    return 1 / invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
}
float Quat::normRecip()
{
    return invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
}
Quat Quat::normalize()
{
    return *this * this->normRecip();
}
Quat rotatePassive(Quat rotation_quaternion, Quat rotated_quaternion)
{
    return rotation_quaternion * (rotated_quaternion * rotation_quaternion.conj());
}
Quat rotateActive(Quat rotation_quaternion, Quat rotated_quaternion)
{
    return rotation_quaternion.conj() * (rotated_quaternion * rotation_quaternion);
}

Vec3 Quat::quat_to_euler(){
    Vec3 roll_pitch_yaw;
    roll_pitch_yaw.x = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);
    roll_pitch_yaw.y = asinf(-2.0f * (q1*q3 - q0*q2));
    roll_pitch_yaw.z = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3);
    return roll_pitch_yaw;
}

Vec3 Quat::x_dir(){
	Vec3 x_dir;
	x_dir.x = q0*q0 + q1*q1 - q2*q2 - q3*q3;
	x_dir.y = 2.0f*q1*q2 + 2.0f*q0*q3;
	x_dir.z = 2.0f*q1*q3 - 2.0f*q0*q2;
	return x_dir;
}

Vec3 Quat::y_dir(){
	Vec3 y_dir;
	y_dir.x = 2.0f*q1*q2 - 2.0f*q0*q3;
	y_dir.y = q0*q0 - q1*q1 + q2*q2 - q3*q3;
	y_dir.z = 2.0f*q2*q3 + 2.0f*q0*q1;
	return y_dir;
}

Vec3 Quat::z_dir(){
	Vec3 z_dir;
	z_dir.x = 2.0f*q1*q3 - 2.0f*q0*q2;
	z_dir.y = 2.0f*q2*q3 + 2.0f*q0*q1;
	z_dir.z = -1.0f + 2.0f*q0*q0 + 2.0f*q3*q3;
	return z_dir;
}
