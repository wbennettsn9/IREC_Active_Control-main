#pragma once


class Vec3; // This is required in order to avoid circular dependencies

/*
Contains 4 members : q0, q1, q2, q3
Can be used to perform quaternion operations
*/
class Quat
{
public:
    float q0;
    float q1;
    float q2;
    float q3;
    
    // Reserve the object
    Quat() = default;
    // Construct the object with default values
    Quat(float q0, float q1, float q2, float q3);

    /*
    Quaternion Multiplication
    To multiply 2 quaternions just write r * s
    This is not a cumtative!
    */
    Quat operator*(const Quat &other);
    /*
    Quaternion Addition
    To multiply 2 quaternions just write r + s
    */
    Quat operator+(const Quat &other);
    /*
    Quaternion Substraction
    To subtract 2 quaternions just write r - s
    This is not a cumtative!
    */
    Quat operator-(const Quat &other);
    /*
    Quaternion Scaling, only works if float comes AFTER quaternion
    To scale quaternion q by s write q * s
    */ 
    Quat operator*(const float s);
    // Returns the conjugate of the quaternion
    Quat conj();
    // Returns the magnitude of the quaternion
    float norm();
    // Returns 1/norm() or the reciprical of the vector magnitude
    float normRecip();
    // returns the quaternion with the same direction but magnitude of 1
    Quat normalize();
    // Converts quaternion to Vec3 Euler angle
    Vec3 quat_to_euler();
	// Finds x_hat direction defined by rotation quaternion
	Vec3 x_dir();
	// Finds y_hat direction defined by rotation quaternion
	Vec3 y_dir();
	// Finds z_hat direction defined by rotation quaternion
	Vec3 z_dir();
};

Quat rotatePassive(Quat rotation_quaternion, Quat rotated_quaternion);
Quat rotateActive(Quat rotation_quaternion, Quat rotated_quaternion);