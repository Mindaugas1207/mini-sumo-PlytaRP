
#ifndef INC_VMATH_H_
#define INC_VMATH_H_

#include "pico/stdlib.h"
#include <math.h>
#include <array>
#include <tuple>
#include <algorithm>
#include <cstdint>

namespace vmath
{
    template <typename T = double, char iterations = 2> inline T Inv_sqrt(T x)
    {
        static_assert(std::is_floating_point<T>::value, "T must be floating point");
        static_assert(iterations > 0, "itarations must be more than 0");
        typedef typename std::conditional<sizeof(T) == 8, std::int64_t, std::int32_t>::type Tint;

        T y = x;
        T x2 = y * (T)0.5;
        Tint i = *(Tint *)&y;
        i = (sizeof(T) == 8 ? 0x5fe6eb50c7b537a9 : 0x5f3759df) - (i >> 1);
        y = *(T *)&i;

        char it = iterations;
        do
        {
            y = y * ((T)1.5 - (x2 * y * y));
        } while (it--);

        return y;
    }

    template <typename T = double> inline T ConstrainAngle(T x)
    {
        x = std::fmod(x + (T)M_PI, (T)(2 * M_PI));

        if (x < 0) return x + (T)(M_PI);
        return x - (T)M_PI;
    }

    template <typename T = double> inline T UnwrapAngle(T angle) {
        if (angle > (T)M_PI) return angle - (T)(M_PI*2);
        else if (angle < (T)(-M_PI)) return angle + (T)(M_PI*2);
        else return angle;
    }

    template <typename T = double> inline T AngleDifference(T from, T to) {
        return UnwrapAngle(to - from);
    }

    template <typename T = double, size_t c = 3, size_t r = 3>
    struct matrix_t : std::array<std::array<T, c>, r>
    {

    };

    template <typename T = double>
    struct vect_t
    {
        T X, Y, Z;

        T& Roll()  { return this->X; }
        T& Pitch() { return this->Y; }
        T& Yaw()   { return this->Z; }

        void Roll (const T& val) { this->X = val; }
        void Pitch(const T& val) { this->Y = val; }
        void Yaw  (const T& val) { this->Z = val; }

        vect_t& operator=  (const vect_t& rhs) { this->X  = rhs.X; this->Y  = rhs.Y; this->Z  = rhs.Z; return *this; }
        vect_t& operator+= (const vect_t& rhs) { this->X += rhs.X; this->Y += rhs.Y; this->Z += rhs.Z; return *this; }
        vect_t& operator-= (const vect_t& rhs) { this->X -= rhs.X; this->Y -= rhs.Y; this->Z -= rhs.Z; return *this; }
        vect_t& operator*= (const vect_t& rhs) { this->X *= rhs.X; this->Y *= rhs.Y; this->Z *= rhs.Z; return *this; }
        vect_t& operator/= (const vect_t& rhs) { this->X /= rhs.X; this->Y /= rhs.Y; this->Z /= rhs.Z; return *this; }
        template <typename Tr> vect_t& operator=  (const Tr& rhs) { this->X  = rhs; this->Y  = rhs; this->Z  = rhs; return *this; }
        template <typename Tr> vect_t& operator+= (const Tr& rhs) { this->X += rhs; this->Y += rhs; this->Z += rhs; return *this; }
        template <typename Tr> vect_t& operator-= (const Tr& rhs) { this->X -= rhs; this->Y -= rhs; this->Z -= rhs; return *this; }
        template <typename Tr> vect_t& operator*= (const Tr& rhs) { this->X *= rhs; this->Y *= rhs; this->Z *= rhs; return *this; }
        template <typename Tr> vect_t& operator/= (const Tr& rhs) { this->X /= rhs; this->Y /= rhs; this->Z /= rhs; return *this; }

        friend vect_t operator+ (vect_t lhs, const vect_t& rhs) { lhs += rhs; return lhs; }
        friend vect_t operator- (vect_t lhs, const vect_t& rhs) { lhs -= rhs; return lhs; }
        friend vect_t operator* (vect_t lhs, const vect_t& rhs) { lhs *= rhs; return lhs; }
        friend vect_t operator/ (vect_t lhs, const vect_t& rhs) { lhs /= rhs; return lhs; }
        template <typename Tr> friend vect_t operator+ (vect_t lhs, const Tr& rhs) { lhs += rhs; return lhs; }
        template <typename Tr> friend vect_t operator- (vect_t lhs, const Tr& rhs) { lhs -= rhs; return lhs; }
        template <typename Tr> friend vect_t operator* (vect_t lhs, const Tr& rhs) { lhs *= rhs; return lhs; }
        template <typename Tr> friend vect_t operator/ (vect_t lhs, const Tr& rhs) { lhs /= rhs; return lhs; }

        T Length() { return sqrt(this->X * this->X + this->Y * this->Y + this->Z * this->Z); }

        static vect_t Cross(const vect_t& lhs, const vect_t& rhs)
        {
            return {
                lhs.Y * rhs.Z - lhs.Z * rhs.Y,
                lhs.Z * rhs.X - lhs.X * rhs.Z,
                lhs.X * rhs.Y - lhs.Y * rhs.X
            };
        }

        vect_t& Cross(const vect_t& rhs)
        {
            this->X = this->Y * rhs.Z - this->Z * rhs.Y;
            this->Y = this->Z * rhs.X - this->X * rhs.Z;
            this->Z = this->X * rhs.Y - this->Y * rhs.X;
            return *this;
        }
        
        vect_t& operator^= (const vect_t& rhs) { return Cross(rhs); }

        friend vect_t operator^ (vect_t lhs, const vect_t& rhs) { lhs ^= rhs; return lhs; }

        vect_t& Normalize(void) { return *this *= Inv_sqrt(X * X + Y * Y + Z * Z); }

        vect_t& Rotate(const matrix_t<T, 3, 3>& _Rotation)
        {
            this->X = this->X * _Rotation[0][0] + this->Y * _Rotation[0][1] + this->Z * _Rotation[0][2];
            this->Y = this->X * _Rotation[1][0] + this->Y * _Rotation[1][1] + this->Z * _Rotation[1][2];
            this->Z = this->X * _Rotation[2][0] + this->Y * _Rotation[2][1] + this->Z * _Rotation[2][2];
            return *this *= (T)2.0;
        }

        vect_t<T> AngleDifferenceFrom(const vect_t& from)
        {
            return {
                AngleDifference(from.X, this->X),
                AngleDifference(from.Y, this->Y),
                AngleDifference(from.Z, this->Z)
            };
        }
    };

    // struct matrix_s
    // {
    //     std::array<std::array<double, 3>, 3> mx;
    // };

    // typedef struct matrix_s matrix_t;

    template <typename T = double>
    struct rotMatrix_t : matrix_t<T, 3, 3>
    {
        rotMatrix_t Transpose(void)
        {
            rotMatrix_t _Result;

            _Result[0][0] = (*this)[0][0];
            _Result[0][1] = (*this)[1][0];
            _Result[0][2] = (*this)[2][0];

            _Result[1][0] = (*this)[0][1];
            _Result[1][1] = (*this)[1][1];
            _Result[1][2] = (*this)[2][1];

            _Result[2][0] = (*this)[0][2];
            _Result[2][1] = (*this)[1][2];
            _Result[2][2] = (*this)[2][2];

            return _Result;
        }

        double Determinant(void)
        {
            return ((*this)[0][0] * ((*this)[1][1] * (*this)[2][2] - (*this)[1][2] * (*this)[2][1]))
                 - ((*this)[0][1] * ((*this)[1][0] * (*this)[2][2] - (*this)[1][2] * (*this)[2][0]))
                 + ((*this)[0][2] * ((*this)[1][0] * (*this)[2][1] - (*this)[1][1] * (*this)[2][0]));
        }

        vect_t<T> RotateVector(const vect_t<T>& _Vector)
        {
            vect_t<T> _Result;

            _Result.X = _Vector.X * (*this)[0][0] + _Vector.Y * (*this)[0][1] + _Vector.Z * (*this)[0][2];
            _Result.Y = _Vector.X * (*this)[1][0] + _Vector.Y * (*this)[1][1] + _Vector.Z * (*this)[1][2];
            _Result.Z = _Vector.X * (*this)[2][0] + _Vector.Y * (*this)[2][1] + _Vector.Z * (*this)[2][2];

            return _Result *= (T)2.0;
        }
    };

    template <typename T = double>
    struct quat_t
    {
        T W, X, Y, Z;

        quat_t& operator=  (const quat_t& rhs) { this->W  = rhs.W; this->X  = rhs.X; this->Y  = rhs.Y; this->Z  = rhs.Z; return *this; }
        quat_t& operator+= (const quat_t& rhs) { this->W += rhs.W; this->X += rhs.X; this->Y += rhs.Y; this->Z += rhs.Z; return *this; }
        quat_t& operator-= (const quat_t& rhs) { this->W -= rhs.W; this->X -= rhs.X; this->Y -= rhs.Y; this->Z -= rhs.Z; return *this; }
        quat_t& operator*= (const quat_t& rhs) { this->W *= rhs.W; this->X *= rhs.X; this->Y *= rhs.Y; this->Z *= rhs.Z; return *this; }
        quat_t& operator/= (const quat_t& rhs) { this->W /= rhs.W; this->X /= rhs.X; this->Y /= rhs.Y; this->Z /= rhs.Z; return *this; }
        template <typename Tr> quat_t& operator=  (const Tr& rhs) { this->W  = rhs; this->X  = rhs; this->Y  = rhs; this->Z  = rhs; return *this; }
        template <typename Tr> quat_t& operator+= (const Tr& rhs) { this->W += rhs; this->X += rhs; this->Y += rhs; this->Z += rhs; return *this; }
        template <typename Tr> quat_t& operator-= (const Tr& rhs) { this->W -= rhs; this->X -= rhs; this->Y -= rhs; this->Z -= rhs; return *this; }
        template <typename Tr> quat_t& operator*= (const Tr& rhs) { this->W *= rhs; this->X *= rhs; this->Y *= rhs; this->Z *= rhs; return *this; }
        template <typename Tr> quat_t& operator/= (const Tr& rhs) { this->W /= rhs; this->X /= rhs; this->Y /= rhs; this->Z /= rhs; return *this; }

        friend quat_t operator+ (quat_t lhs, const quat_t& rhs) { lhs += rhs; return lhs; }
        friend quat_t operator- (quat_t lhs, const quat_t& rhs) { lhs -= rhs; return lhs; }
        friend quat_t operator* (quat_t lhs, const quat_t& rhs) { lhs *= rhs; return lhs; }
        friend quat_t operator/ (quat_t lhs, const quat_t& rhs) { lhs /= rhs; return lhs; }
        template <typename Tr> friend quat_t operator+ (quat_t lhs, const Tr& rhs) { lhs += rhs; return lhs; }
        template <typename Tr> friend quat_t operator- (quat_t lhs, const Tr& rhs) { lhs -= rhs; return lhs; }
        template <typename Tr> friend quat_t operator* (quat_t lhs, const Tr& rhs) { lhs *= rhs; return lhs; }
        template <typename Tr> friend quat_t operator/ (quat_t lhs, const Tr& rhs) { lhs /= rhs; return lhs; }

        quat_t& Normalize(void) { return *this *= Inv_sqrt(this->W * this->W + this->X * this->X + this->Y * this->Y + this->Z * this->Z); }

        quat_t Multiply(const quat_t& rhs)
        {
            quat_t result;

            result.W = this->W * rhs.W - this->X * rhs.X - this->Y * rhs.Y - this->Z * rhs.Z;
            result.X = this->X * rhs.W + this->W * rhs.X + this->Y * rhs.Z - this->Z * rhs.Y;
            result.Y = this->W * rhs.Y - this->X * rhs.Z + this->Y * rhs.W + this->Z * rhs.X;
            result.Z = this->W * rhs.Z + this->X * rhs.Y - this->Y * rhs.X + this->Z * rhs.W;

            return result;
        }
        
        rotMatrix_t<T> RotationMatrix(void)
        {
            rotMatrix_t<T> _Matrix;
            T xx, yy, zz, xy, xz, yz, xw, yw, zw;

            xx = this->X * this->X;
            xy = this->X * this->Y;
            xz = this->X * this->Z;
            xw = this->X * this->W;
            yy = this->Y * this->Y;
            yz = this->Y * this->Z;
            yw = this->Y * this->W;
            zz = this->Z * this->Z;
            zw = this->Z * this->W;
            
            _Matrix[0][0] = (T)1 -  (2 * (yy - zz));//
            _Matrix[0][1] =           (xy - zw);
            _Matrix[0][2] =           (xz + yw);

            _Matrix[1][0] =           (xy + zw);//
            _Matrix[1][1] = ((T)0.5 -  xx - zz);
            _Matrix[1][2] =           (yz - xw);

            _Matrix[2][0] =           (xz - yw);//
            _Matrix[2][1] =           (yz + xw);//
            _Matrix[2][2] = ((T)0.5 -  xx - yy);//

            return _Matrix;
        }

        static vect_t<T> EulerAngles(const rotMatrix_t<T>& _Rotation)
        {
            return {
                atan2(_Rotation[2][1], _Rotation[2][2]), //Roll
                asinf((T)(-2.0) * _Rotation[2][0]), //Pitch
                atan2(2*_Rotation[1][0], _Rotation[0][0]) //Yaw
            };
        }

        vect_t<T> EulerAngles(void) { return EulerAngles(RotationMatrix()); }
    };
/*
    quat_t<double> quaternionFromRotationZ(double angle)
    {
        quat_t result;
        result.W = cos(angle / 2.0);
        double c = sin(angle / 2.0);
        result.X = 0;
        result.Y = 0;
        result.Z = c;
        return result;
    }
*/
    template <typename T = double>
    struct MadgwickFilter
    {
        quat_t<T> q;
        T beta;

    public:
        void Init()
        {
            beta = (T)0.0;
            q = { (T)1.0, (T)0.0, (T)0.0, (T)0.0 };
        }

        void Init(T _beta)
        {
            beta = _beta;
            q = { (T)1.0, (T)0.0, (T)0.0, (T)0.0 };
        }

        void Reset(void)
        {
            q = { (T)1.0, (T)0.0, (T)0.0, (T)0.0 };
        }

        void SetBeta(const T& _beta) { beta = _beta; }
        T& GetBeta(void)             { return beta;  }

        quat_t<T> Compute(vect_t<T> gyro, vect_t<T> accel, T dt) {

            quat_t<T> grad, qDot;
            // Rate of change of quaternion from gyroscope
            qDot.W = -q.X * gyro.X - q.Y * gyro.Y - q.Z * gyro.Z;
            qDot.X =  q.W * gyro.X + q.Y * gyro.Z - q.Z * gyro.Y;
            qDot.Y =  q.W * gyro.Y - q.X * gyro.Z + q.Z * gyro.X;
            qDot.Z =  q.W * gyro.Z + q.X * gyro.Y - q.Y * gyro.X;
            qDot *= (T)0.5;

            // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
            if(!(accel.X == (T)0.0 && accel.Y == (T)0.0 && accel.Z == (T)0.0))
            {
                // Normalise accelerometer measurement
                accel.Normalize();
                // Auxiliary variables to avoid repeated arithmetic
                T tmp_2yyxx = 2 * q.Y * q.Y + q.X * q.X;
                T tmp_z = 1 + tmp_2yyxx + accel.Z;
                // Gradient decent algorithm corrective step
                grad.W = q.W * tmp_2yyxx + q.Y * accel.X - q.X * accel.Y;
                grad.Z = q.Z * tmp_2yyxx - q.X * accel.X - q.Y * accel.Y;
                grad.X = q.Z * (2 * q.X * q.Z - accel.X) + q.W * (2 * q.W * q.X - accel.Y) - 2 * q.X * tmp_z;
                grad.Y = q.W * (2 * q.W * q.Y + accel.X) + q.Z * (2 * q.Y * q.Z - accel.Y) - 2 * q.Y * tmp_z;
                //grad *= 2; //Can skip this step because of normalization below

                grad.Normalize();
                // Apply feedback step
                qDot -= grad * beta;
            }

            // Integrate rate of change of quaternion to yield quaternion
            q += qDot * dt;
            // Normalise quaternion
            q.Normalize();

            return q;
        }
    };
};

#endif // INC_VMATH_H_
