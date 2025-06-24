#ifndef HFDVECTOR_H_
#define HFDVECTOR_H_

#include <math.h>
#include "hfdDefines.h"

#ifdef __cplusplus
#include <ostream>

template < class T >
class hfdVector3D
{
public:

	typedef T EltType;

    hfdVector3D();
    hfdVector3D(T x, T y, T z);
    explicit hfdVector3D(const T val[3]);

	template<typename U>
    hfdVector3D(const hfdVector3D<U> &rhs)
	{
		*this = rhs;
	}

	template<typename U>
    inline hfdVector3D &operator=(const hfdVector3D<U> &rhs)
	{
		m_p[0] = rhs[0];
		m_p[1] = rhs[1];
		m_p[2] = rhs[2];
		return *this;
	}

	const T &operator[](int i) const;
	T &operator[](int i);

	void set(T x, T y, T z);

	operator const T*() const { return &m_p[0]; }
	operator T*(){ return &m_p[0]; }

    hfdVector3D &operator = (const hfdVector3D &v1);
    hfdVector3D &operator -= (const hfdVector3D &v1);
    hfdVector3D &operator += (const hfdVector3D &v1);
    hfdVector3D &operator *= (const hfdVector3D &v1);
    hfdVector3D &operator *= (HFDdouble s);
    hfdVector3D &operator /= (const hfdVector3D &v1);
    hfdVector3D &operator /= (HFDdouble s);


    HFDboolean isZero(T epsilon) const;
    HFDdouble distance(const hfdVector3D& v1)const;
    HFDdouble distanceSqr(const hfdVector3D& v1) const;

    HFDdouble magnitude() const;
    HFDdouble magnitude(const hfdVector3D& v)const;
	void normalize();
    HFDdouble dotProduct(const hfdVector3D& v1) const;
    hfdVector3D crossProduct(const hfdVector3D& v1)const;

	int getLongestAxisComponent() const;
	int getSecondLongestAxisComponent() const;
	int getShortestAxisComponent() const;


private:
	T m_p[3];
};

typedef hfdVector3D<HFDfloat>  hfdVector3Df;
typedef hfdVector3D<HFDdouble> hfdVector3Dd;

#include "hfdVector.inl"

#else

typedef HFDfloat hfdVector3Df[3];
typedef HFDdouble hfdVector3Dd[3];

#endif // __cplusplus

#ifdef __cplusplus
extern "C"
{
#endif


__inline void hfdVecSet(HFDdouble *vec,
    HFDdouble x,
    HFDdouble y,
    HFDdouble z)
{
	vec[0] = x;
	vec[1] = y;
	vec[2] = z;
}

__inline HFDdouble hfdVecMagnitude(const HFDdouble*vec)
{
	return (sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]));
}

__inline HFDboolean hfdVecIsZero(const HFDdouble *vec, HFDdouble epsilon)
{
	return((fabs(vec[0]) < epsilon) &&
			(fabs(vec[1]) < epsilon) &&
			(fabs(vec[2]) < epsilon));
}

__inline void hfdVecAdd(HFDdouble*res,
    const HFDdouble*vec1,
    const HFDdouble*vec2)
{
	res[0] = vec1[0] + vec2[0];
	res[1] = vec1[1] + vec2[1];
	res[2] = vec1[2] + vec2[2];
}

__inline void hfdVecSubtract(HFDdouble*res,
    const HFDdouble*vec1,
    const HFDdouble*vec2)
{
	res[0] = vec1[0] - vec2[0];
	res[1] = vec1[1] - vec2[1];
	res[2] = vec1[2] - vec2[2];
}

__inline void hfdVecScale(HFDdouble *res,
    const HFDdouble *vec,
    HFDdouble s)
{
	res[0] = vec[0] * s;
	res[1] = vec[1] * s;
	res[2] = vec[2] * s;
}

__inline void hfdVecScaleInPlace(HFDdouble *vec,
    HFDdouble s)
{
	vec[0] = vec[0] * s;
	vec[1] = vec[1] * s;
	vec[2] = vec[2] * s;
}

__inline void hfdVecScaleNonUniform(HFDdouble *res,
    const HFDdouble *vec,
    const HFDdouble *s)
{
	res[0] = vec[0] * s[0];
	res[1] = vec[1] * s[1];
	res[2] = vec[2] * s[2];
}

__inline void hfdVecScaleNonUniformInPlace(HFDdouble *vec,
    const HFDdouble *s)
{
	vec[0] = vec[0] * s[0];
	vec[1] = vec[1] * s[1];
	vec[2] = vec[2] * s[2];
}

__inline void hfdVecNormalize(HFDdouble *res,
    const HFDdouble *vec)
{
    double mag = hfdVecMagnitude(vec);
	if (mag==0)
	{
		return;
	}
    hfdVecScale(res, vec, 1.0 / mag);
}

__inline void hfdVecNormalizeInPlace(HFDdouble *vec)
{
    double mag = hfdVecMagnitude(vec);
	if (mag == 0) return;
    hfdVecScaleInPlace(vec, 1.0 / mag);
}

__inline void hfdVecCrossProduct(HFDdouble *res,
    const HFDdouble *vec1,
    const HFDdouble *vec2)
{
	res[0] = vec1[1] * vec2[2] - vec1[2] * vec2[1];
	res[1] = vec1[2] * vec2[0] - vec1[0] * vec2[2];
	res[2] = vec1[0] * vec2[1] - vec1[1] * vec2[0];
}

__inline HFDdouble hfdVecDotProduct(const HFDdouble *vec1,
    const HFDdouble *vec2)
{
	return (vec1[0] * vec2[0] + vec1[1] * vec2[1] + vec1[2] * vec2[2]);
}

__inline HFDdouble hfdVecDistance(const HFDdouble *vec1,
    const HFDdouble *vec2)
{
    hfdVector3Dd vec3;
    hfdVecSubtract(vec3, vec1, vec2);
    return hfdVecMagnitude(vec3);
}


#ifdef __cplusplus
}
#endif

#endif // !HFDVECTOR_H_
