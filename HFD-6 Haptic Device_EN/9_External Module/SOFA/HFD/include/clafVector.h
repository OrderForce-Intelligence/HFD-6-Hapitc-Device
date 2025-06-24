#ifndef CLAFVECTOR_H_
#define CLAFVECTOR_H_

#include <math.h>
#include "clafDefines.h"

#ifdef __cplusplus
#include <ostream>

template < class T >
class clafVector3D
{
public:

	typedef T EltType;

	clafVector3D();
	clafVector3D(T x, T y, T z);
	explicit clafVector3D(const T val[3]);

	template<typename U>
	clafVector3D(const clafVector3D<U> &rhs)
	{
		*this = rhs;
	}

	template<typename U>
	inline clafVector3D &operator=(const clafVector3D<U> &rhs)
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

	clafVector3D &operator = (const clafVector3D &v1);
	clafVector3D &operator -= (const clafVector3D &v1);
	clafVector3D &operator += (const clafVector3D &v1);
	clafVector3D &operator *= (const clafVector3D &v1);
	clafVector3D &operator *= (CLAFdouble s);
	clafVector3D &operator /= (const clafVector3D &v1);
	clafVector3D &operator /= (CLAFdouble s);


	CLAFboolean isZero(T epsilon) const;
	CLAFdouble distance(const clafVector3D& v1)const;
	CLAFdouble distanceSqr(const clafVector3D& v1) const;

	CLAFdouble magnitude() const;
	CLAFdouble magnitude(const clafVector3D& v)const;
	void normalize();
	CLAFdouble dotProduct(const clafVector3D& v1) const;
	clafVector3D crossProduct(const clafVector3D& v1)const;

	int getLongestAxisComponent() const;
	int getSecondLongestAxisComponent() const;
	int getShortestAxisComponent() const;


private:
	T m_p[3];
};

typedef clafVector3D<CLAFfloat>  clafVector3Df;
typedef clafVector3D<CLAFdouble> clafVector3Dd;

#include "clafVector.inl"

#else

typedef CLAFfloat clafVector3Df[3];
typedef CLAFdouble clafVector3Dd[3];

#endif // __cplusplus

#ifdef __cplusplus
extern "C"
{
#endif


__inline void clafVecSet(CLAFdouble *vec,
	CLAFdouble x,
	CLAFdouble y,
	CLAFdouble z)
{
	vec[0] = x;
	vec[1] = y;
	vec[2] = z;
}

__inline CLAFdouble clafVecMagnitude(const CLAFdouble*vec)
{
	return (sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]));
}

__inline CLAFboolean clafVecIsZero(const CLAFdouble *vec, CLAFdouble epsilon)
{
	return((fabs(vec[0]) < epsilon) &&
			(fabs(vec[1]) < epsilon) &&
			(fabs(vec[2]) < epsilon));
}

__inline void clafVecAdd(CLAFdouble*res,
	const CLAFdouble*vec1,
	const CLAFdouble*vec2)
{
	res[0] = vec1[0] + vec2[0];
	res[1] = vec1[1] + vec2[1];
	res[2] = vec1[2] + vec2[2];
}

__inline void clafVecSubtract(CLAFdouble*res,
	const CLAFdouble*vec1,
	const CLAFdouble*vec2)
{
	res[0] = vec1[0] - vec2[0];
	res[1] = vec1[1] - vec2[1];
	res[2] = vec1[2] - vec2[2];
}

__inline void clafVecScale(CLAFdouble *res,
	const CLAFdouble *vec,
	CLAFdouble s)
{
	res[0] = vec[0] * s;
	res[1] = vec[1] * s;
	res[2] = vec[2] * s;
}

__inline void clafVecScaleInPlace(CLAFdouble *vec,
	CLAFdouble s)
{
	vec[0] = vec[0] * s;
	vec[1] = vec[1] * s;
	vec[2] = vec[2] * s;
}

__inline void clafVecScaleNonUniform(CLAFdouble *res,
	const CLAFdouble *vec,
	const CLAFdouble *s)
{
	res[0] = vec[0] * s[0];
	res[1] = vec[1] * s[1];
	res[2] = vec[2] * s[2];
}

__inline void clafVecScaleNonUniformInPlace(CLAFdouble *vec,
	const CLAFdouble *s)
{
	vec[0] = vec[0] * s[0];
	vec[1] = vec[1] * s[1];
	vec[2] = vec[2] * s[2];
}

__inline void clafVecNormalize(CLAFdouble *res,
	const CLAFdouble *vec)
{
	double mag = clafVecMagnitude(vec);
	if (mag==0)
	{
		return;
	}
	clafVecScale(res, vec, 1.0 / mag);
}

__inline void clafVecNormalizeInPlace(CLAFdouble *vec)
{
	double mag = clafVecMagnitude(vec);
	if (mag == 0) return;
	clafVecScaleInPlace(vec, 1.0 / mag);
}

__inline void clafVecCrossProduct(CLAFdouble *res,
	const CLAFdouble *vec1,
	const CLAFdouble *vec2)
{
	res[0] = vec1[1] * vec2[2] - vec1[2] * vec2[1];
	res[1] = vec1[2] * vec2[0] - vec1[0] * vec2[2];
	res[2] = vec1[0] * vec2[1] - vec1[1] * vec2[0];
}

__inline CLAFdouble clafVecDotProduct(const CLAFdouble *vec1,
	const CLAFdouble *vec2)
{
	return (vec1[0] * vec2[0] + vec1[1] * vec2[1] + vec1[2] * vec2[2]);
}

__inline CLAFdouble clafVecDistance(const CLAFdouble *vec1,
	const CLAFdouble *vec2)
{
	clafVector3Dd vec3;
	clafVecSubtract(vec3, vec1, vec2);
	return clafVecMagnitude(vec3);
}


#ifdef __cplusplus
}
#endif

#endif // !CLAFVECTOR_H_
