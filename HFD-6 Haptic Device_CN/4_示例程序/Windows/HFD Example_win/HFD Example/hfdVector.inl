/************************************************************************/
/*   Module Name:	hfdVector.inl
	 Description:	Inline implementation of 3D Vector Class            */
/************************************************************************/

#ifndef HFDVECTOR_INL_
#define HFDVECTOR_INL_

template<class T>
inline hfdVector3D<T>::hfdVector3D()
{
	m_p[0] = m_p[1] = m_p[2] = 0;
}

template<class T>
inline hfdVector3D<T>::hfdVector3D(T x, T y, T z)
{
	m_p[0] = x;
	m_p[1] = y;
	m_p[2] = z;
}

template<class T>
inline void hfdVector3D<T>::set(T x, T y, T z)
{
	m_p[0] = x;
	m_p[1] = y;
	m_p[2] = z;
}

template<class T>
inline hfdVector3D<T>::hfdVector3D(const T val[3])
{
	m_p[0] = val[0];
	m_p[1] = val[1];
	m_p[2] = val[2];
}

template<class T>
inline const T& hfdVector3D<T>::operator [](int i)const
{
	return m_p[i];
}

template<class T>
inline T& hfdVector3D<T>::operator [](int i)
{
	return m_p[i];
}

template<class T>
inline hfdVector3D<T>& hfdVector3D<T>::operator=(const hfdVector3D<T>&v1)
{
	m_p[0] = v1[0];
	m_p[1] = v1[1];
	m_p[2] = v1[2];
	return *this;
}

template <class T>
inline hfdVector3D<T> &hfdVector3D<T>::operator -= (const hfdVector3D<T> &v1)
{
	m_p[0] -= v1[0];
	m_p[1] -= v1[1];
	m_p[2] -= v1[2];
	return *this;
}

template <class T>
inline hfdVector3D<T> &hfdVector3D<T>::operator += (const hfdVector3D<T> &v1)
{
	m_p[0] += v1[0];
	m_p[1] += v1[1];
	m_p[2] += v1[2];
	return *this;
}

template <class T>
inline hfdVector3D<T> &hfdVector3D<T>::operator *= (const hfdVector3D<T> &v1)
{
	m_p[0] *= v1[0];
	m_p[1] *= v1[1];
	m_p[2] *= v1[2];
	return *this;
}

template <class T>
inline hfdVector3D<T> &hfdVector3D<T>::operator *= (HFDdouble s)
{
	m_p[0] *= s;
	m_p[1] *= s;
	m_p[2] *= s;
	return *this;
}

template <class T>
inline hfdVector3D<T> &hfdVector3D<T>::operator /= (const hfdVector3D<T> &v1)
{
	if (v1[0] == 0 || v1[1] == 0 || v1[2] == 0) return *this;

	m_p[0] /= v1[0];
	m_p[1] /= v1[1];
	m_p[2] /= v1[2];
	return *this;
}

template <class T>
inline hfdVector3D<T> &hfdVector3D<T>::operator /= (HFDdouble s)
{
	if (s == 0) return *this;

	m_p[0] /= s;
	m_p[1] /= s;
	m_p[2] /= s;
	return *this;
}

template <class T>
inline hfdVector3D<T> operator - (const hfdVector3D<T> &v1)
{
	return hfdVector3D<T>(-v1[0],
		                  -v1[1],
		                  -v1[2]);
}

template <class T>
inline hfdVector3D<T> operator - (const hfdVector3D<T> &v1,
	const hfdVector3D<T> &v2)
{
	return hfdVector3D<T>(v1[0] - v2[0],
						  v1[1] - v2[1],
						  v1[2] - v2[2]);
}

template <class T>
inline hfdVector3D<T> operator+ (const hfdVector3D<T> &v1,
	const hfdVector3D<T> &v2)
{
	return hfdVector3D<T>(v1[0] + v2[0],
						  v1[1] + v2[1],
						  v1[2] + v2[2]);
}

template <class T>
inline hfdVector3D<T> operator * (const hfdVector3D<T> &v1,
								  const hfdVector3D<T> &v2)
{
	return hfdVector3D<T>(v1[0] * v2[0],
						  v1[1] * v2[1],
						  v1[2] * v2[2]);
}

template <class T>
inline hfdVector3D<T> operator * (const hfdVector3D<T> &v1, HFDdouble s)
{
	return hfdVector3D<T>(s * v1[0],
						  s * v1[1],
						  s * v1[2]);
}

template <class T>
inline hfdVector3D<T> operator * (HFDdouble s, const hfdVector3D<T> &v1)
{
	return hfdVector3D<T>(s * v1[0],
						  s * v1[1],
						  s * v1[2]);
}

template <class T>
inline hfdVector3D<T> operator / (const hfdVector3D<T> &v1,const hfdVector3D<T> &v2)
{
	return hfdVector3D<T>(v1[0] / v2[0],
						  v1[1] / v2[1],
						  v1[2] / v2[2]);
}

template <class T>
inline hfdVector3D<T> operator / (const hfdVector3D<T> &v1, HFDdouble s)
{
	if (s == 0) return v1;

	return hfdVector3D<T>(v1[0] / s,
						  v1[1] / s,
						  v1[2] / s);
}

template <class T>
inline int operator == (const hfdVector3D<T> &v1, const hfdVector3D<T> &v2)
{
	return (v1[0] == v2[0] &&
			v1[1] == v2[1] &&
			v1[2] == v2[2]);
}

template <class T>
inline int operator != (const hfdVector3D<T> &v1, const hfdVector3D<T> &v2)
{
	return (v1[0] != v2[0] ||
			v1[1] != v2[1] ||
			v1[2] != v2[2]);
}

template<class T>
inline std::ostream& operator<<(std::ostream& os, const hfdVector3D<T> &v)
{
	return os << v[0] << " " << v[1] << " " << v[2];
}

template<class T>
inline std::istream& operator>>(std::istream& is, hfdVector3D<T>& v)
{
	return is >> v[0] >> v[1] >> v[2];
}

template<class T>
inline HFDboolean hfdVector3D<T>::isZero(T epsilon) const
{
	return ((fabs(m_p[0])<epsilon)&&
		    (fabs(m_p[1])<epsilon)&&
		    (fabs(m_p[2])<epsilon));
}

template<class T>
inline HFDdouble hfdVector3D<T>::distance(const hfdVector3D& v1)const
{
	return(*this - v1).magnitude();
}

template<class T>
inline HFDdouble hfdVector3D<T>::distanceSqr(const hfdVector3D& v1) const
{
	hfdVector3D diff = *this - v1;
	return diff.dotProduct(diff);
}

template<class T>
inline HFDdouble hfdVector3D<T>::magnitude() const
{
	return(sqrt(m_p[0] * m_p[0] + m_p[1] * m_p[1] + m_p[2] * m_p[2]));
}

template<class T>
inline HFDdouble hfdVector3D<T>::magnitude(const hfdVector3D<T>& v)const
{
	return v.magnitude();
}

template<class T>
inline void hfdVector3D<T>::normalize()
{
	HFDdouble m = magnitude();
	*this /= m;
}

template<class T>
inline hfdVector3D<T> normalize(const hfdVector3D<T> &v)
{
	HFDdouble m = v.magnitude();
	if (m == 0) return v;
	return v / m;
}

template <class T>
inline HFDdouble dotProduct(const hfdVector3D<T> &v1, const hfdVector3D<T> &v2)
{
	return (v1[0] * v2[0] +
			v1[1] * v2[1] +
			v1[2] * v2[2]);
}

template <class T>
inline hfdVector3D<T> crossProduct(const hfdVector3D<T> &v1,
								   const hfdVector3D<T> &v2)
{
	hfdVector3D<T> result(v1[1] * v2[2] - v1[2] * v2[1],
						  v1[2] * v2[0] - v1[0] * v2[2],
						  v1[0] * v2[1] - v1[1] * v2[0]);
	return result;
}


// project v1 onto to v2
template<class T>
inline hfdVector3D<T> project(const hfdVector3D<T> &v1, const hfdVector3D<T> &v2)
{
	return(dotProduct(v1, v2) / dotProduct(v2, v2))*v2;
}

template <class T>
inline HFDdouble hfdVector3D<T>::dotProduct(const hfdVector3D<T>& v1)const
{
	return dotProduct(*this, v1);
}

template <class T>
inline hfdVector3D<T> hfdVector3D<T>::crossProduct(const hfdVector3D<T> &v1) const
{
	return ::crossProduct(*this, v1);
}

template<class T>
inline int hfdVector3D<T>::getLongestAxisComponent()const
{
	if (fabs(m_p[2]) >= fabs(m_p[1]))
	{
		if (fabs(m_p[2]) >= fabs(m_p[0]))
		{
			return 2;
		}
		else
		{
			return 0;
		}
	}
	else if (fabs(m_p[1]) >= fabs(m_p[0]))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

template <class T>
inline int hfdVector3D<T>::getSecondLongestAxisComponent() const
{
	int longestAxis = getLongestAxisComponent();
	if (longestAxis == 0)
	{
		if (fabs(m_p[2]) >= fabs(m_p[1]))
			return 2;
		else
			return 1;
	}
	else if (longestAxis == 1)
	{
		if (fabs(m_p[2]) >= fabs(m_p[0]))
			return(2);
		else
			return(0);
	}
	else
	{
		if (fabs(m_p[1]) >= fabs(m_p[0]))
			return 1;
		else
			return 0;
	}
}

template <class T>
inline int hfdVector3D<T>::getShortestAxisComponent() const
{
	if (fabs(m_p[0]) <= fabs(m_p[1]))
	{
		if (fabs(m_p[0]) <= fabs(m_p[2]))
		{
			return 0;
		}
		else
		{
			return 2;
		}
	}
	else if (fabs(m_p[1]) <= fabs(m_p[2]))
	{
		return 1;
	}
	else
	{
		return 2;
	}
}


#endif