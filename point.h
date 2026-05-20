#ifndef _POINT_H_
#define _POINT_H_

#include<math.h>

class Point
{
public:
	double x;
	double y;
	Point(const Point& a) :x(a.x), y(a.y) {}
	Point(double x = 0.0f, double y = 0.0f) : x(x), y(y) {}
	Point& operator=(const Point& a) {
		x = a.x; y = a.y;
		return *this;
	}
	//重载比较运算符
	bool operator ==(const Point& a)const {
		return x == a.x && y == a.y;
	}
	bool operator !=(const Point& a) const {
		return x != a.x || y != a.y;
	}
	//加减法
	Point operator +(const Point& a) const {
		return Point(x + a.x, y + a.y);
	}
	Point operator -(const Point& a) const {
		return Point(x - a.x, y - a.y);
	}
	//点乘
	double operator *(const Point& a) const {
		return x * a.x + y * a.y;
	}
	//标量乘、除法
	Point operator *(double a) const {
		return Point(x * a, y * a);
	}
	Point operator /(double a) const {
		double oneOverA = 1.0f / a; // 没有对除零检查
		return Point(x * oneOverA, y * oneOverA);
	}

	void translate(double ix, double iy) {
		x += ix;
		y += iy;
	}
};

inline double PointMag(const Point& a) {
	return sqrt(a.x * a.x + a.y * a.y);
}

class Point3d
{
public:
	double x;
	double y;
	double z;
	Point3d(const Point3d& a) :x(a.x), y(a.y), z(a.z) {}
	Point3d(double x = 0.0f, double y = 0.0f, double z = 0.0f) : x(x), y(y), z(z) {}
	Point3d& operator=(const Point3d& a) {
		x = a.x; y = a.y; z = a.z;
		return *this;
	}
	//重载比较运算符
	bool operator ==(const Point3d& a)const {
		return x == a.x && y == a.y && z == a.z;
	}
	bool operator !=(const Point3d& a) const {
		return x != a.x || y != a.y || z != a.z;
	}
	//加减法
	Point3d operator +(const Point3d& a) const {
		return Point3d(x + a.x, y + a.y, z + a.z);
	}
	Point3d operator -(const Point3d& a) const {
		return Point3d(x - a.x, y - a.y, z - a.z);
	}
	//点乘
	double operator *(const Point3d& a) const {
		return x * a.x + y * a.y + z * a.z;
	}
	//标量乘、除法
	Point3d operator *(double a) const {
		return Point3d(x * a, y * a, z * a);
	}
	Point3d operator /(double a) const {
		double oneOverA = 1.0f / a; // 没有对除零检查
		return Point3d(x * oneOverA, y * oneOverA, z * oneOverA);
	}

	void translate(double ix, double iy, double iz) {
		x += ix;
		y += iy;
		z += iz;
	}
};

inline double PointMag3d(const Point3d& a) {
	return sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
}

#endif // !_POINT_H_

