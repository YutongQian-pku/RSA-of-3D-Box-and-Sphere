#ifndef _VOXEL_H_
#define _VOXEL_H_

#include"point.h"

using namespace std;

class Point3d;

class voxel {
public:
	voxel(double character_L, double box_l);//初始化整体属性
	voxel();
	~voxel();
	Point3d center;
	Point3d v1, v2, v3, v4, v5, v6, v7, v8;
	bool is_occupy = false;//默认为未占用，false
	long long voxel_n;
	long long int voxel_N;
	double length;
};

long long get_voxel_id(Point3d loc, voxel v);
void initial_vox(voxel* v);
void scr_vox(double box_l, voxel* vox, long long num, int cycle);

#endif