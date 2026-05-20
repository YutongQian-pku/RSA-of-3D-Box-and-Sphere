#include <cstdio>
#include <fstream>
#include <iomanip>
#include "voxel.h"
#include "particle.h"
using namespace std;

voxel::voxel() {}
voxel::voxel(double character_L, double box_l) {
	double a = character_L / 3.0;
	voxel_n = static_cast<long long>(box_l / a);
	voxel_N = voxel_n * voxel_n * voxel_n;
	length = box_l / voxel_n;

}
voxel::~voxel() {}

long long get_voxel_id(Point3d loc, voxel v) {
	long long ix, iy, iz, id;
	long long n = v.voxel_n;

	ix = static_cast<long long>(loc.x / v.length);
	iy = static_cast<long long>(loc.y / v.length);
	iz = static_cast<long long>(loc.z / v.length);
	if (ix > n - 1) ix = n - 1;
	if (iy > n - 1) iy = n - 1;
	if (iz > n - 1) iz = n - 1;
	if (ix < 0) ix = 0;
	if (iy < 0) iy = 0;
	if (iz < 0) iz = 0;

	id = ix + n * iy + n * n * iz;
	return id;
}

void initial_vox(voxel* v) {
    // 循环变量i改为long long，匹配voxel_N的类型
    for (long long i = 0; i < v[0].voxel_N; i++) {
        long long iz = static_cast<long long>(static_cast<double>(i) / (static_cast<double>(v[0].voxel_n * v[0].voxel_n)));
        long long iy = static_cast<long long>(static_cast<double>(i - iz * v[0].voxel_n * v[0].voxel_n) / static_cast<double>(v[0].voxel_n));
        long long ix = i - iy * v[0].voxel_n - iz * v[0].voxel_n * v[0].voxel_n;

        v[i].center.x = (static_cast<double>(ix) + 0.5) * v[i].length;
        v[i].center.y = (static_cast<double>(iy) + 0.5) * v[i].length;
        v[i].center.z = (static_cast<double>(iz) + 0.5) * v[i].length;
    }
}

// 第三个参数num改为long long（对应voxel_N）
void scr_vox(double box_l, voxel* vox, long long num, int cycle) {
    char cha[40];
    sprintf(cha, "packing_voxel_%d.scr", cycle);
    ofstream scr(cha);
    scr << fixed << setprecision(8);

    scr << "-osnap off" << endl;
    scr << "vscurrent 2" << endl;
    scr << "color 2" << endl;

    // 循环变量i改为long long
    for (long long i = 0; i < num; ++i) {
        if (vox[i].is_occupy == false) {
            scr << "box c " << vox[i].center.x << "," << vox[i].center.y << "," << vox[i].center.z << " c " << vox[i].length << endl;
        }
    }
    
    scr << "zoom e" << endl;
    scr << "vpoint 0,0,1" << endl;
    scr.close();
}
