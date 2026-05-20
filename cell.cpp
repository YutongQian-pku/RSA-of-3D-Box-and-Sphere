#include "cell.h"
#include "particle.h"
#include "voxel.h"

using namespace std;
//**************cell相关函数**************************
Cell::Cell() { num = 0; }
Cell::Cell(double ccharacter_L, double L) {
	num = 0;
	cell_L = L;
	cellcharacter_L = ccharacter_L;
	cell_n = (int)(cell_L / cellcharacter_L);
	cell_l = cell_L / (double)(cell_n);
	cell_N = cell_n * cell_n * cell_n;
}
Cell::~Cell() {}

void initial_grid(Cell* cell) {
	int N = cell[0].cell_N;
	int n = cell[0].cell_n;

	int x, y, z;
	int ix, iy, iz, id, iid;

	for (iz = 0; iz < n; iz++) {
		for (iy = 0; iy < n; iy++) {
			for (ix = 0; ix < n; ix++) {
				iid = ix + n * iy + n * n * iz;

				z = iz - 1; if (z < 0) { z = n - 1; cell[iid].boundary_tag = 6; cell[iid].tag_num++; }

				y = iy - 1; if (y < 0) { y = n - 1; cell[iid].boundary_tag = 4; cell[iid].tag_num++; }
				x = ix - 1; if (x < 0) { x = n - 1; cell[iid].boundary_tag = 2; cell[iid].tag_num++; }
				id = x + n * y + n * n * z;
				cell[iid].cell_id[0] = id;
				//**************************************************
				x = ix;
				id = x + n * y + n * n * z;
				cell[iid].cell_id[1] = id;
				//**************************************************
				x = ix + 1; if (x > n - 1) { x = 0; cell[iid].boundary_tag = 1; cell[iid].tag_num++; }
				id = x + n * y + n * n * z;
				cell[iid].cell_id[2] = id;
				//**************************************************
				y = iy;
				x = ix - 1; if (x < 0) { x = n - 1; cell[iid].boundary_tag = 2; cell[iid].tag_num++; }
				id = x + n * y + n * n * z;
				cell[iid].cell_id[3] = id;
				//**************************************************
				x = ix;
				id = x + n * y + n * n * z;
				cell[iid].cell_id[4] = id;
				//**************************************************
				x = ix + 1; if (x > n - 1) { x = 0; cell[iid].boundary_tag = 1; cell[iid].tag_num++; }
				id = x + n * y + n * n * z;
				cell[iid].cell_id[5] = id;
				//**************************************************
				y = iy + 1; if (y > n - 1) { y = 0; cell[iid].boundary_tag = 3; cell[iid].tag_num++; }
				x = ix - 1; if (x < 0) { x = n - 1; cell[iid].boundary_tag = 2; cell[iid].tag_num++; }
				id = x + n * y + n * n * z;
				cell[iid].cell_id[6] = id;
				//**************************************************
				x = ix;
				id = x + n * y + n * n * z;
				cell[iid].cell_id[7] = id;
				//**************************************************
				x = ix + 1; if (x > n - 1) { x = 0; cell[iid].boundary_tag = 1; cell[iid].tag_num++; }
				id = x + n * y + n * n * z;
				cell[iid].cell_id[8] = id;
				//**************************************************

				z = iz;

				y = iy - 1; if (y < 0) { y = n - 1; cell[iid].boundary_tag = 4; cell[iid].tag_num++; }
				x = ix - 1; if (x < 0) { x = n - 1; cell[iid].boundary_tag = 2; cell[iid].tag_num++; }
				id = x + n * y + n * n * z;
				cell[iid].cell_id[9] = id;
				//**************************************************
				x = ix;
				id = x + n * y + n * n * z;
				cell[iid].cell_id[10] = id;
				//**************************************************
				x = ix + 1; if (x > n - 1) { x = 0; cell[iid].boundary_tag = 1; cell[iid].tag_num++; }
				id = x + n * y + n * n * z;
				cell[iid].cell_id[11] = id;
				//**************************************************
				y = iy;
				x = ix - 1; if (x < 0) { x = n - 1; cell[iid].boundary_tag = 2; cell[iid].tag_num++; }
				id = x + n * y + n * n * z;
				cell[iid].cell_id[12] = id;
				//**************************************************
				//x = ix;             //该处对应中心点
				//id = x + n * y + n * n * z;
				//cell[ix + n * iy + n * n * iz].cell_id[5] = id;
				//**************************************************
				x = ix + 1; if (x > n - 1) { x = 0; cell[iid].boundary_tag = 1; cell[iid].tag_num++; }
				id = x + n * y + n * n * z;
				cell[iid].cell_id[13] = id;
				//**************************************************
				y = iy + 1; if (y > n - 1) { y = 0; cell[iid].boundary_tag = 3; cell[iid].tag_num++; }
				x = ix - 1; if (x < 0) { x = n - 1; cell[iid].boundary_tag = 2; cell[iid].tag_num++; }
				id = x + n * y + n * n * z;
				cell[iid].cell_id[14] = id;
				//**************************************************
				x = ix;
				id = x + n * y + n * n * z;
				cell[iid].cell_id[15] = id;
				//**************************************************
				x = ix + 1; if (x > n - 1) { x = 0; cell[iid].boundary_tag = 1; cell[iid].tag_num++; }
				id = x + n * y + n * n * z;
				cell[iid].cell_id[16] = id;
				//**************************************************

				z = iz + 1; if (z > n - 1) { z = 0; cell[iid].boundary_tag = 5; cell[iid].tag_num++; }

				y = iy - 1; if (y < 0) { y = n - 1; cell[iid].boundary_tag = 4; cell[iid].tag_num++; }
				x = ix - 1; if (x < 0) { x = n - 1; cell[iid].boundary_tag = 2; cell[iid].tag_num++; }
				id = x + n * y + n * n * z;
				cell[iid].cell_id[17] = id;
				//**************************************************
				x = ix;
				id = x + n * y + n * n * z;
				cell[iid].cell_id[18] = id;
				//**************************************************
				x = ix + 1; if (x > n - 1) { x = 0; cell[iid].boundary_tag = 1; cell[iid].tag_num++; }
				id = x + n * y + n * n * z;
				cell[iid].cell_id[19] = id;
				//**************************************************
				y = iy;
				x = ix - 1; if (x < 0) { x = n - 1; cell[iid].boundary_tag = 2; cell[iid].tag_num++; }
				id = x + n * y + n * n * z;
				cell[iid].cell_id[20] = id;
				//**************************************************
				x = ix;
				id = x + n * y + n * n * z;
				cell[iid].cell_id[21] = id;
				//**************************************************
				x = ix + 1; if (x > n - 1) { x = 0; cell[iid].boundary_tag = 1; cell[iid].tag_num++; }
				id = x + n * y + n * n * z;
				cell[iid].cell_id[22] = id;
				//**************************************************
				y = iy + 1; if (y > n - 1) { y = 0; cell[iid].boundary_tag = 3; cell[iid].tag_num++; }
				x = ix - 1; if (x < 0) { x = n - 1; cell[iid].boundary_tag = 2; cell[iid].tag_num++; }
				id = x + n * y + n * n * z;
				cell[iid].cell_id[23] = id;
				//**************************************************
				x = ix;
				id = x + n * y + n * n * z;
				cell[iid].cell_id[24] = id;
				//**************************************************
				x = ix + 1; if (x > n - 1) { x = 0; cell[iid].boundary_tag = 1; cell[iid].tag_num++; }
				id = x + n * y + n * n * z;
				cell[iid].cell_id[25] = id;
				//**************************************************
			}
		}
	}
}

int get_cell_id(Cell &cell, double x, double y, double z) {
	int ix, iy, iz, id;
	double l = cell.cell_l;
	int n = cell.cell_n;

	ix = (int)(x / l);
	iy = (int)(y / l);
	iz = (int)(z / l);
	if (ix > n - 1) ix = n - 1;//个人感觉貌似以下6行没必要
	if (iy > n - 1) iy = n - 1;
	if (iz > n - 1) iz = n - 1;
	if (ix < 0) ix = 0;
	if (iy < 0) iy = 0;
	if (iz < 0) iz = 0;

	id = ix + n * iy + n * n * iz;
	return id;
}

bool check_over(Cell* cell, Particle* p, voxel* vox) {
	if (vox[get_voxel_id(p->center, vox[0])].is_occupy == true)return true;//flag为true时，为相交情形，不能放入
	int id;
	double box_l = cell[0].cell_L;
	
	//报错原因：其实是在下面这行代码中，该代码试图将cell[0]传入而不是传入指针，这就导致了复制操作，而cell附属的particle也被复制了，违背了unique的原则
	id = get_cell_id(cell[0], p->center.x, p->center.y, p->center.z);

	for (const auto& particlePtr : cell[id].particles) {
		auto p1 = particlePtr->clone();
		if (p->intersects(p1.get())){
			return true;
		}
	}

	////改动：区分边界位置    1：x+，2：x-，3：y+，4：y-，5：z+，6：z-
	int x_min[9] = { 0,3,6,9,12,14,17,20,23 };
	int x_max[9] = { 2,5,8,11,13,16,19,22,25 };
	int y_min[9] = { 0,1,2,9,10,11,17,18,19 };
	int y_max[9] = { 6,7,8,14,15,16,23,24,25 };
	int z_min[9] = { 0,1,2,3,4,5,6,7,8 };
	int z_max[9] = { 17,18,19,20,21,22,23,24,25 };
	int a = cell[id].boundary_tag;
	int b = cell[id].tag_num;
	if (a == 0) {
		for (int k = 0; k < 26; k++) {
			for (const auto& particlePtr : cell[cell[id].cell_id[k]].particles) {
				auto p1 = particlePtr->clone();
				if (p->intersects(p1.get())) {
					return true;
				}
			}
		}
	}
	else if (a == 5 && b == 1) {//z_max
		for (int i = 0; i < 9; i++) {
			for (const auto& particlePtr : cell[cell[id].cell_id[z_max[i]]].particles) {
				auto p1 = particlePtr->clone();
				p1->translate(0, 0, 1.0f * box_l);
				if (p->intersects(p1.get())) return true;
			}
		}
		for (int k = 0; k < 26; k++) {
			for (const auto& particlePtr : cell[cell[id].cell_id[k]].particles) {
				auto p1 = particlePtr->clone();
				if (p->intersects(p1.get())) return true;
			}
		}
	}
	else if (a == 4 && b == 3) {//y_min
		for (int i = 0; i < 9; i++) {
			for (const auto& particlePtr : cell[cell[id].cell_id[y_min[i]]].particles) {
				auto p1 = particlePtr->clone();
				p1->translate(0, -1.0f * box_l, 0);
				if (p->intersects(p1.get()))return true;
			}
		}
		for (int k = 0; k < 26; k++) {
			for (const auto& particlePtr : cell[cell[id].cell_id[k]].particles) {
				auto p1 = particlePtr->clone();
				if (p->intersects(p1.get())) return true;
			}
		}
	}
	else if (a == 3 && b == 3) {//y_max
		for (int i = 0; i < 9; i++) {
			for (const auto& particlePtr : cell[cell[id].cell_id[y_max[i]]].particles) {
				auto p1 = particlePtr->clone();
				p1->translate(0, 1.0f * box_l, 0);
				if (p->intersects(p1.get())) return true;
			}
		}
		for (int k = 0; k < 26; k++) {
			for (const auto& particlePtr : cell[cell[id].cell_id[k]].particles) {
				auto p1 = particlePtr->clone();
				if (p->intersects(p1.get())) return true;
			}
		}
	}
	else if (a == 2 && b == 9) {//x_min
		for (int i = 0; i < 9; i++) {
			for (const auto& particlePtr : cell[cell[id].cell_id[x_min[i]]].particles) {
				auto p1 = particlePtr->clone();
				p1->translate(-1.0f * box_l, 0, 0);
				if (p->intersects(p1.get())) return true;
			}
		}
		for (int k = 0; k < 26; k++) {
			for (const auto& particlePtr : cell[cell[id].cell_id[k]].particles) {
				auto p1 = particlePtr->clone();
				if (p->intersects(p1.get())) return true;
			}
		}
	}
	else if (a == 1 && b == 9) {//x_max
		for (int i = 0; i < 9; i++) {
			for (const auto& particlePtr : cell[cell[id].cell_id[x_max[i]]].particles) {
				auto p1 = particlePtr->clone();
				p1->translate(1.0f * box_l, 0, 0);
				if (p->intersects(p1.get())) return true;
			}
		}
		for (int k = 0; k < 26; k++) {
			for (const auto& particlePtr : cell[cell[id].cell_id[k]].particles) {
				auto p1 = particlePtr->clone();
				if (p->intersects(p1.get())) return true;
			}
		}
	}
	else if (a == 6 && b == 1) {//z_min
		for (int i = 0; i < 9; i++) {
			for (const auto& particlePtr : cell[cell[id].cell_id[z_min[i]]].particles) {
				auto p1 = particlePtr->clone();
				p1->translate(0, 0, -1.0f * box_l);
				if (p->intersects(p1.get())) return true;
			}
		}
		for (int k = 0; k < 26; k++) {
			for (const auto& particlePtr : cell[cell[id].cell_id[k]].particles) {
				auto p1 = particlePtr->clone();
				if (p->intersects(p1.get())) return true;
			}
		}
	}
	else {
		for (int k = 0; k < 26; ++k) {
			for (int m = -1; m < 2; ++m) {
				for (int n1 = -1; n1 < 2; ++n1) {
					for (int q = -1; q < 2; q++) {
						for (const auto& particlePtr : cell[cell[id].cell_id[k]].particles) {
							auto p1 = particlePtr->clone();
							p1->translate(m* box_l, n1* box_l, q* box_l);
							if (p->intersects(p1.get())) return true;
						}
					}
				}
			}
		}
	}
	return false;//不相交
}