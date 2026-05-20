#ifndef _CELL_H_
#define _CELL_H_

#include <vector>
#include <memory>
using namespace std;
#include"particle.h"

class Cell {
public:
	Cell(double ccharacter_L, double box_l);
	//第一个参数：特征长度；第二个参数：packing的边长
	Cell();
	~Cell();

	//cell parameter
	double cell_L;//cell的总边长，即packing空间的边长
	double cell_l;//cell的边长
	int cell_n;//一条边上cell的个数
	int cell_N;//总的cell个数
	double cellcharacter_L;

	//for search
	int num; //cell内的颗粒数
	vector<unique_ptr<Particle>> particles;

	int cell_id[26];

	int boundary_tag = 0;//0：内部，1：x+，2：x-，3：y+，4：y-，5：z+，6：z-
	int tag_num = 0;//打上tag标签的次数

	Cell& operator=(Cell&& other)noexcept {
		if (this != &other) {
			particles = std::move(other.particles);
			cell_L = other.cell_L;
			cellcharacter_L = other.cellcharacter_L;
			for (int i = 0; i <26; i++) {
				cell_id[i] = other.cell_id[i];
			}
			cell_l = other.cell_l;
			cell_N = other.cell_N;
			cell_n = other.cell_n;
			num = other.num;
			tag_num = other.tag_num;
		}
		return *this;
	}
};

void initial_grid(Cell* cell);
int get_cell_id(Cell &cell, double x, double y, double z);
bool check_over(Cell* cell, Particle* originalParticle, voxel* vox);


#endif