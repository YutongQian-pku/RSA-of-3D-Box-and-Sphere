#ifndef _PARTICLE_H_
#define _PARTICLE_H_

#include <memory>
#include <cmath>
#include <vector>

#define PI 3.1415926535898

using namespace std;

#include "voxel.h"
#include "point.h"

class voxel;
class Sphere;
class Box;

// 三维刚体姿态：ux、uy、uz 是长方体局部 x/y/z 轴在全局坐标中的单位方向。
struct Orientation3d {
    Point3d ux;
    Point3d uy;
    Point3d uz;

    Orientation3d(Point3d x_axis = Point3d(1.0, 0.0, 0.0),
                  Point3d y_axis = Point3d(0.0, 1.0, 0.0),
                  Point3d z_axis = Point3d(0.0, 0.0, 1.0))
        : ux(x_axis), uy(y_axis), uz(z_axis) {}
};

// ─────────────────────────────────────────────────────────────────
//  抽象基类 Particle
// ─────────────────────────────────────────────────────────────────
class Particle {
public:
    Point3d center;
    double character_L; // 用于 cell / voxel 网格尺度划分
    double outer_r;     // 外接球半径（快速排斥检测）
    double inner_r;     // 内切球半径（voxel 占据扩展量）

    virtual void translate(double x, double y, double z) = 0;
    virtual bool intersects(Particle* p) const = 0;
    virtual std::unique_ptr<Particle> clone() const = 0;
    virtual bool is_onboundary(double box_l) const = 0;
    virtual bool point_is_in(Point3d v) const = 0;

    // 判断 voxel 的 8 个顶点是否均在粒子内部。
    virtual void voxel_is_in(voxel* v) const {
        Point3d v1, v2, v3, v4, v5, v6, v7, v8;
        v1.x = v->center.x - v->length * 0.5;
        v1.y = v->center.y - v->length * 0.5;
        v1.z = v->center.z - v->length * 0.5;

        v2.x = v->center.x + v->length * 0.5;
        v2.y = v->center.y - v->length * 0.5;
        v2.z = v->center.z - v->length * 0.5;

        v3.x = v->center.x - v->length * 0.5;
        v3.y = v->center.y + v->length * 0.5;
        v3.z = v->center.z - v->length * 0.5;

        v4.x = v->center.x + v->length * 0.5;
        v4.y = v->center.y + v->length * 0.5;
        v4.z = v->center.z - v->length * 0.5;

        v5.x = v->center.x - v->length * 0.5;
        v5.y = v->center.y - v->length * 0.5;
        v5.z = v->center.z + v->length * 0.5;

        v6.x = v->center.x + v->length * 0.5;
        v6.y = v->center.y - v->length * 0.5;
        v6.z = v->center.z + v->length * 0.5;

        v7.x = v->center.x - v->length * 0.5;
        v7.y = v->center.y + v->length * 0.5;
        v7.z = v->center.z + v->length * 0.5;

        v8.x = v->center.x + v->length * 0.5;
        v8.y = v->center.y + v->length * 0.5;
        v8.z = v->center.z + v->length * 0.5;

        int a = point_is_in(v1) + point_is_in(v2) + point_is_in(v3) + point_is_in(v4)
              + point_is_in(v5) + point_is_in(v6) + point_is_in(v7) + point_is_in(v8);
        if (a == 8) v->is_occupy = true;
    }

    // 默认实现：基于外接球 AABB 扫描 voxel。Box 会重载为紧致 AABB。
    virtual void cal_occupy_single(voxel* v, double box_l) const {
        double up    = center.z + outer_r;
        double down  = center.z - outer_r;
        double left  = center.y - outer_r;
        double right = center.y + outer_r;
        double front = center.x + outer_r;
        double back  = center.x - outer_r;

        Point3d p1, p2, p3, p5;
        p1.x = back;  p1.y = left;  p1.z = down;
        p2.x = front; p2.y = left;  p2.z = down;
        p3.x = back;  p3.y = right; p3.z = down;
        p5.x = back;  p5.y = left;  p5.z = up;

        long long id1 = get_voxel_id(p1, v[0]);
        long long id2 = get_voxel_id(p2, v[0]);
        long long id3 = get_voxel_id(p3, v[0]);
        long long id5 = get_voxel_id(p5, v[0]);

        long long xnum = (id2 - id1) + 1;
        long long ynum = (id3 - id1) / v[0].voxel_n + 1;
        long long znum = (id5 - id1) / (v[0].voxel_n * v[0].voxel_n) + 1;

        for (long long i = 0; i < znum; i++) {
            for (long long j = 0; j < ynum; j++) {
                for (long long k = 0; k < xnum; k++) {
                    long long iid = id1 + k + v[0].voxel_n * j
                                  + v[0].voxel_n * v[0].voxel_n * i;
                    voxel_is_in(&v[iid]);
                }
            }
        }
    }

    virtual void cal_occupy(voxel* v, double box_l) {
        if (is_onboundary(box_l)) {
            for (int i = -1; i <= 1; i++) {
                for (int j = -1; j <= 1; j++) {
                    for (int k = -1; k <= 1; k++) {
                        translate(i * box_l, j * box_l, k * box_l);
                        cal_occupy_single(&v[0], box_l);
                        translate(-1.0 * i * box_l, -1.0 * j * box_l, -1.0 * k * box_l);
                    }
                }
            }
        }
        else cal_occupy_single(&v[0], box_l);
    }

    virtual unique_ptr<Particle> enlarge(double add) const = 0;

    virtual ~Particle() {}
};

// ─────────────────────────────────────────────────────────────────
//  Sphere
// ─────────────────────────────────────────────────────────────────
class Sphere : public Particle {
public:
    double R;
    Sphere();
    Sphere(double R, Point3d ccenter);
    ~Sphere();
    void translate(double x, double y, double z) override;
    bool intersects(Particle* p) const override;
    std::unique_ptr<Particle> clone() const override {
        return std::make_unique<Sphere>(*this);
    }
    bool is_onboundary(double box_l) const override;
    bool point_is_in(Point3d v) const override;
    unique_ptr<Particle> enlarge(double add) const override;
};

// ─────────────────────────────────────────────────────────────────
//  Box — 任意姿态长方体
//
//  A, B, C     — 长方体三条边长
//  ux, uy, uz  — 三条局部轴在全局坐标下的单位方向
//  half_*      — 三个半边长
//  outer_r     — 外接球半径 = 0.5 * sqrt(A²+B²+C²)
//  inner_r     — 内切球半径 = 0.5 * min(A,B,C)
//  character_L — 2 * outer_r，用于 cell/voxel 尺度划分
//
//  求交：
//    Box-Box 使用 15 个分离轴的 OBB-SAT 精确检测。
//    Sphere-Box 使用球心到 OBB 的最近点距离检测。
// ─────────────────────────────────────────────────────────────────
class Box : public Particle {
public:
    double A, B, C;
    double half_A, half_B, half_C;
    Point3d ux, uy, uz;

    Box();
    Box(double a, double b, double c, Point3d ccenter,
        Orientation3d orient = Orientation3d());
    ~Box();

    void translate(double x, double y, double z) override;
    bool intersects(Particle* p) const override;
    std::unique_ptr<Particle> clone() const override {
        return std::make_unique<Box>(*this);
    }
    bool is_onboundary(double box_l) const override;
    bool point_is_in(Point3d v) const override;
    void cal_occupy_single(voxel* v, double box_l) const override;
    unique_ptr<Particle> enlarge(double add) const override;

    void get_aabb_half_extents(double& hx, double& hy, double& hz) const;
    Point3d closest_point(Point3d p) const;
    bool sphere_intersects(const Sphere* s) const;
    bool box_intersects(const Box* b) const;
    void get_corners(Point3d out[8]) const;
};

// ─────────────────────────────────────────────────────────────────
//  辅助函数
// ─────────────────────────────────────────────────────────────────

// 在 [0, L)^3 中生成随机位置
Point3d get_randloc(double L);

// 均匀随机生成 SO(3) 姿态，用于长方体随机取向
Orientation3d get_randorientation();

// 为兼容旧代码保留：在单位球面上均匀随机生成方向向量
Point3d get_randdir();

// 输出 AutoCAD 脚本
void scr(double box_l, vector<unique_ptr<Particle>>& particles, int num, int cycle);

#endif // _PARTICLE_H_
