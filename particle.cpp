#include <cmath>
#include <memory>
#include <algorithm>
#include <fstream>
#include <iomanip>

#include "particle.h"

using namespace std;

namespace {

const double EPS = 1e-12;

Point3d cross_product(const Point3d& a, const Point3d& b) {
    return Point3d(a.y * b.z - a.z * b.y,
                   a.z * b.x - a.x * b.z,
                   a.x * b.y - a.y * b.x);
}

Point3d normalize_safe(Point3d v, Point3d fallback) {
    double len = PointMag3d(v);
    if (len <= EPS) return fallback;
    return v / len;
}

Orientation3d orthonormalize(Orientation3d o) {
    Point3d ux = normalize_safe(o.ux, Point3d(1.0, 0.0, 0.0));

    // 去除 uy 在 ux 上的分量，避免输入姿态因数值误差不正交。
    Point3d uy_raw = o.uy - ux * (o.uy * ux);
    Point3d uy = normalize_safe(uy_raw, Point3d(0.0, 1.0, 0.0));
    if (fabs(uy * ux) > 1e-8) {
        // fallback 可能与 ux 近平行时，换一个参考方向重新构造。
        Point3d ref = fabs(ux.x) < 0.9 ? Point3d(1.0, 0.0, 0.0) : Point3d(0.0, 1.0, 0.0);
        uy = normalize_safe(cross_product(cross_product(ux, ref), ux), Point3d(0.0, 1.0, 0.0));
    }

    Point3d uz = normalize_safe(cross_product(ux, uy), Point3d(0.0, 0.0, 1.0));
    uy = normalize_safe(cross_product(uz, ux), uy);
    return Orientation3d(ux, uy, uz);
}

inline double abs_dot(const Point3d& a, const Point3d& b) {
    return fabs(a * b);
}

void write_point(ofstream& out, const Point3d& p) {
    out << p.x << "," << p.y << "," << p.z;
}

void write_3dface(ofstream& out,
                  const Point3d& p1,
                  const Point3d& p2,
                  const Point3d& p3,
                  const Point3d& p4) {
    out << "3dface ";
    write_point(out, p1); out << " ";
    write_point(out, p2); out << " ";
    write_point(out, p3); out << " ";
    write_point(out, p4); out << endl << endl;
}

} // namespace

// ═══════════════════════════════════════════════════════════════
//  Sphere 实现
// ═══════════════════════════════════════════════════════════════

Sphere::Sphere() {}

Sphere::Sphere(double cR, Point3d ccenter) {
    center      = ccenter;
    outer_r     = cR;
    inner_r     = cR;
    character_L = 2.2 * cR;
    R           = cR;
}

Sphere::~Sphere() {}

void Sphere::translate(double x, double y, double z) {
    center.x += x;
    center.y += y;
    center.z += z;
}

bool Sphere::intersects(Particle* p) const {
    // 快速排斥：外接球距离检测。
    if (PointMag3d(center - p->center) > (outer_r + p->outer_r)) return false;

    const Sphere* s = dynamic_cast<const Sphere*>(p);
    if (s) {
        double dist = PointMag3d(center - s->center);
        return dist < (R + s->R);
    }

    const Box* b = dynamic_cast<const Box*>(p);
    if (b) {
        return b->sphere_intersects(this);
    }

    // 未知类型：保守判定为相交。
    return true;
}

bool Sphere::is_onboundary(double box_l) const {
    double half = character_L * 0.5;
    return (center.z + half >= box_l || center.z - half <= 0.0 ||
            center.y - half <= 0.0 || center.y + half >= box_l ||
            center.x + half >= box_l || center.x - half <= 0.0);
}

bool Sphere::point_is_in(Point3d v) const {
    return PointMag3d(v - center) < R;
}

unique_ptr<Particle> Sphere::enlarge(double add) const {
    return make_unique<Sphere>(R + add, center);
}

// ═══════════════════════════════════════════════════════════════
//  Box 实现
// ═══════════════════════════════════════════════════════════════

Box::Box() {}

Box::Box(double a, double b, double c, Point3d ccenter, Orientation3d orient) {
    center = ccenter;
    A = a;
    B = b;
    C = c;
    half_A = A * 0.5;
    half_B = B * 0.5;
    half_C = C * 0.5;

    Orientation3d o = orthonormalize(orient);
    ux = o.ux;
    uy = o.uy;
    uz = o.uz;

    outer_r     = 0.5 * sqrt(A * A + B * B + C * C);
    inner_r     = 0.5 * min(A, min(B, C));
    character_L = 2.0 * outer_r;
}

Box::~Box() {}

void Box::translate(double x, double y, double z) {
    center.x += x;
    center.y += y;
    center.z += z;
}

bool Box::intersects(Particle* p) const {
    // 快速排斥：外接球距离检测。
    if (PointMag3d(center - p->center) > (outer_r + p->outer_r)) return false;

    const Box* b = dynamic_cast<const Box*>(p);
    if (b) return box_intersects(b);

    const Sphere* s = dynamic_cast<const Sphere*>(p);
    if (s) return sphere_intersects(s);

    // 未知类型：保守判定为相交。
    return true;
}

void Box::get_aabb_half_extents(double& hx, double& hy, double& hz) const {
    hx = fabs(ux.x) * half_A + fabs(uy.x) * half_B + fabs(uz.x) * half_C;
    hy = fabs(ux.y) * half_A + fabs(uy.y) * half_B + fabs(uz.y) * half_C;
    hz = fabs(ux.z) * half_A + fabs(uy.z) * half_B + fabs(uz.z) * half_C;
}

bool Box::is_onboundary(double box_l) const {
    double hx, hy, hz;
    get_aabb_half_extents(hx, hy, hz);
    return (center.x - hx <= 0.0 || center.x + hx >= box_l ||
            center.y - hy <= 0.0 || center.y + hy >= box_l ||
            center.z - hz <= 0.0 || center.z + hz >= box_l);
}

bool Box::point_is_in(Point3d v) const {
    Point3d d = v - center;
    return (fabs(d * ux) < half_A &&
            fabs(d * uy) < half_B &&
            fabs(d * uz) < half_C);
}

Point3d Box::closest_point(Point3d p) const {
    Point3d d = p - center;
    Point3d q = center;

    double dist = d * ux;
    if (dist > half_A) dist = half_A;
    else if (dist < -half_A) dist = -half_A;
    q = q + ux * dist;

    dist = d * uy;
    if (dist > half_B) dist = half_B;
    else if (dist < -half_B) dist = -half_B;
    q = q + uy * dist;

    dist = d * uz;
    if (dist > half_C) dist = half_C;
    else if (dist < -half_C) dist = -half_C;
    q = q + uz * dist;

    return q;
}

bool Box::sphere_intersects(const Sphere* s) const {
    Point3d q = closest_point(s->center);
    Point3d diff = s->center - q;
    return (diff * diff) < (s->R * s->R);
}

bool Box::box_intersects(const Box* b) const {
    // OBB-OBB SAT，参考 Gottschalk / Ericson 的 15 分离轴写法。
    const Point3d Aaxis[3] = { ux, uy, uz };
    const Point3d Baxis[3] = { b->ux, b->uy, b->uz };
    const double Ahalf[3] = { half_A, half_B, half_C };
    const double Bhalf[3] = { b->half_A, b->half_B, b->half_C };

    double Rm[3][3];
    double AbsR[3][3];

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            Rm[i][j] = Aaxis[i] * Baxis[j];
            AbsR[i][j] = fabs(Rm[i][j]) + 1e-12;
        }
    }

    Point3d d = b->center - center;
    double t[3] = { d * Aaxis[0], d * Aaxis[1], d * Aaxis[2] };

    double ra, rb, tv;

    // 3 个 A 盒局部轴。
    for (int i = 0; i < 3; ++i) {
        ra = Ahalf[i];
        rb = Bhalf[0] * AbsR[i][0] + Bhalf[1] * AbsR[i][1] + Bhalf[2] * AbsR[i][2];
        if (fabs(t[i]) > ra + rb) return false;
    }

    // 3 个 B 盒局部轴。
    for (int j = 0; j < 3; ++j) {
        ra = Ahalf[0] * AbsR[0][j] + Ahalf[1] * AbsR[1][j] + Ahalf[2] * AbsR[2][j];
        rb = Bhalf[j];
        tv = fabs(t[0] * Rm[0][j] + t[1] * Rm[1][j] + t[2] * Rm[2][j]);
        if (tv > ra + rb) return false;
    }

    // 9 个叉乘轴 A_i x B_j。
    ra = Ahalf[1] * AbsR[2][0] + Ahalf[2] * AbsR[1][0];
    rb = Bhalf[1] * AbsR[0][2] + Bhalf[2] * AbsR[0][1];
    tv = fabs(t[2] * Rm[1][0] - t[1] * Rm[2][0]);
    if (tv > ra + rb) return false;

    ra = Ahalf[1] * AbsR[2][1] + Ahalf[2] * AbsR[1][1];
    rb = Bhalf[0] * AbsR[0][2] + Bhalf[2] * AbsR[0][0];
    tv = fabs(t[2] * Rm[1][1] - t[1] * Rm[2][1]);
    if (tv > ra + rb) return false;

    ra = Ahalf[1] * AbsR[2][2] + Ahalf[2] * AbsR[1][2];
    rb = Bhalf[0] * AbsR[0][1] + Bhalf[1] * AbsR[0][0];
    tv = fabs(t[2] * Rm[1][2] - t[1] * Rm[2][2]);
    if (tv > ra + rb) return false;

    ra = Ahalf[0] * AbsR[2][0] + Ahalf[2] * AbsR[0][0];
    rb = Bhalf[1] * AbsR[1][2] + Bhalf[2] * AbsR[1][1];
    tv = fabs(t[0] * Rm[2][0] - t[2] * Rm[0][0]);
    if (tv > ra + rb) return false;

    ra = Ahalf[0] * AbsR[2][1] + Ahalf[2] * AbsR[0][1];
    rb = Bhalf[0] * AbsR[1][2] + Bhalf[2] * AbsR[1][0];
    tv = fabs(t[0] * Rm[2][1] - t[2] * Rm[0][1]);
    if (tv > ra + rb) return false;

    ra = Ahalf[0] * AbsR[2][2] + Ahalf[2] * AbsR[0][2];
    rb = Bhalf[0] * AbsR[1][1] + Bhalf[1] * AbsR[1][0];
    tv = fabs(t[0] * Rm[2][2] - t[2] * Rm[0][2]);
    if (tv > ra + rb) return false;

    ra = Ahalf[0] * AbsR[1][0] + Ahalf[1] * AbsR[0][0];
    rb = Bhalf[1] * AbsR[2][2] + Bhalf[2] * AbsR[2][1];
    tv = fabs(t[1] * Rm[0][0] - t[0] * Rm[1][0]);
    if (tv > ra + rb) return false;

    ra = Ahalf[0] * AbsR[1][1] + Ahalf[1] * AbsR[0][1];
    rb = Bhalf[0] * AbsR[2][2] + Bhalf[2] * AbsR[2][0];
    tv = fabs(t[1] * Rm[0][1] - t[0] * Rm[1][1]);
    if (tv > ra + rb) return false;

    ra = Ahalf[0] * AbsR[1][2] + Ahalf[1] * AbsR[0][2];
    rb = Bhalf[0] * AbsR[2][1] + Bhalf[1] * AbsR[2][0];
    tv = fabs(t[1] * Rm[0][2] - t[0] * Rm[1][2]);
    if (tv > ra + rb) return false;

    return true;
}

void Box::cal_occupy_single(voxel* v, double box_l) const {
    double hx, hy, hz;
    get_aabb_half_extents(hx, hy, hz);

    double x_lo = center.x - hx;
    double x_hi = center.x + hx;
    double y_lo = center.y - hy;
    double y_hi = center.y + hy;
    double z_lo = center.z - hz;
    double z_hi = center.z + hz;

    Point3d p1, p2, p3, p5;
    p1.x = x_lo; p1.y = y_lo; p1.z = z_lo;
    p2.x = x_hi; p2.y = y_lo; p2.z = z_lo;
    p3.x = x_lo; p3.y = y_hi; p3.z = z_lo;
    p5.x = x_lo; p5.y = y_lo; p5.z = z_hi;

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
                long long iid = id1 + k
                              + v[0].voxel_n * j
                              + v[0].voxel_n * v[0].voxel_n * i;
                voxel_is_in(&v[iid]);
            }
        }
    }
}

unique_ptr<Particle> Box::enlarge(double add) const {
    // 用局部三方向半边长同步外扩 add。该长方体包含真实 Minkowski 外扩体，
    // 因而用于 voxel 剪枝时偏保守但安全。
    return make_unique<Box>(A + 2.0 * add, B + 2.0 * add, C + 2.0 * add,
                            center, Orientation3d(ux, uy, uz));
}

void Box::get_corners(Point3d out[8]) const {
    Point3d ax = ux * half_A;
    Point3d ay = uy * half_B;
    Point3d az = uz * half_C;

    out[0] = center - ax - ay - az;
    out[1] = center + ax - ay - az;
    out[2] = center - ax + ay - az;
    out[3] = center + ax + ay - az;
    out[4] = center - ax - ay + az;
    out[5] = center + ax - ay + az;
    out[6] = center - ax + ay + az;
    out[7] = center + ax + ay + az;
}

// ═══════════════════════════════════════════════════════════════
//  其他辅助函数
// ═══════════════════════════════════════════════════════════════

Point3d get_randloc(double L) {
    Point3d result;
    result.x = L * double(rand()) / double(RAND_MAX);
    result.y = L * double(rand()) / double(RAND_MAX);
    result.z = L * double(rand()) / double(RAND_MAX);
    return result;
}

Point3d get_randdir() {
    double z   = 2.0 * double(rand()) / double(RAND_MAX) - 1.0;
    double phi = 2.0 * PI * double(rand()) / double(RAND_MAX);
    double r   = sqrt(max(0.0, 1.0 - z * z));
    return Point3d(r * cos(phi), r * sin(phi), z);
}

Orientation3d get_randorientation() {
    // James Arvo / Shoemake 方法：由均匀随机四元数生成 SO(3) 均匀姿态。
    double u1 = double(rand()) / double(RAND_MAX);
    double u2 = double(rand()) / double(RAND_MAX);
    double u3 = double(rand()) / double(RAND_MAX);

    double qx = sqrt(1.0 - u1) * sin(2.0 * PI * u2);
    double qy = sqrt(1.0 - u1) * cos(2.0 * PI * u2);
    double qz = sqrt(u1)       * sin(2.0 * PI * u3);
    double qw = sqrt(u1)       * cos(2.0 * PI * u3);

    double xx = qx * qx, yy = qy * qy, zz = qz * qz;
    double xy = qx * qy, xz = qx * qz, yz = qy * qz;
    double wx = qw * qx, wy = qw * qy, wz = qw * qz;

    // 旋转矩阵列向量作为局部三轴。
    Point3d ux(1.0 - 2.0 * (yy + zz),
               2.0 * (xy + wz),
               2.0 * (xz - wy));

    Point3d uy(2.0 * (xy - wz),
               1.0 - 2.0 * (xx + zz),
               2.0 * (yz + wx));

    Point3d uz(2.0 * (xz + wy),
               2.0 * (yz - wx),
               1.0 - 2.0 * (xx + yy));

    return orthonormalize(Orientation3d(ux, uy, uz));
}

// ───────────────────────────────────────────────────────────────
//  输出 AutoCAD 脚本：绘制所有任意姿态长方体、球以及模拟箱框架。
//
//  长方体使用 6 个 3DFACE 面片绘制；球使用 SPHERE 命令。
// ───────────────────────────────────────────────────────────────
void scr(double box_l, vector<unique_ptr<Particle>>& particles, int num, int cycle) {
    char cha[40];
    sprintf(cha, "packing_%d.scr", cycle);
    ofstream out(cha);
    out << fixed << setprecision(8);

    out << "-osnap off"  << endl;
    out << "erase all "  << endl;
    out << "vscurrent r" << endl;

    for (int i = 0; i < num; ++i) {
        const Box* box = dynamic_cast<const Box*>(particles[i].get());
        if (box) {
            out << "color 1" << endl;
            Point3d c[8];
            box->get_corners(c);
            write_3dface(out, c[0], c[1], c[3], c[2]); // z-
            write_3dface(out, c[4], c[5], c[7], c[6]); // z+
            write_3dface(out, c[0], c[1], c[5], c[4]); // y-
            write_3dface(out, c[2], c[3], c[7], c[6]); // y+
            write_3dface(out, c[0], c[2], c[6], c[4]); // x-
            write_3dface(out, c[1], c[3], c[7], c[5]); // x+
            continue;
        }

        const Sphere* sp = dynamic_cast<const Sphere*>(particles[i].get());
        if (sp) {
            out << "color 3" << endl;
            out << "sphere ";
            write_point(out, sp->center);
            out << " " << sp->R << endl;
        }
    }

    // 绘制模拟箱的 12 条棱（细圆柱模拟线框）。
    out << "-color 7" << endl;
    out << "zoom e"        << endl;
    out << "vpoint 1,1,1"  << endl;
    out << "cylinder 0,0,0 0.05 a " << box_l << ",0,0"              << endl;
    out << "cylinder 0,0,0 0.05 a 0," << box_l << ",0"              << endl;
    out << "cylinder 0,0,0 0.05 a 0,0," << box_l                    << endl;
    out << "cylinder " << box_l << "," << box_l << "," << box_l
        << " 0.05 a " << box_l << ",0," << box_l                    << endl;
    out << "cylinder " << box_l << "," << box_l << "," << box_l
        << " 0.05 a 0," << box_l << "," << box_l                    << endl;
    out << "cylinder " << box_l << "," << box_l << "," << box_l
        << " 0.05 a " << box_l << "," << box_l << ",0"              << endl;
    out << "cylinder " << box_l << ",0," << box_l
        << " 0.05 a " << box_l << ",0,0"                            << endl;
    out << "cylinder " << box_l << ",0," << box_l
        << " 0.05 a 0,0," << box_l                                  << endl;
    out << "cylinder " << box_l << "," << box_l << ",0"
        << " 0.05 a " << box_l << ",0,0"                            << endl;
    out << "cylinder " << box_l << "," << box_l << ",0"
        << " 0.05 a 0," << box_l << ",0"                            << endl;
    out << "cylinder 0," << box_l << "," << box_l
        << " 0.05 a 0," << box_l << ",0"                            << endl;
    out << "cylinder 0," << box_l << "," << box_l
        << " 0.05 a 0,0," << box_l                                  << endl;

    out.close();
}
