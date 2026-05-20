#ifndef _SIMULATION_H_
#define _SIMULATION_H_

using namespace std;

#include <fstream>
#include <vector>
#include <random>
#include <memory>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <unordered_set>
#include <algorithm>
#include <cmath>
#include <string>

#include "config.h"
#include "particle.h"
#include "cell.h"
#include "voxel.h"

// ═══════════════════════════════════════════════════════════════
//  混合 RSA 颗粒定义
//
//  配置文件格式（conf/i.txt，每行一种颗粒）：
//    BOX A B C prob —— 长方体：三条边长 A、B、C，投入概率 prob
//    S   R prob     —— 球：半径 R，投入概率 prob
// ═══════════════════════════════════════════════════════════════
enum class ParticleKind {
    Box,
    Sphere
};

struct ParticleSpec {
    ParticleKind kind;
    double A;       // 长方体边长 A；球为 0
    double B;       // 长方体边长 B；球为 0
    double C;       // 长方体边长 C；球为 0
    double R;       // 球半径；长方体为 0
    double prob;    // 投入概率
};

inline const char* particle_kind_name(ParticleKind kind) {
    return kind == ParticleKind::Box ? "BOX" : "S";
}

inline double box_volume(double A, double B, double C) {
    return A * B * C;
}

inline double sphere_volume(double R) {
    return (4.0 / 3.0) * PI * R * R * R;
}

inline double particle_volume(const ParticleSpec& spec) {
    if (spec.kind == ParticleKind::Box) {
        return box_volume(spec.A, spec.B, spec.C);
    }
    return sphere_volume(spec.R);
}

inline unique_ptr<Particle> make_particle(const ParticleSpec& spec,
                                          Point3d center,
                                          Orientation3d orient = Orientation3d()) {
    if (spec.kind == ParticleKind::Box) {
        return make_unique<Box>(spec.A, spec.B, spec.C, center, orient);
    }
    return make_unique<Sphere>(spec.R, center);
}

inline unique_ptr<Particle> make_random_particle(const ParticleSpec& spec,
                                                 Point3d center) {
    if (spec.kind == ParticleKind::Box) {
        return make_unique<Box>(spec.A, spec.B, spec.C, center, get_randorientation());
    }
    return make_unique<Sphere>(spec.R, center);
}

inline int choose_index_from_cumulative(const vector<double>& cumulative_probs,
                                        double rand_val) {
    int index = 0;
    while (index < (int)cumulative_probs.size() && rand_val > cumulative_probs[index]) {
        index++;
    }
    if (index >= (int)cumulative_probs.size()) index = (int)cumulative_probs.size() - 1;
    return index;
}

inline void print_particle_inserted(const Particle* p,
                                    long long try_num,
                                    double dimensionless_t,
                                    int particle_num) {
    cout << "try_num=" << try_num
         << "\tt=" << dimensionless_t
         << "\tnum=" << particle_num;

    const Box* box = dynamic_cast<const Box*>(p);
    if (box) {
        cout << "\ttype=BOX"
             << "\tA=" << box->A
             << "\tB=" << box->B
             << "\tC=" << box->C;
    }
    else {
        const Sphere* s = dynamic_cast<const Sphere*>(p);
        if (s) {
            cout << "\ttype=S"
                 << "\tR=" << s->R;
        }
    }
    cout << endl;
}

// ═══════════════════════════════════════════════════════════════
//  rsa_cycle：对长方体-球混合体系执行 RSA（随机顺序吸附）模拟。
//
//  每步 RSA 尝试时位置随机；长方体姿态在 SO(3) 上随机且各向同性；球无方向。
//  体积分数：φ = Σ V_i · N_i / V_box，其中 V_i 按颗粒类型计算。
// ═══════════════════════════════════════════════════════════════
vector<double> rsa_cycle(const vector<ParticleSpec>& specs,
                         double volume,
                         long long rsa_step,
                         int cycle_num)
{
    // ── 输入合法性验证 ──────────────────────────────────────────
    if (specs.empty()) {
        cerr << "Error: No particle specs were provided!" << endl;
        return vector<double>();
    }

    double sum_probs = 0.0;
    for (const auto& spec : specs) {
        if (spec.kind == ParticleKind::Box) {
            if (spec.A <= 0.0 || spec.B <= 0.0 || spec.C <= 0.0) {
                cerr << "Error: BOX side lengths A, B, C must all be positive!" << endl;
                return vector<double>();
            }
        }
        else {
            if (spec.R <= 0.0) {
                cerr << "Error: Sphere radius R must be positive!" << endl;
                return vector<double>();
            }
        }
        if (spec.prob < 0.0) {
            cerr << "Error: Probability cannot be negative!" << endl;
            return vector<double>();
        }
        sum_probs += spec.prob;
    }
    if (fabs(sum_probs - 1.0) > 1e-6) {
        cerr << "Error: Probabilities must sum to 1! Current sum = "
             << sum_probs << endl;
        return vector<double>();
    }

    // ── 输出参数信息 ────────────────────────────────────────────
    cout << "Particle specs in config order:" << endl;
    for (size_t i = 0; i < specs.size(); ++i) {
        cout << "  [" << i << "] type=" << particle_kind_name(specs[i].kind);
        if (specs[i].kind == ParticleKind::Box) {
            cout << " A=" << specs[i].A
                 << " B=" << specs[i].B
                 << " C=" << specs[i].C;
        }
        else {
            cout << " R=" << specs[i].R;
        }
        cout << " Prob=" << specs[i].prob
             << " Volume=" << particle_volume(specs[i])
             << endl;
    }
    cout << "Box orientation: RANDOM (uniform SO(3))" << endl;

    // ── 构建累积概率分布 ────────────────────────────────────────
    vector<double> cumulative_probs(specs.size());
    double cumulative = 0.0;
    for (size_t i = 0; i < specs.size(); ++i) {
        cumulative += specs[i].prob;
        cumulative_probs[i] = cumulative;
    }

    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<> dis(0.0, 1.0);

    vector<double> accmulate(specs.size(), 0.0);
    vector<int>    counts(specs.size(), 0);
    vector<double> phis;

    // ══════════════════════════════════════════════════════════
    //  主模拟循环（cycle_num 次独立重复）
    // ══════════════════════════════════════════════════════════
    for (int i = 0; i < cycle_num; i++) {

        double box_l = pow(volume, 1.0 / 3.0);

        vector<unique_ptr<Particle>> particles;
        Point3d origin = { 0.0, 0.0, 0.0 };
        Orientation3d identity_orient;

        // ── 计算各类颗粒的 character_L 极值，用于网格初始化 ──
        double max_char_L  = 0.0;
        double min_char_L  = 1e100;
        double min_inner_r = 1e100;
        for (const auto& spec : specs) {
            unique_ptr<Particle> pc = make_particle(spec, origin, identity_orient);
            max_char_L  = max(max_char_L,  pc->character_L);
            min_char_L  = min(min_char_L,  pc->character_L);
            min_inner_r = min(min_inner_r, pc->inner_r);
        }

        // ── 背景 Cell 网格初始化 ────────────────────────────────
        // cell 边长 ≈ max_char_L = 2 * outer_r_max，
        // 保证相交粒子必然落在相邻 cell 中。
        Cell* grid = nullptr;
        Cell cell(max_char_L, box_l);
        grid = new Cell[cell.cell_N];
        for (int ci = 0; ci < cell.cell_N; ci++) {
            grid[ci] = move(cell);
            cell = Cell(max_char_L, box_l);
        }
        initial_grid(&grid[0]);

        // ── Voxel 网格初始化 ────────────────────────────────────
        // voxel 边长 = min_char_L / 3，用于最终细化阶段剪枝。
        voxel* vox = nullptr;
        voxel vox1(min_char_L, box_l);
        vox = new voxel[vox1.voxel_N];
        for (long long vi = 0; vi < vox1.voxel_N; vi++) vox[vi] = vox1;
        initial_vox(&vox[0]);

        long long try_num         = 0;
        double    dimensionless_t = 0.0;
        int       particle_num    = 0;
        int       id;

        vector<int>         final_indices;
        vector<double>      final_probs_local;
        bool                should_check = false;
        const double        threshold    = volume * 10000.0;
        unordered_set<int>  unique_indices;

        // ══════════════════════════════════════════════════════
        //  RSA 主循环
        // ══════════════════════════════════════════════════════
        while (true) {
            try_num++;
            if (try_num > threshold && !should_check) {
                should_check = true;
            }

            // 按累积概率随机选取颗粒类型/尺寸。
            int spec_index = choose_index_from_cumulative(cumulative_probs, dis(gen));

            // 位置随机；长方体姿态随机，球无方向。
            unique_ptr<Particle> insert_p =
                make_random_particle(specs[spec_index], get_randloc(box_l));

            bool flag;
            if (check_over(grid, insert_p.get(), vox)) {
                flag = true;
            }
            else {
                flag = false;
                particles.push_back(move(insert_p));
                counts[spec_index]++;

                if (should_check && unique_indices.find(spec_index) == unique_indices.end()) {
                    unique_indices.insert(spec_index);
                    final_indices.push_back(spec_index);
                    final_probs_local.push_back(specs[spec_index].prob);
                }
            }

            if (!flag) {
                particle_num++;
                dimensionless_t = (double)try_num / volume;

                id = get_cell_id(grid[0],
                                 particles.back()->center.x,
                                 particles.back()->center.y,
                                 particles.back()->center.z);

                print_particle_inserted(particles.back().get(),
                                        try_num,
                                        dimensionless_t,
                                        particle_num);

                auto cloned = particles.back()->clone();
                grid[id].particles.push_back(move(cloned));
                grid[id].num++;

                // 以扩大后的颗粒标记 voxel（扩大量为 min_inner_r）。
                unique_ptr<Particle> p_big =
                    grid[id].particles.back()->enlarge(min_inner_r);
                if (p_big) {
                    p_big->cal_occupy(&vox[0], box_l);
                }
            }

            if (try_num % 1000000 == 0) {
                cout << "try_num=" << try_num
                     << "\tt=" << dimensionless_t
                     << "\tnum=" << particle_num << endl;
            }

            // ── 最终细化阶段（在每个未占据 voxel 内集中投放）──────
            if (try_num == rsa_step) {

                if (final_indices.empty()) {
                    cout << "final_indices is empty, using all particle specs." << endl;
                    for (size_t ni = 0; ni < specs.size(); ni++) {
                        final_indices.push_back((int)ni);
                        final_probs_local.push_back(specs[ni].prob);
                    }
                }

                cout << "Final refinement stage..." << endl;

                // 对 final_probs_local 归一化。
                double sum_fp = 0.0;
                for (double fp : final_probs_local) sum_fp += fp;

                vector<double> norm_probs;
                norm_probs.reserve(final_probs_local.size());
                cout << "Normalized Probabilities:" << endl;
                for (size_t ni = 0; ni < final_probs_local.size(); ++ni) {
                    int idx = final_indices[ni];
                    double np = final_probs_local[ni] / sum_fp;
                    norm_probs.push_back(np);
                    cout << "  type=" << particle_kind_name(specs[idx].kind);
                    if (specs[idx].kind == ParticleKind::Box) {
                        cout << " A=" << specs[idx].A
                             << " B=" << specs[idx].B
                             << " C=" << specs[idx].C;
                    }
                    else {
                        cout << " R=" << specs[idx].R;
                    }
                    cout << " Prob=" << np << endl;
                }

                // 构建最终阶段累积概率。
                vector<double> final_cum(norm_probs.size());
                double fc = 0.0;
                for (size_t ni = 0; ni < norm_probs.size(); ++ni) {
                    fc += norm_probs[ni];
                    final_cum[ni] = fc;
                }

                // 每个 voxel 的尝试次数。
                int final_addnum = (int)max(
                    2E4 * vox1.length * vox1.length * vox1.length,
                    FINAL_ADDNUM_MIN);
                cout << "Trials per voxel: " << final_addnum << endl;
                cout << "Voxel side length: " << vox1.length << endl;

                for (long long vi = 0; vi < vox1.voxel_N; vi++) {
                    if (vox[vi].is_occupy) continue;

                    // voxel 最小角点。
                    Point3d vmin;
                    vmin.x = vox[vi].center.x - vox[vi].length * 0.5;
                    vmin.y = vox[vi].center.y - vox[vi].length * 0.5;
                    vmin.z = vox[vi].center.z - vox[vi].length * 0.5;

                    for (int j = 0; j < final_addnum; j++) {
                        double rx = vmin.x + vox[vi].length * double(rand()) / double(RAND_MAX);
                        double ry = vmin.y + vox[vi].length * double(rand()) / double(RAND_MAX);
                        double rz = vmin.z + vox[vi].length * double(rand()) / double(RAND_MAX);

                        int local_index = choose_index_from_cumulative(final_cum, dis(gen));
                        int spec_index2 = final_indices[local_index];

                        unique_ptr<Particle> insert_p2 =
                            make_random_particle(specs[spec_index2], Point3d(rx, ry, rz));

                        bool flag2;
                        if (check_over(grid, insert_p2.get(), vox)) {
                            flag2 = true;
                        }
                        else {
                            flag2 = false;
                            particles.push_back(move(insert_p2));
                            counts[spec_index2]++;
                        }

                        if (!flag2) {
                            particle_num++;
                            dimensionless_t = (double)try_num / volume;

                            id = get_cell_id(grid[0],
                                             particles.back()->center.x,
                                             particles.back()->center.y,
                                             particles.back()->center.z);

                            auto cloned2 = particles.back()->clone();
                            grid[id].particles.push_back(move(cloned2));
                            grid[id].num++;

                            unique_ptr<Particle> p_big2 =
                                particles.back()->enlarge(min_inner_r);
                            if (p_big2) {
                                p_big2->cal_occupy(&vox[0], box_l);
                            }

                            cout << "Final stage added: total=" << particle_num << endl;
                        }
                    }
                    // 该 voxel 已完成细化，标记为占据。
                    vox[vi].is_occupy = true;
                }

                // 统计最终 voxel 占据情况。
                int vox_count = 0;
                for (long long vi = 0; vi < vox1.voxel_N; vi++) {
                    if (vox[vi].is_occupy) vox_count++;
                }
                cout << "all_vox_num         = " << vox1.voxel_N << endl;
                cout << "voxel_occupied_num  = " << vox_count     << endl;
                cout << "voxel_free_num      = " << vox1.voxel_N - vox_count << endl;

                cout << "Finished!" << endl;
                break;
            }
        } // end RSA while

        // ── 输出 AutoCAD 脚本（仅第 0 次循环）────────────────────
        if (i == 0) {
            scr(box_l, particles, particle_num, i);
        }

        // ── 统计 voxel 占据情况 ──────────────────────────────────
        int vox_total = 0;
        for (long long vi = 0; vi < vox1.voxel_N; vi++) {
            if (vox[vi].is_occupy) vox_total++;
        }
        cout << "all_vox_num        = " << vox1.voxel_N << endl;
        cout << "voxel_occupied_num = " << vox_total     << endl;

        // ── 释放内存 ─────────────────────────────────────────────
        delete[] grid;
        delete[] vox;

        // ── 计算并记录体积分数 ───────────────────────────────────
        ofstream out("result.txt", ios::app);
        out << fixed << setprecision(8);
        double phi_i = 0.0;
        for (size_t si = 0; si < specs.size(); si++) {
            double vol_one = particle_volume(specs[si]);
            phi_i += vol_one * counts[si] / volume;
            accmulate[si] += static_cast<double>(counts[si]);
            out << counts[si] << "\t";
        }
        out << phi_i << endl;
        out.close();
        phis.push_back(phi_i);

        counts.assign(specs.size(), 0);
    } // end cycle for

    // ── 计算均值和标准差 ─────────────────────────────────────────
    double phi_avg = 0.0;
    for (double phi : phis) phi_avg += phi / phis.size();

    double phi_var = 0.0;
    for (double phi : phis) phi_var += (phi - phi_avg) * (phi - phi_avg);
    if (phis.size() > 1) phi_var /= (phis.size() - 1);
    phi_var = sqrt(phi_var);

    for (size_t si = 0; si < specs.size(); si++) {
        accmulate[si] /= cycle_num;
    }
    accmulate.push_back(phi_avg);
    accmulate.push_back(phi_var);

    return accmulate;
}

#endif // _SIMULATION_H_
