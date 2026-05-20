#define _CRT_SECURE_NO_WARNINGS
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <iostream>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <cstdlib>
#include <vector>
#include <ctime>
#include <string>
#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <algorithm>
#include <cctype>

#include "config.h"     // VOLUME, RSA_STEP_COEFFICIENT, CYCLE_NUM, FINAL_ADDNUM_MIN
#include "simulation.h"

using namespace std;
namespace fs = std::filesystem;

// 命令行用法：
//   ./rsa <i>
//
//   i —— 配置文件编号。程序读取“程序所在文件夹/conf/i.txt”。
//
// 配置文件格式（每行一种颗粒；空行和 # 注释会被忽略）：
//   BOX A B C p    长方体：三条边长 A、B、C，投入概率 p
//   S   R p        球：半径 R，投入概率 p
//
// 示例：
//   BOX 1 2 3 0.5
//   S   0.8 0.5
//
// 全局参数（见 config.h）：
//   VOLUME                — 模拟箱体积 box_l^3
//   RSA_STEP_COEFFICIENT  — rsa_step = RSA_STEP_COEFFICIENT * VOLUME
//   CYCLE_NUM             — 独立重复模拟次数
//   FINAL_ADDNUM_MIN      — 最终细化阶段每 voxel 最小尝试次数

fs::path executable_dir(const char* argv0) {
    // Linux 下优先使用 /proc/self/exe，保证即使从其他工作目录启动也能找到程序所在目录。
    try {
        fs::path proc_path = fs::read_symlink("/proc/self/exe");
        if (!proc_path.empty()) return proc_path.parent_path();
    }
    catch (...) {
        // 非 Linux 或 /proc 不可用时回退到 argv[0]
    }

    fs::path p = fs::absolute(argv0);
    if (p.has_parent_path()) return p.parent_path();
    return fs::current_path();
}

string trim_comment(const string& line) {
    size_t pos = line.find('#');
    if (pos == string::npos) return line;
    return line.substr(0, pos);
}

vector<ParticleSpec> read_particle_config(const fs::path& config_path) {
    ifstream in(config_path);
    if (!in.is_open()) {
        throw runtime_error("Cannot open config file: " + config_path.string());
    }

    vector<ParticleSpec> specs;
    string line;
    int line_no = 0;
    while (getline(in, line)) {
        line_no++;
        line = trim_comment(line);
        istringstream iss(line);

        string type;
        if (!(iss >> type)) continue; // 空行或纯注释

        transform(type.begin(), type.end(), type.begin(), ::toupper);

        if (type == "BOX" || type == "B") {
            double A, B, C, p;
            if (!(iss >> A >> B >> C >> p)) {
                throw runtime_error("Invalid BOX line at " + to_string(line_no) +
                                    ": expected 'BOX A B C probability'");
            }
            specs.push_back({ ParticleKind::Box, A, B, C, 0.0, p });
        }
        else if (type == "S") {
            double R, p;
            if (!(iss >> R >> p)) {
                throw runtime_error("Invalid S line at " + to_string(line_no) +
                                    ": expected 'S R probability'");
            }
            specs.push_back({ ParticleKind::Sphere, 0.0, 0.0, 0.0, R, p });
        }
        else {
            throw runtime_error("Unknown particle type at line " + to_string(line_no) +
                                ": " + type + " (expected BOX or S)");
        }
    }

    if (specs.empty()) {
        throw runtime_error("Config file contains no particle specs: " + config_path.string());
    }

    double sum_p = 0.0;
    for (const auto& spec : specs) sum_p += spec.prob;
    if (fabs(sum_p - 1.0) > 1e-6) {
        throw runtime_error("Probabilities in config must sum to 1. Current sum = " +
                            to_string(sum_p));
    }

    return specs;
}

string compact_double(double v) {
    ostringstream oss;
    oss << setprecision(8) << v;
    string s = oss.str();
    for (char& c : s) {
        if (c == '.') c = 'p';
        else if (c == '-') c = 'm';
        else if (c == '+') c = 'p';
    }
    return s;
}

string make_output_folder_name(const string& config_id,
                               const vector<ParticleSpec>& specs) {
    string folder_name = "run_" + config_id;
    for (const auto& spec : specs) {
        folder_name += "_";
        folder_name += particle_kind_name(spec.kind);
        if (spec.kind == ParticleKind::Box) {
            folder_name += "_A" + compact_double(spec.A);
            folder_name += "_B" + compact_double(spec.B);
            folder_name += "_C" + compact_double(spec.C);
        }
        else {
            folder_name += "_R" + compact_double(spec.R);
        }
        folder_name += "_P" + compact_double(spec.prob);
    }
    return folder_name;
}

void append_summary_result(const fs::path& result_path,
                           const string& config_id,
                           const vector<ParticleSpec>& specs,
                           const vector<double>& result) {
    ofstream outfile(result_path, ios_base::app);
    if (!outfile.is_open()) {
        cout << "Unable to open " << result_path << " for writing." << endl;
        return;
    }

    outfile << config_id << "\t";
    for (const auto& spec : specs) {
        outfile << particle_kind_name(spec.kind) << "\t";
        if (spec.kind == ParticleKind::Box) {
            outfile << spec.A << "\t"
                    << spec.B << "\t"
                    << spec.C << "\t";
        }
        else {
            outfile << spec.R << "\t";
        }
        outfile << spec.prob << "\t";
    }
    for (double v : result) outfile << v << "\t";
    outfile << endl;

    cout << "Results appended to " << result_path << "." << endl;
}

int main(int argc, char* argv[]) {
    srand(static_cast<unsigned int>(time(NULL)));

    if (argc != 2) {
        cerr << "Usage: " << argv[0] << " <config_id>" << endl;
        cerr << "Example: " << argv[0] << " 2   # reads conf/2.txt" << endl;
        return 1;
    }

    string config_id = argv[1];
    if (config_id.find('/') != string::npos ||
        config_id.find('\\') != string::npos) {
        cerr << "Error: config_id must be a file number/name without path separators." << endl;
        return 1;
    }

    try {
        fs::path base_dir    = executable_dir(argv[0]);
        fs::path config_path = base_dir / "conf" / (config_id + ".txt");

        vector<ParticleSpec> specs = read_particle_config(config_path);

        // 模拟箱体积和最大尝试步数来自 config.h。
        double    volume   = VOLUME;
        long long rsa_step = static_cast<long long>(RSA_STEP_COEFFICIENT * volume);
        int       cycle    = CYCLE_NUM;

        string folder_name = make_output_folder_name(config_id, specs);
        fs::path output_dir = base_dir / folder_name;
        fs::create_directories(output_dir);

        fs::path old_path = fs::current_path();
        fs::current_path(output_dir);

        // 执行 RSA 模拟。
        vector<double> result = rsa_cycle(specs, volume, rsa_step, cycle);

        fs::current_path(old_path);

        // 将结果追加写入程序所在目录下的 results.txt。
        append_summary_result(base_dir / "results.txt", config_id, specs, result);
    }
    catch (const exception& e) {
        cerr << "Error: " << e.what() << endl;
        return 1;
    }

    return 0;
}
