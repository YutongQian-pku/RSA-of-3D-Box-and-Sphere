#ifndef _CONFIG_H_
#define _CONFIG_H_

// 1. 模拟箱总体积 (box_l^3)
const double VOLUME = 5000.0;

// 2. 无量纲填充时间系数（rsa_step = 系数 * VOLUME）
const double RSA_STEP_COEFFICIENT = 1E3;

// 3. 最终细化阶段每个 voxel 的最小尝试次数
const double FINAL_ADDNUM_MIN = 200.0;

// 4. 独立模拟循环次数
const int CYCLE_NUM = 1;

#endif // _CONFIG_H_

