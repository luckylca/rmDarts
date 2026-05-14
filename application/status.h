#include "stdint.h"

// ==========================================
// 1. 状态位定义 (Bitmask Definition)
// ==========================================

// --- A. 扳机与装填机构状态 (0-3位) ---
#define FLAG_TRIGGER_AT_LOAD_POS (1 << 0)  // 扳机处于“接镖/装弹”位置
#define FLAG_TRIGGER_AT_SHOOT_POS (1 << 1) // 扳机处于“蓄能/发射”位置
#define FLAG_RELOAD_ROTATED (1 << 2)       // 旋转换弹电机到位 (30°/120°)
#define FLAG_ARM_ANGLE_READY (1 << 3)      // 机械臂角度到位

// --- B. 摩擦轮与发射准备状态 (4-9位) ---
#define FLAG_L_BOTTOM_REACHED (1 << 4) // 左换弹电机到位
#define FLAG_R_BOTTOM_REACHED (1 << 5) // 右换弹电机到位
#define FLAG_L_CHARGE_REACHED (1 << 6)  // 左蓄力电机到位
#define FLAG_R_CHARGE_REACHED (1 << 7)  // 右蓄力电机到位
#define FLAG_L_REBOUND_REACHED (1 << 8) // 左反弹电机到位
#define FLAG_R_REBOUND_REACHED (1 << 9) // 右反弹电机到位
#define FLAG_GIMBAL_AIMED (1 << 10)      // 云台瞄准完成

// --- C. 动作完成标志 (10-15位) ---
#define FLAG_DART_DROPPED (1 << 11) // 电磁铁已断电，飞镖已掉入扳机
#define FLAG_FIRED (1 << 12)        // 扳机已释放，飞镖已射出
#define FLAG_IN_PLACE (1 << 13)    // 视觉通信到位

// ==========================================
// 2. 组合掩码 (逻辑判断核心)
// ==========================================

// 旋转换弹并且扳机位置到位了，就可以装弹了
#define MASK_READY_TO_LOAD (FLAG_L_BOTTOM_REACHED | FLAG_R_BOTTOM_REACHED)//换弹开始标志位

#define MASK_READY_TO_SHOOT (FLAG_RELOAD_ROTATED |  \
                            FLAG_DART_DROPPED |  \
                            FLAG_ARM_ANGLE_READY |  \
                            FLAG_TRIGGER_AT_LOAD_POS)//没有用到

#define MASK_READY_TO_CHASSIS (FLAG_DART_DROPPED)//检查飞镖是否掉落
                            
// [发射条件]：扳机必须顶出发射位 + 蓄力/反弹电机OK + 云台瞄准OK + 且飞镖已经装进去了+视觉到位
#define MASK_READY_TO_FIRE (FLAG_TRIGGER_AT_SHOOT_POS |                       \
                            FLAG_L_CHARGE_REACHED | FLAG_R_CHARGE_REACHED |   \
                            FLAG_L_REBOUND_REACHED | FLAG_R_REBOUND_REACHED | \
                            FLAG_GIMBAL_AIMED |                               \
                            FLAG_DART_DROPPED | \
                            FLAG_IN_PLACE)//准备发射标志位

#define FLAG_READY_TO_SHOOT_WITHOUT_VISION (FLAG_L_REBOUND_REACHED | FLAG_R_REBOUND_REACHED)                            
// ==========================================
// 3. 数据结构与宏
// ==========================================
typedef struct
{
    uint8_t currentStep;     // 当前是第几发 (0-3)
    uint32_t statusFlags[4]; // 4发飞镖的独立状态板
} DartSystem_t;

extern DartSystem_t DartSys;

// 语法糖宏

#define DART_SET_BIT(idx, flag)   (DartSys.statusFlags[idx] |= (flag))
#define DART_CLEAR_BIT(idx, flag) (DartSys.statusFlags[idx] &= ~(flag))             //清楚某设备的某标志
#define DART_CHECK_BIT(idx, flag) ((DartSys.statusFlags[idx] & (flag)) == (flag))
#define DART_CHECK_MASK(idx, mask) ((DartSys.statusFlags[idx] & (mask)) == (mask))