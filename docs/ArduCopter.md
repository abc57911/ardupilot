# ArduCopter 開發指南

## 項目簡介

ArduPilot 是一個功能齊全的開源自動駕駛儀軟體，能夠控制多種飛行器系統，包括：

- **多旋翼機 (ArduCopter)**
- **固定翼飛機 (ArduPlane)** - 包括 QuadPlane VTOL
- **地面車輛/船隻 (Rover)**
- **水下潛水器 (Sub)**
- **輕於空氣的飛行器 (Blimp)**

---

## Copter 類別結構

### 類別概述

`Copter` 類別是 ArduCopter 應用程式的核心，它繼承自 `AP_Vehicle` 基類。

```cpp
class Copter : public AP_Vehicle {
public:
    // 子系統引用
    Mode *flightmode;           // 當前飛行模式
    AC_AttitudeControl *attitude_control;  // 姿態控制器
    AC_PosControl *pos_control;           // 位置控制器
    AP_MotorsMulticopter *motors;          // 馬達控制
    // ... 其他子系統
};
```

### 主要成員變數

| 成員 | 類型 | 功能 |
|------|------|------|
| `flightmode` | `Mode*` | 當前飛行模式指標 |
| `attitude_control` | `AC_AttitudeControl*` | 姿態控制器 |
| `pos_control` | `AC_PosControl*` | 位置控制器 |
| `motors` | `AP_MotorsMulticopter*` | 馬達控制 |

---

## 飛行模式

### 飛行模式列表

ArduCopter 提供多種飛行模式，透過繼承 `Mode` 基類實現：

| 模式 | 類別名稱 | 說明 |
|------|----------|------|
| STABILIZE | `ModeStabilize` | 手動控制機體角度，手動油門 |
| ACRO | `ModeAcro` | 手動控制機體框架角速率，手動油門 |
| ALT_HOLD | `ModeAltHold` | 手動控制機體角度，自動油門保持高度 |
| AUTO | `ModeAuto` | 使用任務命令進行全自動航點控制 |
| GUIDED | `ModeGuided` | 使用地面站即時命令進行全自動飛向座標 |
| LOITER | `ModeLoiter` | 自動水平加速，自動油門保持位置 |
| RTL | `ModeRTL` | 自動返回起飛點 |
| LAND | `ModeLand` | 自動降落並控制水平位置 |
| SPORT | `ModeSport` | 手動地球框架角速率控制，手動油門 |
| FLIP | `ModeFlip` | 自動沿滾轉軸翻轉飛行器 |
| AUTOTUNE | `ModeAutoTune` | 自動調整飛行器的滾轉和俯仰增益 |
| POSHOLD | `ModePosHold` | 自動位置保持，帶手動覆蓋，自動油門 |
| BRAKE | `ModeBrake` | 使用慣性/GPS系統進行完全剎車 |
| THROW | `ModeThrow` | 使用慣性/GPS系統進行拋擲啟動模式 |
| SMART_RTL | `ModeSmartRTL` | 智能返航通過重溯其路徑返回家 |
| FLOWHOLD | `ModeFlowHold` | 使用光流保持位置，無需測距儀 |
| ZIGZAG | `ModeZigZag` | 以預定義的A點和B點以Z字形方式飛行 |

### Mode 基類

每個飛行模式都繼承 `Mode` 基類，實現三個主要方法：

```cpp
class Mode {
public:
    virtual bool init();
    virtual void run() = 0;
    virtual void exit();

protected:
    Copter *copter;
    AC_WPNav *wpnav;        // 航點導航
    AC_Loiter *loiter;     // 盤旋控制器
    AC_PosControl *pos_control;   // 位置控制器
    AC_AttitudeControl *attitude_control;  // 姿態控制器
    AP_Motors *motors;      // 馬達輸出
};
```

### 模式切換邏輯

模式切換由 `Copter::set_mode()` 函數處理：

```cpp
bool Copter::set_mode(Mode *new_mode, mode_reason_t reason) {
    // 1. 更新模式切換原因
    _last_reason = reason;

    // 2. 檢查是否已在目標模式
    if (flightmode == new_mode) {
        return true;
    }

    // 3. 執行安全檢查
    // - GPS 有效性檢查
    // - EKF 高度檢查
    // - 地理圍欄檢查
    // - RC 失控保護檢查

    // 4. 初始化新模式
    if (!new_mode->init()) {
        return false;
    }

    // 5. 退出當前模式
    if (flightmode != nullptr) {
        flightmode->exit();
    }

    // 6. 更新模式指標
    flightmode = new_mode;

    // 7. 記錄並通知 GCS
    return true;
}
```

---

## 控制系統架構

ArduCopter 採用**級聯控制架構**，從高層次指令到馬達輸出，包含四個主要層次。

### 控制迴圈層級

| 層級 | 類別 | 頻率 | 功能 |
|------|------|------|------|
| 飛行模式層 | `Mode` 類別 | - | 根據當前飛行模式產生位置、速度或姿態目標 |
| 位置控制層 | `AC_PosControl` | 50 Hz | 將 3D 位置目標轉換為姿態和推力指令 |
| 姿態控制層 | `AC_AttitudeControl` | 400 Hz | 將姿態目標轉換為機體角速率指令 |
| 馬達混控層 | `AP_MotorsMulticopter` | 400 Hz | 將機體角速率和推力指令混控為各個馬達的輸出 |

### 姿態控制器 (AC_AttitudeControl)

姿態控制採用兩級級聯控制器：

1. **外部迴圈（角度 P 控制器）**：將角度誤差轉換為期望角速率
2. **內部迴圈（角速率 PID 控制器）**：將角速率誤差轉換為馬達指令

#### 主要方法

```cpp
// 設定滾轉/俯仰的角度目標和偏航的角速率目標
void input_euler_angle_roll_pitch_euler_rate_yaw(
    float roll_angle_cd,      // 滾轉角度 (厘度)
    float pitch_angle_cd,     // 俯仰角度 (厘度)
    float yaw_rate_rads       // 偏航角速率 (rad/s)
);

// 主角速率控制迴圈
void rate_controller_run();

// 多旋翼專用的角速率控制器運行函數
void rate_controller_run_dt();
```

### 位置控制器 (AC_PosControl)

位置控制採用三級級聯控制器：

1. **位置 P 控制器**：位置誤差 → 期望速度
2. **速度 PID 控制器**：速度誤差 → 期望加速度
3. **加速度前饋**：將加速度轉換為姿態指令

#### 水平位置控制 (NE 軸)

```cpp
// 設定帶有運動學塑形的位置目標
void input_pos_NEU_m(const Vector3f& target_neu_cm);

// 運行位置和速度迴圈
void update_pos_vel_controller_NE();

// 將加速度轉換為姿態指令
void accel_to_lean_angles();
```

#### 垂直位置控制 (U 軸)

```cpp
// 運行垂直位置和速度迴圈
void update_pos_vel_controller_U();

// 運行速度和加速度迴圈
void update_vel_accel_controller_U();
```

### 馬達混控 (AP_MotorsMulticopter)

將滾轉、俯仰、偏航和油門指令轉換為單個馬達輸出。

#### 主要方法

```cpp
// 將指令發送到馬達
void output();

// 設定偏航裕度
void set_yaw_headroom(uint16_t yaw_headroom);

// 更新估計的懸停油門
void update_throttle_hover();

// 返回馬達使用的輸出位元遮罩
uint32_t get_motor_mask() const;
```

---

## 飛行安全設計

### 預飛檢查 (AP_Arming)

`AP_Arming` 類別在飛行器起飛前執行一系列檢查：

- GPS 定位品質 (`gps_checks()`)
- IMU 健康狀況 (`ins_checks()`)
- 指南針校準 (`compass_checks()`)
- 電池電量 (`battery_checks()`)
- 遙控器校準
- 感測器一致性

可透過 `ARMING_CHECK` 參數（位元遮罩）選擇檢查項目。

### 失控保護 (Failsafes)

飛行中的保護機制：

- 遙控器訊號丟失
- 地面站遙測丟失
- 電池電量過低
- EKF 故障
- 地理圍欄突破

觸發時執行：自動降落、返航 (RTL) 或智慧返航 (SmartRTL)

### 看門狗重置 (Watchdog Reset)

- 如果主迴圈停止運行，自動重置 CPU
- 快速啟動並恢復姿態估計和位置
- 使用 `AP_HAL::Util::PersistentData` 儲存持久性資料

---

## 姿態估計系統

ArduCopter 的姿態估計系統主要由 `AP_AHRS` (Attitude and Heading Reference System) 庫提供。

### AP_AHRS 功能

- 融合 IMU、GPS、氣壓計等傳感器數據
- 提供飛行器的姿態、位置和速度估計
- 使用 Extended Kalman Filter (EKF) 進行數據融合

---

## 核心架構

### AP_Vehicle 基類

所有飛行器類型都繼承自 `AP_Vehicle`：

- 提供通用功能（初始化、排程、序列埠管理）
- 實現 `setup()` 和 `loop()` 作為主執行迴圈入口
- 飛行器類型在編譯時透過預處理器標誌確定

### AP_HAL (硬體抽象層)

確保軟體跨平台相容性，支援多種 RTOS：

- **ChibiOS**
- **NuttX**

### 共享核心函式庫

| 函式庫 | 功能說明 |
|--------|----------|
| `AP_AHRS` | 姿態航向參考系統 |
| `AP_Scheduler` | 排程器 |
| `AP_Mission` | 任務系統 |
| `GCS_MAVLink` | 地面控制站通訊 |
| `AP_Logger` | 資料記錄 |
| `AP_Param` | 參數系統 |
| `AP_GPS` | GPS 定位 |
| `AP_InertialSensor` | 慣性傳感器 |

---

## 通訊系統

### MAVLink 通訊

- **`GCS_MAVLINK` 類**：管理單個 MAVLink 通訊通道
- **`MAVLink_routing` 類**：處理訊息在通道之間的轉發
- **訊息流**：具有可配置速率的組織方式

### CAN 匯流排

- 支援 DroneCAN 協定
- **`AP_MAVLinkCAN`** 類處理 CAN 訊框轉發

---

## Lua 腳本擴展系統

允許用戶在不改變核心韌體的情況下添加自訂功能。

### 核心組件

- **`AP_Scripting`** 類：管理 Lua 虛擬機，提供與 ArduPilot 內部 API 的綁定
- **`lua_scripts`** 類：管理腳本生命週期、載入和執行排程

### API 綁定架構

- 在 `bindings.desc` 中定義
- 由程式碼生成器產生 C++ 膠水程式碼

### 腳本執行模型

1. **初始化和載入**：從檔案系統載入，編譯成位元組碼
2. **沙盒化**：在受限環境中運行，限制 Lua 標準庫訪問
3. **協同排程**：透過返回函數和延遲時間使用協同多工
4. **CPU 時間限制**：透過 Lua 調試掛鉤機制強制執行

---

## 測試與模擬

- **Autotest Framework**：自動化測試框架
- **SITL (Software In The Loop)**：軟體在環模擬
- **Log Replay**：日誌回放分析

---

## 檔案結構

```
ArduCopter/
├── Copter.h              # Copter 類別定義
├── Copter.cpp            # Copter 類別實現
├── mode.h                # Mode 基類定義
├── mode.cpp              # Mode 基類實現
├── mode_*.cpp            # 各飛行模式實現
│   ├── mode_stabilize.cpp
│   ├── mode_althold.cpp
│   ├── mode_loiter.cpp
│   ├── mode_auto.cpp
│   └── ...
├── sensors/              # 感測器相關
├── control/              # 控制系統相關
└── defs.h                # 定義和常數
```

---

## 設計理念總結

1. **通用性與專業化並存** - 共享核心 + 專門韌體
2. **可擴展性** - Lua 腳本、自定義控制器
3. **可靠性與安全性** - 嚴格預飛檢查、失控保護
4. **跨平台相容性** - AP_HAL 抽象層

---

## 參考資源

- [ArduPilot 官方網站](https://ardupilot.org/)
- [ArduPilot GitHub](https://github.com/ArduPilot/ardupilot)
- [ArduCopter 文檔](https://ardupilot.org/copter/)
