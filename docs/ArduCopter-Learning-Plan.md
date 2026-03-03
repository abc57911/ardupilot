# ArduCopter 開發學習計劃

## 學習者背景

| 項目 | 情況 |
|------|------|
| 開發背景 | 網站工程師，無韌體開發經驗 |
| 目標 | 基於 ArduPilot 開發自定義功能或商業產品 |
| 興趣方向 | 感知與導航系統 + 任務管理與自主飛行 + 硬體整合 |
| 投入時間 | 每週 10-20 小時 |
| 硬體環境 | 有開發板（Pixhawk/H7）、測距儀（TF-Luna/TFmini Plus） |

---

## 階段一：基礎知識建立

**時間**：2-3 週（20-40 小時）

**目標**：建立嵌入式開發與 ArduPilot 的基礎概念

### 1.1 C++ 基礎複習

| 主題 | 說明 | 資源 |
|------|------|------|
| 指標與參照 |指標操作、參照傳遞、智慧指標（unique_ptr）| C++ Primer |
| 類別與繼承 | 多態、虛擬函數、存取控制 | C++ Primer |
| 模板 | 泛型程式設計、模板特化 | C++ Primer |
| 記憶體管理 | 堆疊/堆積配置、RAII 模式 | C++ Primer |
| STL 容器 | vector、array、map 的使用 | C++ Primer |

### 1.2 開發環境建置
    
> **本機環境**：Debian GNU/Linux 13 (trixie) - 原生系統（非 WSL）

#### 已安裝的工具

| 工具 | 版本 | 狀態 |
|------|------|------|
| OS | Debian 13 (trixie) | ✓ 已具備 |
| Git | 2.47.3 | ✓ 已安裝 |
| Python | 3.14.2 (pyenv) | ✓ 已安裝 |
| gcc/g++ | - | ✓ 已安裝 |
| make | - | ✓ 已安裝 |
| cmake | 3.31.6 | ✓ 已安裝 |

#### 需要安裝的工具

```bash
# 安裝 cmake 及相關依賴
sudo apt update
sudo apt install cmake

# 安裝 ArduPilot 其他依賴（Debian/Ubuntu 通用）
sudo apt install build-essential python3-pip python3-dev
sudo apt install libtool pkg-config
sudo apt install gawk wget git-core

# 安裝 MAVProxy（地面站軟體）
pip3 install MAVProxy
```

#### ArduPilot 原始碼

| 項目 | 路徑 |
|------|------|
| 原始碼目錄 | `/home/disney/personal/ardupilot` |
| 初始化 submodule | `git submodule update --init --recursive` |

#### 編譯 ArduCopter

```bash
cd /home/disney/personal/ardupilot

# 設定目標板（sitl 為軟體模擬）
./waf configure --board sitl

# 編譯 ArduCopter
./waf copter

# 編譯產物位置：build/sitl/bin/arducopter
```

> **其他選項**：
> - `./waf -j8 copter` - 使用 8 核心並行編譯
> - `./waf clean` - 清理編譯產物

#### SITL 模擬器

```bash
cd /home/disney/personal/ardupilot

# 啟動 SITL 模擬器（ArduCopter）
Tools/autotest/sim_vehicle.py -v ArduCopter -f hexa --console --map
```

**參數說明**：
| 參數 | 說明 |
|------|------|
| `-v ArduCopter` | 指定飛行器類型 |
| `-f hexa` | 機架類型（六旋翼） |
| `-L KSFO` | 起始位置（舊金山機場） |
| `--console` | 開啟 MAVProxy 控制台 |
| `--map` | 開啟地圖顯示 |

#### 地面站軟體

| 軟體 | 說明 |
|------|------|
| QGroundControl | 跨平台地面站（推薦 Linux 使用） |
| Mission Planner | Windows 地面站 |

> **Note**：QGroundControl 可從 https://qgroundcontrol.com/ 下載

**參考文檔**：
- [ArduPilot Developer Documentation](https://ardupilot.org/dev/)
- [Building the Code](https://ardupilot.org/dev/docs/building-the-code.html)
- [SITL Simulator](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html)

### 1.3 專案結構理解

```
ardupilot/
├── ArduCopter/          # 多旋翼飛行器韌體
│   ├── Copter.h/cpp    # 主類別
│   ├── mode*.cpp       # 飛行模式
│   └── defs.h          # 常數定義
├── libraries/           # 共享函式庫
│   ├── AC_AttitudeControl/  # 姿態控制
│   ├── AC_PosControl/       # 位置控制
│   ├── AP_AHRS/             # 姿態估計
│   ├── AP_Mission/          # 任務系統
│   └── AP_HAL/              # 硬體抽象層
├── modules/             # MAVLink、通訊模組
├── tools/              # 開發工具、模擬器
└── waf                 # 編譯系統
```

### 1.4 核心概念入門

| 概念 | 說明 |
|------|------|
| AP_HAL | 硬體抽象層，確保跨平台相容性 |
| AP_Vehicle | 飛行器基類，提供通用功能 |
| AP_Scheduler | 任務排程系統 |
| 飛行模式 | Mode 基類與各模式實現 |

**實作練習**：
1. 完成 SITL 環境建置並啟動模擬器
2. 使用地面站連接並驗證基本功能
3. 嘗試切換不同飛行模式

---

## 階段二：核心系統深入

**時間**：4-6 週（40-80 小時）

**目標**：掌握與開發目標相關的核心子系統

### 2.1 飛行模式架構

**相關性**：任務管理的基礎

| 模式類別 | 檔案 | 功能 |
|----------|------|------|
| Mode | mode.h/cpp | 基類，定義 init/run/exit 介面 |
| ModeAuto | mode_auto.cpp | 全自動任務執行 |
| ModeGuided | mode_guided.cpp | 地面站即時指令 |
| ModeLoiter | mode_loiter.cpp | 位置保持 |

**關鍵函數**：
- `Mode::init()` - 模式初始化
- `Mode::run()` - 模式主迴圈
- `Copter::set_mode()` - 模式切換

**學習重點**：
- 模式切換邏輯與安全檢查
- 模式間的狀態繼承

### 2.2 姿態估計系統

**相關性**：感知導航的基礎

| 組件 | 說明 |
|------|------|
| AP_AHRS | 姿態與航向參考系統 |
| EKF | Extended Kalman Filter 多感測器融合 |
| AP_InertialSensor | IMU 原始數據處理 |
| AP_GPS | GPS 定位處理 |
| AP_Baro | 氣壓高度計 |

**關鍵概念**：
- EKF 狀態估計（位置、速度、姿態）
- 感測器健康狀態監控
- 資料融合權重調整

**實作練習**：
1. 解讀 EKF 狀態日誌
2. 分析感測器數據品質

### 2.3 控制系統

**相關性**：所有飛行功能的基礎

| 控制器 | 類別 | 功能 |
|--------|------|------|
| 位置控制 | AC_PosControl | 位置 → 速度 → 加速度 → 姿態 |
| 姿態控制 | AC_AttitudeControl | 姿態 → 角速率 → 馬達輸出 |
| 馬達混控 | AP_MotorsMulticopter | 滾轉/俯仰/偏航/油門 → 馬達PWM |

**控制迴圈頻率**：
- 位置控制：50 Hz
- 姿態控制：400 Hz
- 馬達輸出：400 Hz

**學習重點**：
- 級聯控制架構
- PID 控制器調參
- 運動學限制

### 2.4 感測器框架

**相關性**：硬體整合

| 感測器類型 | 支援協定 | 範例 |
|------------|----------|------|
| 測距儀 | I2C, UART, SPI | TF-Luna, VL53L0X |
| 光流感測器 | SPI | PMW3901 |
| GPS | UART | u-blox, HERE+ |
| 羅盤 | I2C | RM3100, HMC5883 |

**驅動程式結構**：
```cpp
class AP_RangeFinder {
public:
    // 初始化
    bool init();

    // 讀取距離
    float distance_cm() const;

    // 狀態檢查
    SensorStatus status() const;
};
```

**實作練習**：
1. 整合現有的測距儀
2. 讀取並驗證距離數據

### 2.5 MAVLink 通訊

**相關性**：地面站整合

| 組件 | 說明 |
|------|------|
| GCS_MAVLink | 地面站通訊管理 |
| MAVLink_routing | 訊息路由 |
| MAVLink FTP | 檔案傳輸 |

**常用訊息**：
- HEARTBEAT - 飛行器狀態
- GLOBAL_POSITION - 全球位置
- LOCAL_POSITION_NED - 區域位置
- COMMAND_ACK - 指令確認

**學習重點**：
- 自定義 MAVLink 訊息
- 遙測資料流配置

---

## 階段三：專題開發實踐

**時間**：6-8 週（60-120 小時）

**目標**：針對興趣方向進行實際開發

### 方向 A：感知與導航系統

#### 3.A.1 光流感測器整合

| 主題 | 說明 |
|------|------|
| AP_OpticalFlow | 光流感測器驅動 |
| 光流數據融合 | 與 EKF 整合 |
| 光流位置估計 | 無 GPS 環境下的位置估算 |

#### 3.A.2 測距儀數據處理

| 主題 | 說明 |
|------|------|
| AP_RangeFinder | 測距儀框架 |
| 障礙物檢測 | 距離閾值判斷 |
| 地形跟隨 | 地形追蹤功能 |

#### 3.A.3 簡單避障邏輯

| 主題 | 說明 |
|------|------|
| 障礙物地圖 | 障礙物資訊管理 |
| 避障決策 | 檢測到障礙時的反應 |
| 航線重規劃 | 繞過障礙物 |

### 方向 B：任務管理與自主飛行

#### 3.B.1 自定義任務指令

| 主題 | 說明 |
|------|------|
| MAVLink Mission | 任務協議 |
| AP_Mission | 任務管理系統 |
| 自定義指令 | 添加新的任務命令 |

#### 3.B.2 航點行為擴展

| 主題 | 說明 |
|------|------|
| Nav_CMD | 導航命令 |
| Do_CMD | 動作命令 |
| 條件命令 | 執行條件判斷 |

#### 3.B.3 進階導航

| 主題 | 說明 |
|------|------|
| AC_WPNav | 航點導航 |
| 軌跡規劃 | 平滑路徑生成 |
| 目標追蹤 | 移動目標跟隨 |

### 方向 C：硬體整合

#### 3.C.1 I2C/SPI 設備驅動

| 主題 | 說明 |
|------|------|
| AP_HAL::I2C | I2C 通訊介面 |
| AP_HAL::SPI | SPI 通訊介面 |
| 設備驅動框架 | 標準化驅動結構 |

#### 3.C.2 CAN 匯流排設備

| 主題 | 說明 |
|------|------|
| DroneCAN 協議 | CAN 匯流排通訊 |
| AP_MAVLinkCAN | CAN 訊息轉發 |
| 馬達/電調整合 | CAN 馬達控制 |

#### 3.C.3 PWM 與馬達控制

| 主題 | 說明 |
|------|------|
| AP_Motors | 馬達輸出框架 |
| 電調協定 | PWM, DShot, CAN |
| 安全機制 | 馬達鎖定、輸出監控 |

---

## 階段四：開發流程與測試

**時間**：2-3 週（20-40 小時）

**目標**：建立正確的開發workflow

### 4.1 程式碼規範

| 規範項目 | 說明 |
|----------|------|
| 命名慣例 | 類別、函數、變數命名規則 |
| 程式碼格式 | indent、braces、max line length |
| 註解規範 | 檔案頭、函數說明 |
| 編譯警告 | 零警告原則 |

**參考資源**：
- [ArduPilot Coding Style](https://ardupilot.org/dev/docs/code-style-guide.html)

### 4.2 測試框架

| 測試類型 | 說明 |
|----------|------|
| 單元測試 | AP_Test 框架 |
| SITL 測試 | 自動化模擬測試 |
| 硬體測試 | 實際飛行器測試 |

**測試工具**：
- autotest.py - 自動化測試腳本
- MAVLink 模擬器 - 訊息注入測試

### 4.3 從模擬到硬體

| 階段 | 說明 |
|------|------|
| SITL | 純軟體模擬 |
| HIL | 硬體在環模擬 |
| 實際飛行 | 安全場地測試 |

**安全須知**：
- 始終在安全場地測試
- 準備緊急停止機制
- 逐步驗證每個功能

---

## 硬體建議清單

### 必備硬體

| 類別 | 建議型號 | 用途 |
|------|----------|------|
| 飛行控制器 | Pixhawk 6C / CubeOrange | 主要開發平台 |
| 測距儀 | TF-Luna / TFmini Plus | **已具備** |
| GPS | HERE+ 3 / u-blox Neo-M9N | 室外定位 |
| 遙控器 | 支援 PPM/SBUS | RC 輸入 |
| 電調 | BLHeli_32 / CAN電調 | 馬達控制 |

### 建議擴充硬體

| 類別 | 建議型號 | 用途 |
|------|----------|------|
| 光流感測器 | PMW3901 / OpenFlow | 室內定位 |
| 風扇 | 散熱風扇 | 測距儀散熱 |
| 數傳 | 915MHz / 2.4GHz | 遙測數傳 |
| 電池 | 3S/4S LiPo | 電源供應 |

### 預估費用

| 項目 | 預估價格（USD） |
|------|-----------------|
| Pixhawk 6C | $150-200 |
| GPS 模組 | $50-80 |
| 遙控器 | $100-200 |
| 數傳 | $50-100 |
| **總計** | **$350-580** |

---

## 推薦學習資源

### 官方文檔

| 資源 | 網址 |
|------|------|
| ArduPilot 官網 | https://ardupilot.org/ |
| Developer 文檔 | https://ardupilot.org/dev/ |
| Copter 文檔 | https://ardupilot.org/copter/ |
| GitHub | https://github.com/ArduPilot/ardupilot |

### 代碼查詢工具

| 工具 | 說明 |
|------|------|
| DeepWiki | AI 驅動的代碼庫文檔查詢工具，可透過自然語言查詢 ArduPilot 程式碼架構、函數定義與使用方式 |

> **使用方式**：當遇到不熟悉的 API、類別或函數時，可使用 DeepWiki 快速查詢其在代碼庫中的定義、上下文與相關用法。

### 社群資源

| 資源 | 說明 |
|------|------|
| ArduPilot Discord | 開發者交流頻道 |
| ArduPilot 論壇 | 技術討論區 |
| DroneCode SDK | MAVLink 客戶端庫 |

### 書籍推薦

| 書籍 | 說明 |
|------|------|
| C++ Primer | C++ 語法基礎 |
| 多旋翼飛行器設計 | 無人機基礎理論 |
| PID 控制系統 | 飛行控制理論 |

---

## 預期成果

完成本學習計劃後，您將具備：

1. **獨立開發能力** - 能夠獨立開發 ArduCopter 自定義功能
2. **完整開發流程** - 從模擬到實際硬體的開發經驗
3. **專題實作經驗** - 針對感知導航或任務管理的實際專案
4. **規範化程式碼** - 符合 ArduPilot 專案規範的程式碼產出
5. **測試與除錯** - SITL 測試與硬體調試能力

---

## 學習時間線總覽

| 週次 | 階段 | 主要內容 |
|------|------|----------|
| 1-2 | 階段一 | C++ 基礎、開發環境建置 |
| 3 | 階段一 | 專案結構理解、核心概念 |
| 4-5 | 階段二 | 飛行模式、姿態估計 |
| 6-7 | 階段二 | 控制系統、感測器框架 |
| 8-9 | 階段二 | MAVLink 通訊 |
| 10-15 | 階段三 | 專題開發實踐 |
| 16-18 | 階段四 | 開發流程、測試、優化 |

**總學習時間**：18 週（約 4-5 個月），每週 10-20 小時

---

## 學習進度追蹤

| 日期 | 項目 | 狀態 |
|------|------|------|
| 2026-03-02 | cmake 安裝 | ✓ 完成 |
| 2026-03-02 | submodule 初始化 | ✓ 完成 |
| 2026-03-02 | ArduCopter 編譯成功 | ✓ 完成 |

---

## 下一步行動

1. **本週**：完成開發環境建置（SITL）
2. **第 2 週**：閱讀 ArduCopter 原始碼結構
3. **第 3 週**：嘗試編譯並運行 SITL 模擬器
4. **第 4 週**：開始階段二的學習

---

*建立日期：2026-03-02*
*基於 ArduCopter 開發指南 v1.0*
