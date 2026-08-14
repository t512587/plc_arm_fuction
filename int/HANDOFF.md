# 第二台車手臂系統 — 技術交接文件

> 最後更新：2026-08-07

---

## 1. 檔案架構與角色

### 1.1 主程式

| 檔案 | 角色 |
|---|---|
| `start_daemons.py` | **一鍵啟動器**。同時啟動 `d435_camera_daemon.py` 跟 `canbus_daemon.py` 兩個常駐服務（各自獨立process，一個掛掉不影響另一個），統一印出兩邊的log，Ctrl+C時兩個都乾淨關閉。**使用相機或手臂功能之前，請先執行 `python start_daemons.py`**。 |
| `d435_camera_daemon.py` | **相機常駐服務**。獨立進程，開機時開一次 RealSense pipeline 並持續串流，背景快取最新一張對齊後的 RGB-D 畫面，透過本機 HTTP（預設 `127.0.0.1:8756`）的 `/frame`、`/health` 提供給其他程式抓取，避免每次拍照都重開相機。 |
| `canbus_daemon.py` | **CAN匯流排常駐服務**。獨立進程，開機時 `ArmController.connect()` 一次並持續保持連線，透過本機 HTTP（預設 `127.0.0.1:8757`）提供讀角度、移動到點位、單軸移動、停止/關閉馬達等指令，避免每次手臂動作都重新開序列埠、重跑「Warming up CAN adapter (1s)」暖機。 |
| `canbus/remote_arm_controller.py` | `RemoteArmController`／`RemoteMotorService`：跟 `canbus/arm_controller.py`／`canbus/motor_service.py` 介面完全相容的HTTP替身，`d435_control.py` 用它取代直接建立 `ArmController()`，改成向 `canbus_daemon.py` 下指令。 |
| `d435_control.py` | **手臂主程式**。控制 D435i 拍 RGB-D（實際畫面向 `d435_camera_daemon.py` 用 HTTP 索取），呼叫 SAM2 suction-hotspot server 取得吸取點，用 calibration model 把 camera x/y 轉成 ID142/ID143 目標角度，透過 `RemoteArmController` 向 `canbus_daemon.py` 下達馬達指令。支援 LView（左取右放）和 RView（右取左放）雙方向。若相機或CAN常駐服務未啟動，相關動作會直接拋出明確錯誤，不會自己另開一個 pipeline/序列埠。 |
| `calibration_ui.py` | 校正採點 Tkinter UI。呼叫主程式做 detect，顯示 `control_result.png`，讓使用者用 `2motor_sync.py` 手動對齊後輸入實際角度，存入 CSV。**同樣依賴兩個常駐服務已啟動**，因為它是透過 `d435_control.py` 拍照跟動手臂。 |
| `calibration_fit_cli.py` | 把校正 CSV fit 成二次多項式 calibration coefficients 的 CLI 工具。 |
| `action_main.py` | 整合 PLC + D435 UI 的啟動入口，載入 `plc2/main.py`。**不會自動啟動常駐服務**，`vision_transfer` 相關流程要用相機/手臂前，仍需另外手動執行 `start_daemons.py`（樹莓派上長期部署可考慮之後另外設定 systemd 開機自動啟動）。 |

### 1.2 canbus/ 模組

| 檔案 | 角色 |
|---|---|
| `canbus/arm_controller.py` | 高階馬達控制介面：read_positions、go_to_point、stop_all、shutdown_all |
| `canbus/motor_service.py` | 底層 CAN/RMD 指令，支援 Waveshare USB-CAN-A 和 python-can |
| `canbus/motor_protocol.py` | CAN 協定編解碼（arbitration ID、command data、response parsing） |
| `canbus/arm_config.py` | Motor ID mapping、速度常數、安全角度限制、config/point_config 載入 |
| `canbus/config.json` | USB-CAN port 設定（`/dev/ttyUSB0`、bitrate 1000000） |
| `canbus/point_config.json` | 固定點位：HOME、MOVE、LView、RView、LGrap、RGrap、STANDBY |
| `canbus/2motor_sync.py` | 手動控制/讀角度/STOP ALL/SHUTDOWN ALL 的 Tkinter UI 工具 |
| `canbus/tk_ui.py` | Tk UI 字型設定 helper（被 2motor_sync.py 依賴） |

### 1.3 PLC2 後端整合

| 檔案 | 角色 |
|---|---|
| `plc2/service/arm_vision_workflow_service.py` | **關鍵**：後端透過 subprocess 呼叫 `d435_control.py`，傳入 `--move-home-only`、`--home-tolerance` 等 CLI 參數。所有手臂動作都需要 `canbus_daemon.py` 已在背景執行；`vision_transfer`（`--execute`）額外需要 `d435_camera_daemon.py`。 |
| `plc2/config/services.yml` | `arm_vision_workflow.script_path` 指向 `d435_control.py` |
| `plc2/flow/main_cycle_flow.py` | 主循環流程引擎 |
| `plc2/api/main.py` | FastAPI 路由 |
| `plc2/ui/main.py` | Tkinter UI |

---

## 2. 第二台車專屬參數（不可與第一台車混用）

### 2.1 Calibration Coefficients

模型：`angle = a*x + b*y + c*x² + d*xy + e*y² + f`

**LView** (fitted from 24 samples):
```
ID 142: (-3.41643143, -0.85823981, 0.05802321, -0.09017803, 0.10025649, 58.07462456)
ID 143: ( 1.34783807,  3.65204515, -0.13589233, -0.01394930, -0.13336646, -88.48138011)
```

**RView** (fitted from 21 samples):
```
ID 142: ( 3.50379437, -0.77081708, -0.03201293, -0.09647447, -0.07305849, -48.91500422)
ID 143: (-1.42868671,  3.63586380,  0.10614333, -0.00233213,  0.09877633,  41.54382802)
```

> ⚠️ 第一台車 LView 常數項是 175.08，第二台車是 58.07，完全不同。絕對不能互相替換。

### 2.2 Prediction Limits（限制 vision 預測角度）

| View | ID 142 | ID 143 |
|---|---|---|
| LView | (9.0, 114.0) | (-172.0, -62.0) |
| RView | (-100.0, 14.0) | (15.0, 127.0) |

> 這些只限制 vision 預測的吸取角度。外圈搬運路徑的 ±360 分支不受此限制。

### 2.3 ROI

| View | ROI (x1, y1, x2, y2) |
|---|---|
| LView | (202, 117, 460, 312) |
| RView | (210, 186, 480, 388) |

### 2.4 Depth Range

| View | min (m) | max (m) |
|---|---|---|
| LView | 0.60 | 0.82 |
| RView | 0.60 | 0.82 |

### 2.5 Point Config（canbus/point_config.json）

| 點位 | ID142 | ID143 | ID144 | ID145 |
|---|---|---|---|---|
| HOME | 6.0 | -22.93 | 6.7 | 36.49 |
| MOVE | -7.01 | -22.87 | 6.71 | -15.71 |
| LView | 6.0 | -22.93 | -58.44 | 101.66 |
| RView | 6.0 | -22.93 | 75.88 | -33.05 |
| LGrap | 79.14 | -129.54 | 6.7 | 36.49 |
| RGrap | -65.25 | 70.35 | 6.7 | 36.49 |

### 2.6 Server 設定

目前主程式中設定為 Demo server：
```python
SERVER_URL = "https://demo.bizlion.com.tw/tmts/suction-hotspot/detect"
SERVER_AUTH = ("tmts", "N1++jI8eBOLTogEL0gLz5ehBhTqv50cjKonThISlrQo=")
```

Local GPU server（目前註解掉）：
```python
SERVER_URL = "http://192.168.50.233:8000/detect"
SERVER_AUTH = None
```

---

## 3. 主要流程

### 3.1 Vision-Only 測試

```bash
uv run python d435_control.py --view LView --target-speed 40 --yes
uv run python d435_control.py --view RView --target-speed 40 --yes
```

流程：`connect → HOME → LView/RView → capture D435i → call SAM2 server → save control_result.png → return HOME`

### 3.2 LView Execute（左取右放）

```bash
uv run python d435_control.py --view LView --execute --target-speed 40
```

流程：
```
HOME → LView → detect → HOME → LGrap → predicted left suction target
→ [Enter: 確認物件已吸取]
→ ID143 OuterMid(-) = HOME_ID143 - 180
→ ID142 right_mirror = 2*HOME_ID142 - left_ID142
→ ID143 right_outer_branch = (2*HOME_ID143 - left_ID143) - 360
→ [Enter: 確認物件已放開]
→ ID143 OuterMid(-) → ID142 HOME → ID143 HOME → HOME
```

### 3.3 RView Execute（右取左放）

```bash
uv run python d435_control.py --view RView --execute --target-speed 40
```

流程：
```
HOME → RView → detect → HOME → RGrap → predicted right suction target
→ [Enter: 確認物件已吸取]
→ ID143 OuterMid(+) = HOME_ID143 + 180
→ ID142 left_mirror = 2*HOME_ID142 - right_ID142
→ ID143 left_outer_branch = (2*HOME_ID143 - right_ID143) + 360
→ [Enter: 確認物件已放開]
→ ID143 OuterMid(+) → ID142 HOME → ID143 HOME → HOME
```

> ⚠️ 第一次跑 `--execute` 不要加 `--auto-step`，保留每段 Enter 確認安全。

### 3.4 Dry Run（不動馬達，只拍照偵測）

```bash
uv run python d435_control.py --view LView --dry-run
```

### 3.5 後端呼叫（ArmVisionWorkflowService）

後端呼叫主程式時會帶入以下 CLI 參數：

```bash
python d435_control.py --view LView --execute --yes \
  --target-speed 200 --settle 2.5 --target-wait 2.0 --pick-wait 2.0 \
  --home-tolerance 1.0 --home-stable-reads 3 --home-timeout 15.0 --home-poll 0.2
```

HOME-only 移動（例如只動 camera 回 HOME）：
```bash
python d435_control.py --move-home-only camera \
  --home-tolerance 1.0 --home-stable-reads 3 --home-timeout 15.0 --home-poll 0.2 --settle 2.5
```

指定姿態移動：
```bash
python d435_control.py --move-pose-only LView \
  --home-tolerance 1.0 --home-stable-reads 3 --home-timeout 15.0 --home-poll 0.2 --settle 2.5
```

---

## 4. 校正流程

### Step 1：採點

```bash
cd ~/int
uv run python calibration_ui.py
```

1. 選擇 LView 或 RView
2. UI 呼叫 `d435_control.py` 做一次 vision detect
3. 檢查 `control_result.png` 黃點位置
4. 用 `2motor_sync.py` 手動把吸頭對到黃點
5. 讀取實際 ID142 / ID143 角度
6. 存入 `calibration_points_LView.csv` 或 `calibration_points_RView.csv`
7. 重複 20+ 次

### Step 2：Fit

```bash
uv run python calibration_fit_cli.py --view LView
uv run python calibration_fit_cli.py --view RView
```

### Step 3：貼回主程式

把產出的 coefficients 貼回 `d435_control.py` 的 `CALIB_BY_VIEW` 和 `PREDICTION_LIMITS_BY_VIEW`。

---

## 5. 後端版本相容性

### 必須支援的 CLI 參數

`ArmVisionWorkflowService` 會用到以下參數（定義在 `_command()`、`_home_confirmation_command()`、`_home_component_command()`、`_named_pose_command()`）：

| 參數 | 用途 |
|---|---|
| `--view` | 選擇 LView/RView |
| `--execute` | 執行完整 pick flow |
| `--yes` | 跳過安全確認 |
| `--target-speed` | 移動速度 (dps) |
| `--settle` | 到位後等待秒數 |
| `--target-wait` | 目標位置等待秒數 |
| `--pick-wait` | PLC 交接等待秒數 |
| `--home-tolerance` | HOME 角度容差 |
| `--home-stable-reads` | HOME 連續穩定讀取次數 |
| `--home-timeout` | HOME 確認超時秒數 |
| `--home-poll` | HOME 讀取間隔秒數 |
| `--confirm-home-only` | 只確認 HOME，不移動 |
| `--move-home-only arm/camera` | 只移動 arm 或 camera 回 HOME |
| `--move-pose-only POSE` | 移動到指定姿態 |

### 版本錯誤排查

如果後端出現 `unrecognized arguments: --move-home-only ...`：

1. **不是** CAN bus 沒連上、不是馬達壞了
2. 代表跑到了**舊版**或**不同路徑**的腳本
3. 排查步驟：
   ```bash
   # 確認實際跑的是哪一份
   find . -name "d435_control*.py"
   
   # 確認版本是否有 home-only 支援
   uv run python d435_control.py --help | grep move-home-only
   
   # 確認 services.yml 設定
   cat plc2/config/services.yml | grep script_path
   ```

---

## 6. 安全注意事項

- `MOTOR_ANGLE_LIMITS` 在 `canbus/arm_config.py` 定義，ID 142 限制 (-70, 113)
- `MAX_MOVE_DEGREES = 180.0`：單次移動超過 180° 會被 safety blocked
- Prediction limits 只限制 vision 預測角度，外圈搬運路徑用 general motor limits
- `SIGTERM` handler 會自動呼叫 `stop_all_motors_confirmed()`
- 後端取消時會先送 `ARM_STOP_CONFIRMED` 再終止子程序
