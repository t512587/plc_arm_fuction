# ITRI 機械手臂與 PLC 整合控制系統

本專案的主要操作入口為 [`int/action_main.py`](int/action_main.py)。它會啟動整合式 PLC、D435i 視覺與 CAN Bus 機械手臂控制介面，供現場執行取放流程、設備操作與狀態監看。

> 專案目前沒有 `action.py`；本文所稱的主程式是 `action_main.py`。

## 主要功能

- 透過 Mitsubishi PLC（MC Protocol）控制升降、真空與周邊設備。
- 以 D435i 擷取 RGB-D 影像，呼叫吸取點辨識服務。
- 透過 CAN Bus 控制四軸機械手臂，支援 LView（左取右放）與 RView（右取左放）。
- 支援命令列全自動流程，可直接用 `action_main.py --height ...` 啟動 PLC 與手臂串接。
- 提供 Tkinter 圖形介面，以及 FastAPI 後端服務。
- 具備 HOME 確認、移動角度限制與流程取消時停止馬達等安全機制。

## 專案結構

```text
.
├─ README.md
└─ int/
   ├─ action_main.py          # 主程式入口：整合 PLC + D435 操作介面
   ├─ requirements.txt        # 主要相依套件
   ├─ d435_control.py         # D435i、吸取點辨識與手臂取放控制
   ├─ plc2/                   # PLC 後端、流程與操作介面
   ├─ canbus/                 # CAN Bus 馬達控制與手動操作工具
   ├─ calibration_ui.py       # 校正採點輔助介面
   └─ calibration_fit_cli.py  # 校正資料擬合輔助工具
```

## 環境需求

- Python 3.10～3.12
- `uv`（建議使用）或 Python 虛擬環境
- Intel RealSense D435i
- 已設定完成的 CAN Bus 轉接器與機械手臂
- 可連線的 Mitsubishi PLC

請先依現場設備確認以下設定：

- `int/plc2/config/plc_config.yml`：PLC IP、連接埠與通訊設定。
- `int/canbus/config.json`：CAN 介面、序列埠與鮑率。
- `int/canbus/point_config.json`：手臂 HOME、LView、RView 等姿態。

## 安裝

在 `int` 目錄中安裝依賴：

```powershell
cd int
uv sync
```

若不使用 `uv`：

```powershell
cd int
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

## 啟動主程式

在 `int` 目錄執行：

```powershell
uv run python action_main.py
```

或在已啟用虛擬環境時執行：

```powershell
python action_main.py
```

程式會：

1. 啟動本機 FastAPI PLC 後端（預設使用連接埠 `8000`）。
2. 開啟 Tkinter 整合操作介面。
3. 由介面執行 PLC、視覺與機械手臂流程。

## 命令列自動任務

`action_main.py` 除了啟動圖形介面，也支援直接執行自動化流程。這個模式適合現場已確認流程、只想輸入目標高度就讓 PLC 與機械手臂連動的情況。

### 基本指令

在 `int` 目錄執行：

```powershell
uv run python action_main.py --height 350 --yes
```

或在已啟用虛擬環境時：

```powershell
python action_main.py --height 350 --yes
```

### 參數說明

- `--height 350`
  - 指定這次自動任務的取料高度，單位是 mm。
  - 系統會先把升降機送到最低，再依流程進入視覺與手臂交接。
- `--forward-mm 400`
  - 指定 Y1 / Y2 貨盤前進距離，預設是 `400`。
  - 若現場要微調推進距離，可以改這個值。
- `--yes`
  - 代表你已確認要控制真實硬體。
  - 沒有這個參數時，程式只會拒絕進入實機模式，避免誤觸。
- `--dry-run`
  - 只顯示流程摘要，不會連 PLC，不會動硬體。
  - 可先用來確認參數是否正確。
- `--status`
  - 顯示上一個自動任務寫入的快照。
  - 方便確認目前停在哪個階段、最後一次讀到的 PLC 與手臂狀態。

### 自動流程內容

目前這條自動任務會依序做：

```text
最低點 → STANDBY
→ Y1 / Y2 前進並吸取
→ 回最低點
→ 升到 560mm 安全高度
→ 啟動 LView 手臂與 D435 流程
→ 取料高度進場
→ 轉移到跨側安全高度
→ 放料後回 560mm
→ 回最低點
→ Y1 / Y2 推料並回原點
→ 回最低點
```

流程中會保留最後一筆快照到：

`int/plc2/runtime/auto_transfer_last.json`

如果中途中止，這份快照可用來查最後停在哪個 stage、PLC 大致位置與手臂回報狀態。

## 系統結構與資料流

目前主程式只載入 `plc2/`：`action_main.py` 先切換到 `int/plc2`，再執行 `plc2/main.py`。`int/plc/`、`int/plc3/` 是其他版本或舊版模組，修改主流程時應以 `plc2/` 為準。

```text
action_main.py
  └─ plc2/main.py
      ├─ api/main.py                 FastAPI：PLC 與流程 API
      └─ ui/main.py                  Tkinter：現場操作介面
          └─ flow/main_cycle_flow.py 主循環／安全狀態機
              ├─ service/*.py        各設備的單一職責操作
              │   └─ service/plc_service.py
              │       └─ plc/client.py → Mitsubishi MC Protocol → PLC
              └─ service/arm_vision_workflow_service.py
                  └─ subprocess 執行 d435_control.py
                      └─ canbus/ → USB-CAN 轉接器 → ID142～ID145 馬達
```

設計原則是 **UI／API 不直接寫 D、M 位址，Flow 不直接控制通訊，硬體位址集中在設定檔**。這樣變更現場點位或設備時，可把影響範圍限制在設定與相對應的 Service。

### PLC 模組分層

| 層級 | 位置 | 責任 |
| --- | --- | --- |
| 連線設定 | `plc2/config/plc_config.yml` | PLC 名稱、IP、Port、MC Protocol Unit。 |
| 點位字典 | `plc2/config/points.yml` | 每個業務名稱對應的 D／M 位址、型別、可寫入權限。 |
| 設備映射 | `plc2/config/services.yml` | 將 Service 使用的角色（例如升降目標、Y1 真空）映射到 point ID。 |
| 流程參數 | `plc2/config/flows/*.yml` | 高度、容差、輪詢週期、逾時等流程參數。 |
| PLC 通訊 | `plc2/plc/client.py`、`plc2/service/plc_service.py` | MC Protocol、型別轉換、讀寫與通訊錯誤處理。 |
| 設備服務 | `plc2/service/` | 升降、Y 軸、真空、貨盤、HOME 等單一設備動作。 |
| 流程 | `plc2/flow/` | 排列設備動作、等待回授、取消、逾時與安全檢查。 |

`points.yml` 是 PLC 位址的唯一來源。每筆資料包含 `id`、`device`、`address`、`type`、`writable` 等欄位；`PlcService` 依照這些欄位決定使用 D word 或 M bit 讀寫。D 點可使用 `s16`、`u16`、`s32`、`u32`、`f32`，並可用 `scale` 將 PLC 原始值轉成工程單位。

目前主要點位如下：

| 功能 | 業務 point ID | PLC 位址 | 用途 |
| --- | --- | --- | --- |
| 升降目前高度 | `X_CUR_POS` | `D52` | 讀取升降高度。 |
| 升降定位目標 | `X_FWD_POS` | `D500` | 寫入目標高度。 |
| 升降定位啟動／下降 | `X_Move`／`X_MOVE_DOWN` | `M376`／`M379` | 啟動定位或下降動作。 |
| 升降手動上／下 | `X_UP`／`X_DOWN` | `M350`／`M351` | 手動控制。 |
| 中央真空／破真空 | `X_VAC_ON`／`X_VAC_OFF` | `M54`／`M56` | 吸取與釋放。 |
| Y1 目前位置與目標 | `Y1_CUR_POS`／`Y1_FWD_POS` | `D62`／`D510` | Y1 貨盤位置與前進距離。 |
| Y1 前進與真空 | `Y1_MOVE`、`Y1_VAC_ON`、`Y1_VAC_OFF` | `M375`、`M50`、`M51` | Y1 動作與真空。 |
| Y2 目前位置與目標 | `Y2_CUR_POS`／`Y2_FWD_POS` | `D72`／`D560` | Y2 貨盤位置與前進距離。 |
| Y2 前進與真空 | `Y2_MOVE`、`Y2_VAC_ON`、`Y2_VAC_OFF` | `M374`、`M52`、`M53` | Y2 動作與真空。 |
| X／Y 軸 HOME 回授 | `X_HOME`、`Y1_HOME`、`Y2_HOME` | `M10`、`M11`、`M12` | HOME 狀態確認。 |

> `D500`、`D510`、`D560` 是目標數值；真正啟動設備的是對應 M 命令。修改流程時不可只寫 D 值而未處理啟動、停止與回授確認。

### PLC 的修改方式

1. **位址改變**：只修改 `plc2/config/points.yml` 中既有 point ID 的 `device`／`address`，不要在 Python 程式中搜尋取代 D/M 位址。
2. **新增 PLC 點位**：先新增 `points.yml` 定義；若供既有設備服務使用，再在 `services.yml` 增加映射，最後由 Service 使用 point ID。
3. **新增動作或設備**：建立／擴充 `plc2/service/` 中的 Service；跨設備順序、等待與安全條件放在 `plc2/flow/`，不要寫入 UI。
4. **調整時間與容差**：優先修改 `plc2/config/flows/main_cycle.yml`，避免把現場參數寫死在程式碼。
5. **變更前驗證**：先確認 `writable: true`、型別與範圍；PLC 寫入遇到通訊中斷時，程式刻意不會自動重送，以免硬體重複動作。

### 主循環與 PLC／手臂交接

`plc2/flow/main_cycle_flow.py` 管理主循環的階段與安全條件。核心順序為：

```text
第一次第一步：貨盤／升降定位與真空處理
  → 第二步：升降到安全高度 → D435 偵測 → CAN 手臂取料
  → PLC 移至計算後高度並開 M54 → 回跨側安全高度
  → 手臂換邊 → PLC 到放料高度並開 M56 → 手臂回 HOME
  → 第二次第一步：完成後續貨盤動作
```

第二步開始時，手臂與相機的 HOME 確認會失效，升降高度受程式硬性限制在 695 mm 內；只有在四軸再次讀回並確認 HOME 後，才可解除限制。取消流程時，Flow 會關閉 PLC 動作輸出並檢查回授；手臂子程序則需回報四顆馬達停止確認。

方向名稱要特別注意：PLC 的貨盤方向與相機視角命名相反。`Y1 → Y2` 使用 `RView`，`Y2 → Y1` 使用 `LView`；對應邏輯在 `TransferDirection.camera_view`，修改名稱或方向時必須一併檢查此處與校正係數。

## CAN Bus 與手臂結構

CAN Bus 設定與姿態資料集中在 `int/canbus/`：

| 檔案 | 責任 | 修改時機 |
| --- | --- | --- |
| `config.json` | 轉接器介面、通道、CAN bitrate、序列鮑率、逾時。 | 更換 USB-CAN 轉接器、COM／`/dev/ttyUSB*` 或通訊參數。 |
| `motor_protocol.py` | RMD／CAN 封包編解碼。 | 更換馬達協定時才調整。 |
| `motor_service.py` | CAN 傳送、接收與連線管理。 | 更換通訊實作或處理方式。 |
| `arm_controller.py` | 手臂高階動作，如讀角度、到姿態、停止全部馬達。 | 新增通用手臂命令。 |
| `arm_config.py` | 馬達 ID、速度、角度限制與姿態設定載入。 | 改變馬達編號、安全限制或控制常數。 |
| `point_config.json` | HOME、MOVE、LView、RView、LGrap、RGrap 等四軸姿態。 | 實機重新示教固定姿態。 |
| `2motor_sync.py` | 手動讀角度、移動、停止與關機的輔助 UI。 | 維修、示教與校正時使用。 |

`arm_config.py` 的邏輯馬達名稱與 CAN 馬達 ID 對應如下：

| 邏輯名稱 | CAN 馬達 ID | 角色 |
| --- | --- | --- |
| `ID 142` | 2 | 吸盤大臂。 |
| `ID 143` | 3 | 吸盤小臂。 |
| `ID 144` | 4 | 相機小臂。 |
| `ID 145` | 5 | 相機大臂。 |

`point_config.json` 使用沒有空格的鍵名（例如 `ID142`），但載入時會由 `arm_config.load_point_config()` 轉成程式使用的 `ID 142` 格式。新增固定姿態時，請在 JSON 的 `points` 陣列增加一筆完整四軸角度；再確認 `d435_control.py` 或呼叫端是否已使用該姿態名稱。

CAN Bus 相關修改建議：

1. **只改轉接器或連接埠**：修改 `canbus/config.json`，不需要變更控制邏輯。
2. **只重新示教姿態**：修改 `canbus/point_config.json`；先低速測試，確認四軸角度都正確。
3. **變更馬達 ID**：同步修改 `canbus/arm_config.py` 的 `MOTORS`，並確認 CAN 實體編號與 UI／校正流程一致。
4. **放寬角度或單次移動限制**：修改 `MOTOR_ANGLE_LIMITS` 或 `MAX_MOVE_DEGREES` 前必須實機評估碰撞範圍；這些是安全限制，不應為了排除錯誤直接移除。

## 視覺與校正資料對應

`d435_control.py` 是 D435i、吸取點服務與 CAN 手臂之間的橋接程式。它會由 `plc2/service/arm_vision_workflow_service.py` 以子程序方式啟動；其路徑、Python 執行檔、逾時、HOME 容差與手臂速度設定都在 `plc2/config/services.yml` 的 `arm_vision_workflow` 區塊。

視覺偵測產生的相機座標會依 LView／RView 的校正模型換算為 ID142、ID143 目標角度。固定姿態在 `canbus/point_config.json`，而視覺座標對應係數在 `d435_control.py`；兩者用途不同，修改其中一個不會自動更新另一個。重新校正後，必須確認新係數與正確視角、正確車台一同更新。

## 操作前安全檢查

首次操作或更換設備設定後，請先確認：

- 手臂活動範圍內無人員與障礙物。
- 緊急停止、真空、升降與 CAN Bus 通訊正常。
- PLC 與 CAN Bus 設定為現場實際設備，且沒有使用其他車台的校正係數。
- 先在低速與受監看情況下測試 HOME、LView、RView。

## 校正輔助程式

下列程式不是日常主操作入口，僅在需要重新建立視覺座標與手臂角度的對應關係時使用。

### 圖形化採點：`calibration_ui.py`

```powershell
cd int
uv run python calibration_ui.py
```

用途：選擇 LView 或 RView，執行視覺偵測，確認吸取點後手動對位並輸入實際的 ID142／ID143 角度。採樣資料會分別存到：

- `calibration_points_LView.csv`
- `calibration_points_RView.csv`

建議每個視角至少蒐集 20 個分散且有效的點。採點完成後，按 UI 的 **Fit current view**，會產生對應視角的擬合結果檔。

### 命令列擬合：`calibration_fit_cli.py`

```powershell
cd int
python calibration_fit_cli.py
```

用途：這是使用單一 `calibration_points.csv` 的獨立命令列採點／擬合工具；可手動輸入座標與角度，或貼上 CONTROL TARGET JSON，再擬合為二次多項式：

```text
angle = a*x + b*y + c*x² + d*x*y + e*y² + f
```

至少需要 6 個點才能擬合，建議 20 點以上。它會輸出 `calibration_fit_result.txt`。完成後請檢查 RMSE 與最大誤差，確認可接受後，再將係數更新到 `d435_control.py` 的視角校正設定中。

> `calibration_fit_cli.py` 的 `calibration_points.csv` 與 `calibration_ui.py` 產生的 LView／RView CSV 是兩套不同的資料流程，請勿直接混用檔名。

> 校正係數與設備機構相關，不能直接混用不同車台或不同視角的結果。

## 其他工具

- `int/canbus/2motor_sync.py`：手動讀取／調整馬達位置，亦可用於校正時讀取實際角度。
- `int/d435_control.py`：單獨測試視覺與取放流程。

例如僅測試 LView 視覺偵測：

```powershell
cd int
uv run python d435_control.py --view LView --target-speed 40 --yes
```

## 注意事項

- 主程式會操作真實硬體；未完成安全確認前，請勿直接執行完整取放流程。
- 若後端埠號 `8000` 已被其他程式使用，請先停止該程式或調整 PLC2 的後端設定。
- 需要較完整的模組交接與流程細節時，可參閱 [`int/HANDOFF.md`](int/HANDOFF.md)。
