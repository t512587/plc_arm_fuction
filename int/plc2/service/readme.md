# plc2/service 執行流程與分析

這個資料夾是 `plc2` 的 service layer。它不直接決定 HTTP API，也不直接處理 PLC 通訊細節；它的責任是把 `config/services.yml` 與 `config/points.yml` 定義的點位，包成可呼叫的設備能力，並在這一層加上互鎖、狀態追蹤、容錯與背景保護。

## 目錄角色

- `plc_service.py`
  - service 層底座。
  - 管理 PLC 連線、點位讀寫、D/M 型別轉換、寫入前 guard、通訊重試策略。
- `home_service.py`
  - 歸零控制。
  - 一次啟動 X / Y1 / Y2 的 home 命令，並讀取各軸位置。
- `lift_service.py`
  - 升降與左右真空控制。
  - 處理高度讀值、上升/下降命令、停止、左右真空。
- `y_axes_service.py`
  - Y1 / Y2 前進定位控制。
  - 支援單軸或雙軸同時啟動，並提供停止。
- `middle_vacuum_service.py`
  - 中間真空與破真空互鎖控制。
  - 保證中間真空與中間破真空不會同時 ON。
- `slot_vacuum_service.py`
  - Y1 / Y2 載貨槽真空狀態管理。
  - 維護持久化狀態，並用 watchdog 持續補強「有貨時真空必須保持」。

## 實際裝配入口

這個資料夾本身不是主程式。實際建立物件的位置在 `plc2/api/main.py`：

1. 建立 `PLC_SERVICE = PlcService()`
2. 以同一個 `PLC_SERVICE` 注入：
   - `HomeService`
   - `LiftService`
   - `YAxesService`
   - `SlotVacuumService`
   - `MiddleVacuumService`
3. API 啟動時立即呼叫 `SLOT_VACUUM_SERVICE.start_watchdog()`

因此實際執行鏈是：

`HTTP/API -> domain service -> PlcService -> MitsubishiPLCClient -> PLC`

同時：

`SlotVacuumService watchdog -> PlcService -> PLC`

## 可呼叫指令總表

這個資料夾沒有獨立 CLI；實際可呼叫的是各 service class 的公開方法，通常由 API layer 或其他 Python 程式直接呼叫。

### `PlcService`

- `connect(plc_name="main_plc")`
  - 建立指定 PLC 連線。
- `disconnect(plc_name="main_plc")`
  - 中斷指定 PLC 連線。
- `is_connected(plc_name="main_plc")`
  - 回傳是否已連線。
- `get_point(point_id)`
  - 取得點位設定。
- `read_point(point_id)`
  - 依 `points.yml` 定義讀單一點位。
- `write_point(point_id, value)`
  - 依 `points.yml` 定義寫單一點位。
- `read_d_register(plc_name, address, count=1)`
  - 直接讀 D register。
- `write_d_register(plc_name, address, values)`
  - 直接寫 D register。
- `read_bit_device(plc_name, device, address, count=1)`
  - 直接讀 M/X/Y 等 bit device。
- `write_bit_device(plc_name, device, address, values)`
  - 直接寫 M/X/Y 等 bit device。
- `register_write_guard(point_guard=None, bit_guard=None)`
  - 註冊全域寫入保護規則。

### `HomeService`

- `precheck()`
  - 檢查 home 相關點位是否合法且 PLC 已連線。
- `set_home_commands(enabled)`
  - `True` 時啟動 X/Y1/Y2 歸零命令，`False` 時停止。
- `read_positions()`
  - 讀取 `X`、`Y1`、`Y2` 的位置回授。

### `LiftService`

- `precheck()`
  - 檢查升降與左右真空點位是否合法且 PLC 已連線。
- `read_height()`
  - 讀取目前高度。
- `set_vacuum(enabled)`
  - 同時開啟或關閉左右真空。
- `read_vacuum()`
  - 讀取左右真空狀態。
- `start_up()`
  - 啟動上升命令。
- `start_vision_positioning()`
  - 啟動視覺高度定位命令。
- `start_down()`
  - 啟動下降命令。
- `read_motion_commands()`
  - 讀取 `up/down` 兩個升降命令目前狀態。
- `stop()`
  - 停止升降命令。

### `YAxesService`

- `precheck()`
  - 檢查 Y1/Y2 定位點位是否合法且 PLC 已連線。
- `read_positions()`
  - 讀取 `Y1`、`Y2` 位置。
- `start_y1_positioning()`
  - 啟動 Y1 定位。
- `start_y2_positioning()`
  - 啟動 Y2 定位。
- `start_both_positioning()`
  - 同時啟動 Y1/Y2 定位。
- `stop()`
  - 停止所有 Y 軸定位命令。

### `MiddleVacuumService`

- `read_state()`
  - 讀取中間真空/破真空狀態，回傳 `off`、`vacuum`、`break_vacuum` 或 `invalid`。
- `set_mode(mode)`
  - 切換中間真空模式；支援 `off`、`vacuum`、`break_vacuum`。
- `wait_transfer_ready(vacuum_expected, cancel_event, progress_callback=None)`
  - 若 `services.yml` 有設定 `confirmation_point`，會等待 PLC 真空壓力
    bit 連續穩定後才允許手臂繼續。
  - 未設定感測點時，只依 `vacuum_settle_seconds` 或
    `release_settle_seconds` 定時等待，結果會明確標記為
    `timer_only`，不會宣稱已完成感測確認。

### 第二步手臂安全確認

`ArmVisionWorkflowService` 啟動的 D435i/CANBus 子程序會：

- 實際執行入口為專案根目錄的 `d435_control.py`。
- Camera 團隊的方向命名與 PLC Y 軸相反：`RView` 對應 Y1，`LView` 對應 Y2。
- UI 選擇 `Y1 → Y2` 時使用最新 21 點 `RView` 校正，執行右側取料、左側放料。
- UI 選擇 `Y2 → Y1` 時使用 `LView`，執行左側取料、右側放料。
- UI/API 指定的方向只覆寫本次子程序視角，不需要人工修改 `services.yml`。

### 三步驟 UI 人工確認

- 第一步完成後流程停在 `waiting_step2`，UI 顯示「第二步執行確認」視窗。
- 只有按下視窗內的「繼續進行第二步」才會送出第二步 API；關閉視窗或
  選擇「稍後再執行」都會維持等待。
- 第二步完成後流程停在 `waiting_final_step1`，UI 顯示「第三步執行確認」視窗。
- 只有按下視窗內的「繼續進行第三步」才會送出第三步 API。
- 等待期間可從主畫面的按鈕重新開啟目前步驟的確認視窗。

### 第一步／第三步跨側真空保持

- 第一步選擇 `Y1 + 吸` 時，Y1 貨盤照設定移動，但真空鎖定在另一側 `Y2`。
- 第一步選擇 `Y2 + 吸` 時，真空鎖定在另一側 `Y1`。
- 鎖定側會寫入 `vacuum_required=True`，watchdog 會持續補強真空；
  第二步不會自動關閉它。
- 第三步選擇 `Y1 + 推` 或 `Y2 + 推`，代表操作者明確確認該側放回貨架；
  此時才關閉該側真空、開啟該側破真空，並清除保持狀態。

1. 確認 ID142～ID145 都能回讀。
2. 命令 HOME 後，依 `home_tolerance_degrees` 逐顆比較 HOME 角度。
3. 連續達到 `home_stable_reads` 次才允許前往本次選擇的 LView 或 RView。
4. 取放完成回 HOME 後，再次執行相同回讀。
5. 只有最終回讀成功才輸出 `[POSE_STATE] ALL_HOME_CONFIRMED`，並解除
   升降機的 695mm 限制。

UI/API 亦提供獨立的只讀確認：

- `POST /arm-camera-home/confirm`
- 不會命令馬達移動，只讀取 ID142～ID145。
- 使用 `home_tolerance_degrees`、`home_stable_reads`、
  `home_timeout_seconds` 與 `home_poll_interval_seconds` 判定。
- 成功才將共享互鎖設為 `home_confirmed`；失敗或逾時維持 `unknown`。

整合模式的升降機與手臂交接順序為：

1. 第二步先將升降機移到固定安全高度 560mm；不得由其他視覺高度設定放寬。
2. 升降定位命令停止、實際高度落在 560±1mm 且連續穩定兩次後，
   才啟動 Camera／手臂子程序。
3. 手臂到左側吸取姿勢，升降機移動到 D435i 算出的取料高度並開啟 M54。
4. 吸附等待完成後，貨物保持吸附，升降機先回到 560mm。
5. Camera 回 HOME 後先讀回 ID142～ID145，確認四軸 HOME 角度連續穩定；
   HOME 到 LGrap、進入吸取點、換到右側及回 HOME 等每一段手臂動作前，
   子程序都保持暫停；父程序重新停止升降定位並確認 560±1mm 連續穩定
   兩次後，才送出繼續訊號。
6. 手臂到右側放料姿勢後，升降機回到同一個取料高度並切換 M55。
7. 釋放等待完成後，升降機再次回到 560mm；確認通過後才放行手臂回 HOME。
8. 任一次確認超出 560±1mm，第二步立即中止，不會送出手臂繼續訊號。

取消第二步時，父程序先要求子程序送出四顆馬達的 0x81 停止命令。
四顆停止命令都有回覆時，子程序輸出
`[POSE_STATE] ARM_STOP_CONFIRMED`；若逾時或任一顆失敗，HOME 狀態
保持 unknown，695mm 限制不解除。UI 的取消仍不是實體急停。

### `SlotVacuumService`

- `list_states(include_vacuum=True)`
  - 列出 Y1/Y2 載貨槽狀態，必要時附帶 PLC 真空狀態。
- `update_state(side, occupancy, ..., confirm_release=False)`
  - 更新指定側載貨狀態；若改成 `empty` 會要求 `confirm_release=True`。
- `enforce_required_vacuum()`
  - 重新補強所有 `vacuum_required=True` 的側別。
- `start_watchdog()`
  - 啟動背景 watchdog，定期補強真空。
- `stop_watchdog()`
  - 停止 watchdog。

### `service/__init__.py` 匯出的可直接 import 名稱

- `PLC_SERVICE`
- `PlcService`, `PlcServiceError`
- `HomeService`, `HomeServiceError`
- `LiftService`, `LiftServiceError`
- `YAxesService`, `YAxesServiceConfig`, `YAxesServiceError`
- `SlotVacuumService`, `SlotVacuumServiceError`, `SlotState`, `SlotOccupancy`, `CargoPurpose`
- `MiddleVacuumService`, `MiddleVacuumServiceConfig`, `MiddleVacuumServiceError`, `MiddleVacuumMode`
- `LifecycleStatus`, `StatusSnapshot`

## 共通基礎：Config 與 Lifecycle

### Config 來源

所有 service 都依賴 `CONFIG_STORE`，由 `plc2/config/loader.py` 載入：

- `plc_config.yml`：PLC 連線資訊
- `points.yml`：點位定義，包含 device、address、type、writable、min/max、scale
- `services.yml`：service 與 point id 的映射

這代表 service 層幾乎不硬編碼位址，邏輯只認 point id 與 service 設定。例外是少數安全限制直接用 M 位址判斷。

### Lifecycle 追蹤

所有 service 都繼承 `LifecycleTracked`。大部分公開方法都用 `@tracked_operation(...)` 包住，因此每次呼叫都會更新：

- `status`
- `step`
- `message`
- `updated_at`
- 部分結果 `data.result`

狀態值主要有：

- `pending`
- `running`
- `waiting_signal`
- `success`
- `error`

這表示 service 層本身就是一個輕量狀態機，API 可直接把 service 當成可觀測元件。

## 核心底座：PlcService

`PlcService` 是所有 domain service 的唯一 PLC 入口。

### 主要職責

1. 依 `plc_name` 建立/關閉 client 連線
2. 透過 `point_id` 取得 `PointDefinition`
3. 封裝：
   - `read_point(point_id)`
   - `write_point(point_id, value)`
   - `read_d_register(...)`
   - `write_d_register(...)`
   - `read_bit_device(...)`
   - `write_bit_device(...)`
4. 處理 D register 編碼/解碼
5. 寫入前執行 guard
6. 讀取遇到 `PLCConnectionError` 時自動斷線重連再重試一次

### 讀寫流程

#### 讀點位

1. `get_point(point_id)`
2. 確認 PLC client 已連線
3. 若 device 是 `D`
   - 依 type 決定讀 1 word 或 2 words
   - 經 `_decode_d()` 轉成 `int` / `float`
4. 若不是 `D`
   - 走 bit device 讀取
5. 讀取若遇到 `PLCConnectionError`
   - `disconnect()`
   - `connect()`
   - 再讀一次

#### 寫點位

1. 先執行所有 `point_guard`
2. 檢查該點位是否 `writable`
3. 若為 D 點，檢查 `min/max`
4. 依型別編碼
5. 寫入一次

注意：寫入故障時不自動重送。`_write_once()` 對 `PLCConnectionError` 的策略是直接報錯，因為寫入是否已經落到 PLC 不可確定，避免重複動作。

### 型別支援

`PlcService` 支援：

- `s16` / `int`
- `u16`
- `s32`
- `u32`
- `f32`

且支援 `scale`。因此 `points.yml` 可用工程單位包裝原始 PLC 數值。

### 內建安全限制

`BLOCKED_M_ON_ADDRESSES = {389, 399}`

當使用 `write_bit_device("M", ...)` 直接寫 bit 時，如果要把 `M389` 或 `M399` 寫成 `ON`，會被拒絕。  
這個限制只作用在「原始 bit 寫入」，目的很明確：禁止直接啟動這些位址。

## 各 service 執行流程

## 1. HomeService

對應 `services.yml -> home`。

### 設定要求

- `command_points` 與 `position_points` 軸集合必須一致
- command point 必須是可寫 `M`
- position point 必須是 `D`

### 主要流程

#### `precheck()`

用途是開機前驗證設定與連線：

1. 確認 `PlcService.is_connected()`
2. 驗證所有 home command 點都是可寫 M 點
3. 驗證所有位置回授點都是 D 點

#### `set_home_commands(enabled)`

當 `enabled=True`：

1. 逐一把 X / Y1 / Y2 的 home command 寫成 `True`
2. 任一點位寫入失敗時：
   - 已成功寫入者全部回滾成 `False`
   - 將 lifecycle 標成 `error`
3. 全部成功後，狀態設成 `waiting_signal`

當 `enabled=False`：

1. 逐一把所有 home command 寫成 `False`
2. 成功後狀態標成 `success`

#### `read_positions()`

逐軸讀取位置點，回傳：

```python
{"X": float, "Y1": float, "Y2": float}
```

### 分析

- 這個 service 很薄，主要是批次寫入 home 命令。
- 它假設 PLC 端會自行完成 home sequence，Python 端只負責發命令與讀回位置。
- 失敗回滾邏輯存在，這是正確的，因為 home 命令通常不適合半套啟動。

## 2. LiftService

對應 `services.yml -> lift`。

### 功能範圍

- 讀取升降高度
- 啟動上升
- 啟動下降
- 啟動視覺高度定位
- 停止上下命令
- 控制左右真空

### 主要流程

#### `precheck()`

- `height_point` 必須是 D
- `up_point` / `down_point` / 左右真空點必須是可寫 M

#### `read_height()`

1. 讀取高度點位
2. 檢查回傳值必須是有限數值
3. 檢查是否落在 `minimum_height_mm ~ maximum_height_mm`

這裡把感測值合理性檢查放在 service 層，不是單純 passthrough。

#### `start_up()` / `start_vision_positioning()` / `start_down()`

三者都走 `_start(active_point, opposite_point, operation)`：

1. 先把反向命令清成 `False`
2. 再把目標命令寫成 `True`
3. 若失敗，呼叫 `stop()` 清理
4. 成功後 lifecycle 進入 `waiting_signal`

#### `stop()`

把 `up_point` 與 `down_point` 都寫成 `False`。

#### `set_vacuum(enabled)` / `read_vacuum()`

直接控制/讀取左右真空點。

### 分析

- `start_vision_positioning()` 與 `start_up()` 目前使用同一組點位。也就是說，從 service 層看不出兩者控制上有差異；差異可能只存在呼叫語意或 PLC 程式內部。
- 上下命令採互斥清理模式，避免上下同時啟動。
- 左右真空控制沒有在這個 service 自行做更深的互鎖，因為載貨真空保護是交給 `SlotVacuumService` 透過 write guard 接管。

## 3. YAxesService

對應 `services.yml -> y_axes`。

### 功能範圍

- 讀取 Y1 / Y2 位置
- 啟動 Y1 定位
- 啟動 Y2 定位
- 同時啟動 Y1 / Y2 定位
- 停止所有 Y 軸定位命令

### 主要流程

#### `precheck()`

- command points 必須是可寫 M
- position points 必須是 D

#### `start_y1_positioning()` / `start_y2_positioning()` / `start_both_positioning()`

都走 `_start(axes, operation)`：

1. 先 `stop()`，把 Y1/Y2 命令全部清成 `False`
2. 依指定軸逐一寫成 `True`
3. 若任一步失敗，再次 `stop()`
4. 成功後 lifecycle 進入 `waiting_signal`

#### `stop()`

把所有 command point 寫成 `False`。

### 分析

- 這個 service 的核心是「先全停，再選擇性打開」。這種做法簡單且可預測。
- 類別註解明寫：`Position Y1 with M388 and Y2 with M398; never use M389/M399.`
- 配合 `PlcService` 的 `BLOCKED_M_ON_ADDRESSES`，可以推斷系統明確區分「允許的定位啟動位」與「禁止直接啟動位」。

## 4. MiddleVacuumService

對應 `services.yml -> middle_vacuum`。

### 功能範圍

- 用單一 service 控制中間真空與中間破真空
- 保證兩者不可同時 ON

### 初始化流程

建構時就會：

1. 驗證兩個點都是可寫 M 點
2. 解析兩個 point 的實際 address
3. 向 `PlcService.register_write_guard()` 註冊：
   - `point_guard`
   - `bit_guard`

也就是說，這個 service 不只是提供自己的 API，還會改變整個 `PlcService` 的寫入規則。

### `set_mode(mode)`

支援：

- `off`
- `vacuum`
- `break_vacuum`

流程：

1. 解析 mode
2. 開啟 thread-local `bypass`
3. 依模式寫入兩個點位：
   - `vacuum`: break=false, vacuum=true
   - `break_vacuum`: vacuum=false, break=true
   - `off`: 全部 false
4. 關閉 bypass
5. `read_state()` 讀回驗證
6. 若讀回模式不一致，直接報錯

### guard 保護

#### `_guard_point_write`

當外部有人直接把：

- 真空點寫成 `True`
- 或破真空點寫成 `True`

就會先讀另一個點。如果另一個已經是 ON，直接阻擋。

#### `_guard_bit_write`

當外部有人用原始 `write_bit_device("M", ...)` 批量寫入時，也會做同樣互鎖判斷，避免一次寫出雙 ON。

### 分析

- 這個設計是正確的，因為它把互鎖收斂到 `PlcService` 層級，而不是只保證「大家都乖乖用 `MiddleVacuumService.set_mode()`」。
- `bypass` 設計必要，否則 service 自己切換模式時會被自己的 guard 卡住。
- `read_state()` 允許回報 `invalid`，表示 PLC 或外部操作造成雙 ON 異常狀態，但 `set_mode()` 本身不允許把 `invalid` 當控制命令。

## 5. SlotVacuumService

對應 `services.yml -> slot_vacuum`。

這是本資料夾最重的 service，因為它不只是 PLC 點位控制，還包含狀態持久化與背景保護。

### 功能範圍

- 管理 Y1 / Y2 載貨槽狀態
- 根據載貨狀態決定真空是否必須保持
- 防止外部誤關真空或誤開破真空
- 透過 watchdog 持續補強真空
- 將狀態保存到 `runtime/slot_states.json`

### 持久化模型

`SlotState` 內容包含：

- `side`
- `occupancy`
- `cargo_id`
- `purpose`
- `shelf_id`
- `shelf_level`
- `vacuum_required`
- `updated_at`

`SlotStateStore` 啟動時會從 `runtime/slot_states.json` 載入；若檔案不存在或解析失敗，回退到預設 `Y1` / `Y2 = unknown`。

### 初始化流程

建構時會：

1. 載入 config
2. 建立 state store
3. 建立 watchdog 所需 event/thread
4. 向 `PlcService` 註冊 `point_guard` 與 `bit_guard`

因此它和 `MiddleVacuumService` 一樣，屬於會「全域修改寫入規則」的 service。

### `update_state(...)`

這是核心方法。

#### 標記為 `occupied`

1. side 正規化成 `Y1` / `Y2`
2. 檢查 PLC 已連線
3. 推導 `vacuum_required=True`
4. 呼叫 `_ensure_side_vacuum(side)`
   - 若破真空 ON，先關掉
   - 若真空 OFF，打開
   - 最後讀回確認真空確實 ON
5. 將 state 寫入 JSON

#### 標記為 `unknown`

若前一狀態已要求真空，則保持 `vacuum_required=True`。  
也就是 unknown 不會自動釋放真空。

#### 標記為 `empty`

1. 必須傳入 `confirm_release=True`
2. 進入 `_allow_confirmed_release()` 暫時關閉 guard
3. 將該側真空與破真空都寫成 `False`
4. 清掉 cargo/shelf 相關欄位
5. 寫入 JSON

這裡很重要：`empty` 是有副作用的，不只是 metadata 更新，而是實際釋放真空。

### `enforce_required_vacuum()`

掃描所有 side：

1. 若 `vacuum_required=True`
2. 就呼叫 `_ensure_side_vacuum(side)`

成功後回傳實際補強了哪些 side。

### watchdog

`start_watchdog()` 會啟動 daemon thread：

1. 每隔 `enforcement_interval_seconds` 秒醒來
2. 呼叫 `enforce_required_vacuum()`
3. 失敗則吞掉例外，下一輪再試

API 啟動時就會啟動這個 watchdog。

### guard 保護

#### `_guard_point_write`

若某側 `vacuum_required=True`：

- 禁止把該側 vacuum point 寫成 `False`
- 禁止把該側 break vacuum point 寫成 `True`

#### `_guard_bit_write`

若有人用原始 bit 寫法，也禁止：

- `Mxxx = OFF` 關閉該側真空
- `Mxxx = ON` 開啟該側破真空

### 分析

- 這個 service 實際上是在做「載貨安全狀態機」。
- 它把 PLC 點位控制和業務狀態綁在一起：只要系統認為有貨，就不允許任何人隨意放掉真空。
- `confirm_release=True` 是刻意設計的硬確認，避免單純把狀態改成 `empty` 就誤釋放貨物。
- watchdog 讓保護不只發生在 API 呼叫時，也能對抗外部寫入或瞬時掉點。

## service 間交互關係

### 1. 共同依賴 `PlcService`

所有 domain service 都只透過 `PlcService` 存取 PLC。  
這讓 guard、型別轉換、錯誤處理與 lifecycle 行為集中。

### 2. guard 是全域性的

`MiddleVacuumService` 與 `SlotVacuumService` 都會在初始化時註冊寫入 guard。  
因此任何地方只要用同一個 `PlcService` 寫 PLC，都會被這些規則攔截。

這是本資料夾最重要的設計特徵之一。

### 3. `LiftService` 與 `SlotVacuumService` 共享左右真空點

`LiftService.set_vacuum()` 可直接控制左右真空；  
但如果某側在 `SlotVacuumService` 被標記為 `vacuum_required=True`，那麼：

- 關閉真空會被 guard 阻擋
- 啟動破真空也會被 guard 阻擋

所以左右真空的最終控制權其實不完全在 `LiftService`，而是受 `SlotVacuumService` 的狀態約束。

### 4. `YAxesService` 與 `PlcService` 的禁止位址策略互補

`YAxesService` 只使用允許的 command point；  
`PlcService` 額外禁止直接打開 `M389` / `M399`。  
這形成雙層防護。

## 建議的理解順序

如果要追 service 層實際執行流程，建議照這個順序看：

1. `config/services.yml`
   - 先知道每個 service 綁哪些 point id
2. `plc_service.py`
   - 先理解所有讀寫行為與 guard 掛載點
3. `middle_vacuum_service.py`
   - 理解互鎖 guard 模式
4. `slot_vacuum_service.py`
   - 理解載貨狀態、釋放確認、watchdog
5. `home_service.py` / `lift_service.py` / `y_axes_service.py`
   - 這三個相對單純，主要是把 PLC 點位組合成設備動作
6. `api/main.py`
   - 看實際是在哪裡建立 singleton 並暴露成 API

## 風險與設計評估

### 優點

- service 職責切分清楚
- 點位位址集中在 config，不散落於業務邏輯
- lifecycle 可直接對外提供運行狀態
- 寫入 guard 採集中式設計，安全性比單純靠呼叫慣例高
- `SlotVacuumService` 有持久化與 watchdog，對載貨安全很有價值

### 需要注意的點

#### 1. 初始化順序會影響 guard 是否生效

只有在 `MiddleVacuumService` / `SlotVacuumService` 物件被建立後，guard 才會註冊到 `PlcService`。  
如果測試或其他入口只建立 `PlcService`，沒有建立這兩個 service，安全規則就不完整。

#### 2. guard 帶有隱性全域副作用

因為 guard 註冊在共享的 `PlcService` 上，呼叫端從方法簽名看不出來「這次寫入可能被其他 service 狀態阻擋」。  
這提高安全性，但也提高理解成本。

#### 3. watchdog 吞例外

`SlotVacuumService._watchdog_loop()` 發生錯誤時直接 `continue`，不會升級告警。  
這樣系統不會停，但如果 PLC 持續異常，背景保護可能默默失效，只能靠 lifecycle 或外部監控發現。

#### 4. `LiftService.start_up()` 與 `start_vision_positioning()` 控制點相同

目前從 service 實作看不到差異。  
如果這兩個概念在業務上真的不同，未來可能需要更清楚的命名或不同控制路徑。

#### 5. 寫入失敗的回滾策略不一致

- `HomeService` 有部分成功後回滾
- `LiftService` / `YAxesService` 用 `stop()` 作補救
- `PlcService` 原始寫入不重送

整體是合理的，但表示每個 service 的失敗語意不同，API 或 flow 層需要明確知道這些差異。

## 總結

這個資料夾的本質不是單純的 PLC helper，而是：

1. `PlcService` 提供統一 PLC 存取介面
2. 其餘 service 把點位組成具體設備動作
3. `MiddleVacuumService` 與 `SlotVacuumService` 再把安全互鎖下沉到共享寫入層
4. `SlotVacuumService` 額外透過狀態持久化與 watchdog 持續保護「有貨必須保持真空」

如果你要判斷 service 層的實際控制中心，答案是兩個：

- 功能中心：`PlcService`
- 安全中心：`SlotVacuumService` 與 `MiddleVacuumService`
