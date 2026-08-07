# python-plc

三菱 PLC 讀寫範例專案（Python，MC Protocol + FastAPI + Tkinter）

目標：
- 讀取指定三菱 PLC 的暫存器（D/M/X/Y…）
- 透過 Backend API 提供服務
- 提供簡單桌面 UI 顯示/操作點位
- 日後可擴充更多 PLC、更多點位與權限

專案結構：

```bash
python-plc/
├── README.md              # 本說明檔
├── VERSION                # 目前版本號
├── pyproject.toml         # uv / Python 專案設定
├── requirements.txt       # 依賴套件清單
├── main.py                # 一鍵啟動：先起 API 再起 UI
├── api/
│   └── main.py            # FastAPI backend：PLC 讀/寫 API + logging
├── plc/
│   └── client.py          # MitsubishiPLCClient（MC Protocol, pymcprotocol）
├── config/
│   ├── loader.py          # 載入/查詢配置的工具
│   ├── plc_config.yml     # PLC 連線設定
│   ├── points.yml         # 點位設定
│   ├── services.yml       # Service 使用的 point ID 映射
│   └── flows/             # Flow 目標、等待、逾時等參數
├── service/
│   ├── plc_service.py     # 依 point ID 讀寫 PLC
│   ├── home_service.py    # 回原點設備操作
│   └── lift_service.py    # 升降與真空設備操作
├── flow/
│   ├── home_flow.py       # 一鍵回原點流程
│   └── vision_height_flow.py # 視覺高度流程
└── ui/
    └── main.py            # Tkinter UI（group + points 表格，支援讀寫）
```

### Service / Flow 分層

新功能固定依照以下方向呼叫：

```text
UI / API -> Flow -> Service -> plc/client.py -> PLC
```

- `points.yml` 是 D/M 位址的唯一來源。
- `services.yml` 只把設備能力對應到 point ID，不保存流程時間。
- `config/flows/*.yml` 只保存目標、容許誤差、等待與逾時。
- Service 負責單一設備操作並回報 `Service.method + point_id`。
- Flow 負責步驟順序、重試、取消、逾時與清理，不直接使用 D/M 位址。
- PLC 寫入若遇到通訊中斷，不會自動重送，避免設備收到重複動作。

---

## 一、環境準備

在你解壓縮的 `python-plc` 目錄底下執行：

```bash
cd python-plc

# 使用 uv 建立環境並安裝依賴（建議）
uv sync

# 或使用傳統 pip 流程
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt

# requirements.txt 內容：
# fastapi
# uvicorn
# pymcprotocol
# PyYAML
# requests
```

---

## 二、一鍵啟動（建議使用）

在 `python-plc` 目錄下執行：

```bash
uv run python main.py
```

若已啟動 venv，也可執行 `python main.py`。

行為說明：

- 會先讀取 `VERSION` 檔案並印出版本號，例如：
  ```
  python-plc version: 0.2.3
  ```
- 在背景 thread 啟動 FastAPI（uvicorn，預設 `http://0.0.0.0:8000`）
- 稍等約 1 秒後，在主執行緒啟動 Tkinter UI

若你想分開啟動（例如除錯），可以參考下一節。

---

## 三、啟動 Backend（FastAPI，手動方式）

在 `python-plc` 目錄下執行：

```bash
uv run uvicorn api.main:app --reload --host 0.0.0.0 --port 8000
```

啟動後可以用瀏覽器開：

- 健康檢查：<http://127.0.0.1:8000/health>
- Swagger UI：<http://127.0.0.1:8000/docs>

目前 API 主要包含：

- `GET /health`：健康檢查
- `POST /plc/connect?plc=main_plc`：建立與指定 PLC 的連線
- `POST /plc/disconnect?plc=main_plc`：關閉與指定 PLC 的連線
- `GET /points?group=...`：列出點位定義（可依 group 過濾）
- `GET /points/{id}`：查單一點位定義
- `GET /plc/registers`：直接以 `device + start + count` 讀暫存器
- `POST /plc/registers/write`：以 `device + start + values[]` 寫暫存器
- `GET /plc/registers/by-points?ids=...&ids=...`：依多個 `point_id` 讀取點位數值
- `POST /plc/registers/write-by-point`：以 `point_id` 寫入單一點位
- `POST /flows/home/run`：執行一鍵回原點 Flow
- `POST /flows/home/cancel`：取消一鍵回原點 Flow
- `POST /flows/vision-height/run`：執行視覺高度 Flow
- `POST /flows/vision-height/cancel`：取消視覺高度 Flow

後端特性：

- 透過 `config/loader.py` 讀取 `plc_config.yml`、`points.yml`
- 使用 `plc/client.py` 對接 `pymcprotocol`，支援 FX5U MC Protocol TCP
- 實作 logging：
  - 所有讀寫、連線/斷線 API 都會記錄到終端機與 `logs/plc_backend.log`
  - 遇到 `[WinError 10053]` 會自動關閉 PLC 連線，讓下一次呼叫重新連線

---

## 四、啟動桌面 UI（Tkinter，手動方式）

另外開一個終端機（同樣進 `python-plc`、啟動 venv）：

```bash
uv run python -m ui.main
```

或透過 `uv run python main.py` 一次啟動 API + UI。

UI 功能：

- 上方：
  - 連線按鈕：呼叫 `POST /plc/connect?plc=main_plc`
  - 斷線按鈕：呼叫 `POST /plc/disconnect?plc=main_plc`
  - 狀態列：顯示「未連線」或「已連線 (main_plc)」
  - 三步驟總流程：
    - 第一步與第三步可分別設定貨盤、吸/推動作、D500 高度與
      D510/D560 前進距離。
    - 第二步會自動執行 D435i 辨識、CANBus 手臂取放、升降機與
      M54/M55 真空交接。
    - 「執行完整流程」會依序執行第一步、第二步、第三步；只有前一段
      成功才會進入下一段。
    - 若流程停在第二步或第三步，按鈕會改成從該階段續跑，避免重做已
      完成的設備動作。
    - 第二步執行時可開啟進度視窗，查看目前手臂、相機、升降機與真空
      交接狀態。
  - 「確認手臂／Camera HOME」只讀取 ID142～ID145，不送出移動命令：
    - 四軸皆在 HOME 容許誤差內，且連續穩定回讀三次後，狀態才會變成
      `HOME_CONFIRMED`，並將升降高度上限由 695mm 恢復為 1450mm。
    - 任一軸未回覆、角度超差或確認逾時時，狀態維持 `unknown`，不解除
      695mm 安全限制。
    - 執行完整三步驟流程時，UI 也會在第一步之前自動進行一次相同確認。
    - 第一步與第三步只有在手臂及 Camera 的四軸 HOME 回讀確認有效時，
      才能使用 1450mm 上限；未確認時上限為 695mm。
    - 第二步開始時會立即撤銷先前的 HOME 確認，且第二步所有高度來源
      （設定、手臂與 Vision Bridge）均以程式硬限制在 695mm。
    - 第二步結束後，只有再次收到四軸最終 HOME 回讀確認，第三步才會
      恢復 1450mm 上限；失敗、取消或舊式 Bridge 流程皆維持 695mm。
- 左側：
  - 根據 `points.yml` 顯示所有 `group` 名稱
- 右側：
  - 顯示該 group 底下的點位清單
    - 欄位：ID / 名稱 / 裝置 / 位址 / 數值 / 可寫入
    - 數值：連線後會透過 `GET /plc/registers/by-points` 從 PLC 讀取
  - double-click 「數值」欄可編輯 **且該點位 `writable: true` 時**：
    - D 類等數值型點位 → 直接輸入新值
    - M/X/Y 等 bit 點位 → double-click 後直接在 ON/OFF 間切換
    - 確認輸入或切換時會呼叫 `POST /plc/registers/write-by-point` 寫回 PLC
    - 寫入失敗會跳出提示視窗，詳細錯誤可看後端 log

---

## 五、設定方式（重點檔案）

### 1. PLC 連線設定：`config/plc_config.yml`

把 IP/port 改成你現場的三菱 PLC（本專案預設 FX5U CPU 範例）：

```yaml
connections:
  - name: main_plc
    host: 192.168.3.250  # 改成你的 PLC IP
    port: 5000           # MC Protocol TCP port（依你現場設定）
    protocol: mc
    unit: 0              # station/unit
```

程式會透過 `config/loader.py` 的 `CONFIG_STORE` 載入：

```python
from config.loader import CONFIG_STORE

plc_def = CONFIG_STORE.get_plc("main_plc")
```

### 2. 點位設定：`config/points.yml`

照這個格式增加/修改點位（節錄部分）：

```yaml
- id: y1_up_speed
  name: "Y1上升速度"
  plc: main_plc
  device: D
  address: 200
  type: s16
  scale: 1.0
  writable: true
  group: "Y1參數"

- id: y1_up_pos
  name: "Y1上升位置"
  plc: main_plc
  device: D
  address: 500
  type: s32
  pair_high: 501
  scale: 0.1
  writable: true
  group: "Y1參數"
```

欄位說明：

- `id`：點位唯一識別，用在 UI 與 API `/plc/registers/write-by-point`
- `name`：UI 顯示名稱
- `plc`：對應到 `plc_config.yml` 的 `name`
- `device`：D/M/X/Y…
- `address`：起始位址（整數）
- `type`：
  - `int`/`s16`/`u16`：單一 16-bit word
  - `s32`/`u32`/`f32`：使用 `address` + `pair_high` 兩顆 D 組成 32-bit 或 float
- `scale`：可選，讀取後的工程值倍率（例如 0.1 表示內部 500 → 顯示 50.0）
- `pair_high`：當 type 為 32-bit/float 時，高位 D 的地址
- `group`：用來在 UI 左側 group 列表分組
- `writable`：是否允許透過 UI / API 寫入
- `min` / `max`：可選，用於寫入時範圍檢查（目前尚未強制）

程式同樣透過 `CONFIG_STORE` 載入：

```python
from config.loader import CONFIG_STORE

point = CONFIG_STORE.get_point("y1_up_speed")
points_in_group = CONFIG_STORE.list_points(group="Y1參數")
```

---

## 六、PLC 通訊層（MC Protocol, `pymcprotocol`）

實作在 `plc/client.py`，針對你目前使用的 `pymcprotocol` 版本，採用：

- 連線：

```python
from plc.client import MitsubishiPLCClient

client = MitsubishiPLCClient(host="192.168.3.250", port=5000, unit=0)
client.connect()
```

- 讀 D 暫存器（使用 `batchread_wordunits`，頭地址字串 + 長度）：

```python
values = client.read_d_register(300, count=1)  # 對應 D300
```

- 寫 D 暫存器（`batchwrite_wordunits`）：

```python
client.write_d_register(300, [100])  # 對應 D300 := 100
```

- 讀 bit 類裝置（M/X/Y，使用 `batchread_bitunits`）：

```python
bits = client.read_bit_device("M", 351, count=2)  # M351, M352
```

- 寫 bit 類裝置（`batchwrite_bitunits`）：

```python
client.write_bit_device("M", 351, [True, False])  # M351=1, M352=0
```

錯誤處理：

- 若尚未安裝 `pymcprotocol`：
  - 初始化時會丟出 `PLCConnectionError` 提示安裝指令
- 若遇到 `[WinError 10053] 連線已被您主機上的軟體中止。`：
  - client 會自動關閉連線（`self.close()`），讓上層在下次呼叫重新連線

---

## 七、Logging 與除錯

- log 檔案：`logs/plc_backend.log`
- 內容包含：
  - API 呼叫：`/plc/connect`、`/plc/disconnect`、`/plc/registers*`、`/points*`
  - 實際讀寫資訊（plc/device/address/value）
  - 連線失敗、讀寫失敗的錯誤訊息
- 同樣的訊息也會印在啟動 uvicorn 的終端機中，方便即時觀察。

特別處理：

- 遇到 `[WinError 10053]` 時：
  - client 會自動 close
  - `by-points` API 會記錄錯誤並關閉該 PLC client
  - 下次呼叫對應 PLC 會重新連線，適用於「人機 + 本程式」同時連線 FX5U 的情境

---

## 八、版本紀錄（Changelog）

> 註：以下僅列出從 v0.1.3 之後的重要變更；更舊版本為初始骨架。

- **v0.2.3**
  - UI：
    - M/X/Y 類 `bit` 點位在「數值」欄改為 checkbox 形式顯示（ON/OFF），
      勾選即代表 True/False，切換時自動呼叫 `write-by-point` 寫回 PLC
    - 其他數值型（D 等）仍採用文字輸入方式編輯

- **v0.2.2**
  - 後端：
    - `plc_config.yml` 預設 port 改為 `5000`，與你現場 FX5U 的 MC 通訊設定一致
    - `/plc/registers/by-points` 支援 `type + pair_high + scale` 的 D 解碼邏輯，
      可對不同 D 暫存器設定 `s16/u16/s32/u32/f32` 及工程值倍率
  - 設定：
    - `points.yml` 中部分點位（例如 Y1 上升速度、Y1 上升位置、手動速度）
      已示範如何使用 `type=s16/s32`、`pair_high` 與 `scale`

- **v0.2.0**
  - 文件：
    - 整理並補齊 README：加入版本紀錄（Changelog）、一鍵啟動說明、UI/Backend 串接行為與 logging 說明
    - 之後每次版本更新會同步更新此區塊

- **v0.1.9**
  - UI：
    - 支援在表格中直接 double-click「數值」欄，編輯可寫入 (`writable: true`) 的點位
    - 編輯完成時自動呼叫 `POST /plc/registers/write-by-point` 寫回 PLC
    - 寫入失敗會跳出提示，詳細原因可由後端 log 追蹤

- **v0.1.8**
  - 後端：
    - 對 `[WinError 10053]`（連線被主機中止）加入自動處理：
      - 在 `plc/client.py` 中遇到 10053 會自動關閉 PLC client
      - `/plc/registers/by-points` 遇到 10053 會關閉對應 PLC 連線，下次呼叫時重新連線
    - 改善在人機介面 + 本程式同時連線 FX5U 時的穩定性

- **v0.1.7**
  - `plc/client.py`：
    - 依你實際的 `pymcprotocol` 版本，改用 `batchread_wordunits(headdevice, size)`、
      `batchread_bitunits(headdevice, size)` 的呼叫方式（頭地址字串 + 長度）

- **v0.1.6**
  - `plc/client.py`：
    - 自 `readworddevices`/`readbitdevices` 改為 `batchread_*` / `batchwrite_*` 介面
    - 修正函式名稱與參數不相容的問題

- **v0.1.5**
  - `plc_config.yml`：
    - 預設 PLC IP 改為 `192.168.3.250`，對應現場 FX5U CPU

- **v0.1.4**
  - 加入 `VERSION` 檔案與版本控管流程
  - `main.py` 啟動時會印出目前版本號
  - FastAPI `app` 會讀取 `VERSION` 做為 `version` 顯示在 Swagger UI

- **v0.1.3**
  - 修正 `plc/client.py` 中型別註解、語法錯誤
  - 確保在 Python 3.10+ 環境下可以正常匯入模組

之後每次程式有功能調整，會同步：

1. 更新 `VERSION` 檔案中的版本號
2. 於本節補充該版本的變更內容（至少列出：日期/主要修改點）

---

此 README 已包含啟動方式、設定方式、UI 操作與版本變更記錄，
未來只要打開這個檔案，就可以快速了解專案狀態與更新歷史。
