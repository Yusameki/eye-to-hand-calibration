<div align="center">

# 🔧 eye-to-hand-calibration

**HALCON でやった一度のハンドアイキャリブレーション（eye-to-hand）**

<sub>[English](README.md) · **日本語** · [中文](README.zh-CN.md)</sub>

<sub>Photoneo PhoXi 3D スキャナ + ロボットアーム + キャリブレーション球</sub>
<sub>HALCON 21.05 · 2022 年の夏の実験記録</sub>

</div>

---

## これは何か

**固定設置**の 3D スキャナ（eye-to-hand）から見て、ロボットアームの先端がどこにあるのかを知りたい。
やり方は、ロボットにキャリブレーション球を持たせて視野の中で 20 通りの姿勢を取らせ、
各姿勢で「アーム先端の姿勢」+「点群の中での球の姿勢」を一組ずつ記録し、
それを HALCON に渡してカメラとロボットベースの間の変換を解く、というもの。

> ( ¯꒳¯ ) 要するに：**ロボットとカメラに仲良くなってもらう**話です。

<div align="center">

![取得の様子](pic/01.png)

<sub>▲ カメラの視野の中：ロボットアーム + キャリブレーション球 + キャリブレーションボード</sub>

</div>

---

## 目次

- [リポジトリの中身](#リポジトリの中身)
- [流れ](#流れ)
  - [1. 取得](#1-取得)
  - [2. モデリング](#2-モデリング)
  - [3. キャリブレーション](#3-キャリブレーション)
  - [4. 検証](#4-検証)
- [主要パラメータ](#主要パラメータ)
- [環境](#環境)
- [つまずいたところ](#つまずいたところ)

---

## リポジトリの中身

| ファイル / ディレクトリ | 役割 |
| --- | --- |
| `get_calibration_data.hdev` | PhoXi に接続し、点群を取得し、点群をボードのグレー画像に変換 |
| `perform_calibration.hdev` | メインのキャリブレーション：球モデルを作り、20 サンプルを回して hand-eye を解く |
| `evaluation.hdev` | キャリブレーション後の検証：平面フィッティング + 内円検出 + 中心誤差の計算 |
| `ball_test.hdev` | 球バージョンの同じ流れ。キャリブレーション対象を変えるときに使う |
| `create_pose.hdev` | 姿勢を手書きするだけのスクリプト。数値の確認用 |
| `Repose/Diff_Pose_*.dat` | 20 組のキャリブレーション姿勢（回転ベクトル + 並進、単位 m） |
| `Repose/Re_Cam2Obj_*.dat` | 20 組のカメラ→物体の姿勢 |
| `pointcloud/3d_CalBoarde_*.om3` | 20 個のボード点群（HALCON OM3 形式） |
| `BoardSurfaceModel.stl` | ボードの 3D モデル。surface-based matching に使う |
| `pic/` | 途中のスクリーンショット。一部を下に貼っています |
| `PhotoneoLogFiles/` | PhoXi の動作ログ。接続トラブルの切り分け用 |

---

## 流れ

### 1. 取得

`get_calibration_data.hdev` がスキャナと会話します。HALCON は `PhoXi` という
frame grabber インターフェース経由で Photoneo に繋ぎ、デバイスはシリアルかインデックスで選びます：

```halcon
AcqName := 'PhoXi'
info_framegrabber (AcqName, 'device', DeviceInfo, DeviceValues)
open_framegrabber (AcqName, 1, 1, 0, 0, 0, 0, 'default', -1, 'default', -1, \
                   'false', 'default', DeviceValues[0], -1, -1, AcqHandle)
set_framegrabber_param (AcqHandle, 'use_grab_image_async_as_trigger_only', 'enable')
grab_image_async (TextureImage, AcqHandle, 0)
```

いくつか要点：

- **PhoXiControl が先に起動している必要があります**。そうでないと `open_framegrabber` が即失敗します
- 出力構造は PhoXiControl 側で設定しておきます（`OutputStructure` → 必要なフィールドを選ぶ → `Set`）。
  既定では `PointCloud` と `Texture` が enabled です
- 取得した点群の単位は **mm**。あとで **m** に直してからキャリブレーションに渡します
- `Texture` 画像のコントラストは手で調整が必要です。でないとボード上の DataMatrix コードが読めません

<div align="center">

![取得](pic/05.png)
![取得](pic/10.png)

<sub>▲ アームが球を抱えて姿勢を変える。1 フレームが 1 サンプルに対応</sub>

</div>

### 2. モデリング

ボードを点群の中から見つけるために、まず 3D モデルが必要です。そうして surface-based matching が使えます。

```halcon
read_object_model_3d ('BoardSurfaceModel', 'm', [], [], BoardOM3D, Status)
create_surface_model (BoardOM3D, 0.02, [], [], BoardSurfaceModelID)
```

球のほうがもっと簡単で、解析的な球をそのまま生成します（半径 **27.7 mm**）：

```halcon
gen_sphere_object_model_3d ([0,0,0,0,0,0,0], SphereRadius, SphereOM3D)
convex_hull_object_model_3d (SphereOM3D, SphereOM3D)
create_surface_model (SphereOM3D, 0.02, 'model_invert_normals', 'true', SphereSurfaceModelID)
```

この `0.02` は相対サンプリング距離です。小さくするとマッチングが遅く、大きくすると不安定になります。

<div align="center">

![取得](pic/15.png)
![取得](pic/20.png)

<sub>▲ 20 姿勢でワークスペースのほぼ全体をカバー</sub>

</div>

### 3. キャリブレーション

核心はこの 2 行だけ：

```halcon
create_calib_data ('hand_eye_stationary_cam', 0, 0, HECCalibDataID)
set_calib_data (HECCalibDataID, 'model', 'general', 'optimization_method', 'nonlinear')
```

- `hand_eye_stationary_cam` = **カメラは固定**（eye-to-hand）。
  カメラをアームに載せる場合は `hand_eye_moving_cam` にする必要があります
- `optimization_method` は `nonlinear` を選択。姿勢精度が一番良い代わりに少し遅いです

そのあと 20 サンプルをループ：

```halcon
NumCalibrationScenes := 20
for I := 1 to NumCalibrationScenes by 1
    read_object_model_3d (filename, 'm', [], [], ObjectModel3D, Status)
    object_model_3d_to_xyz (X, Y, Z, ObjectModel3D, 'from_xyz_map', [], [])
    get_object_model_3d_params (ObjectModel3D, '&intensity', intensity)
    set_grayval (Image, CalBoard_Rows, CalBoard_Columns, intensity)
    ...
endfor
```

ここに小さな工夫があります：**点群の強度値をグレー画像に書き込み**、行列出は点群の x/y を対応させます。
`find_data_code_2d` は byte 型の画像しか受け付けず、3D 点群は直接渡せないため、
回り道をして点群でボードの「写真」を撮り、そこからコードを読むわけです。

> 🍮 サンプルによってはマッチングが失敗します。スクリプト側で判定していて、
> **マッチしなかったサンプルはキャリブレーションに参加させず飛ばします**。
> 無理に突っ込むと結果が歪みます。

<div align="center">

![取得](pic/27.png)

</div>

### 4. 検証

`evaluation.hdev` は別ルートで、キャリブレーション球を使わずボードに対して幾何的な検証をします。

1. ROI を手で描いて平面部分を切り出す
   ```halcon
   reduce_object_model_3d_by_view (PlaneROI, ObjectModel3D, CamParam, Pose, ObjectModel3D_Plane)
   fit_primitives_object_model_3d (ObjectModel3D_Plane, 'primitive_type', 'plane', OM3D_Plane)
   get_object_model_3d_params (OM3D_Plane, 'primitive_pose', Cam_H_Plane)
   ```
2. 平面の姿勢で点群を正しい向きに直し、`pose_invert` してから `rigid_trans_object_model_3d`
3. 正した画像で内円の輪郭を見つけ、`fit_circle_contour_xld` で中心をフィット
4. 中心の実座標と、キャリブレーションから計算した座標の差が誤差

```halcon
edges_sub_pix (ImageReduced, Circle0_Edges, 'canny', 2, 20, 60)
select_shape_xld (Circle0_Edges, Circle0_Edges_select, ['contlength','circularity'], 'and', [50,0.6], [500,1])
fit_circle_contour_xld (Circle0_Outer, 'ahuber', -1, 2, 0, 3, 2, \
                        Circle0_Row, Circle0_Column, Circle0_Radius, ...)
```

`'ahuber'` は Huber 重み付けで、輪郭上の外れ値に寛容です。実測では既定の最小二乗より安定しました。

---

## 主要パラメータ

| パラメータ | 値 | 説明 |
| --- | --- | --- |
| `NumCalibrationScenes` | 20 | キャリブレーション姿勢の数 |
| `SphereRadius` | 27.7 | キャリブレーション球の半径（mm） |
| 最適化手法 | `nonlinear` | 姿勢が一番正確 |
| キャリブレーション種別 | `hand_eye_stationary_cam` | カメラ固定 |
| カメラ内部パラメータ | `gen_cam_par_area_scan_division` | 2064×1544、焦点距離 0.008 / 画素サイズ 5.2e-06 |

内部パラメータはスクリプトによって少しずつ違っていて、撮影したバッチが違うためです：

```halcon
* 歪みありのバージョン
gen_cam_par_area_scan_division (0.0078066255, -0.121916, 0.00000345, 0.00000345, \
                                1071.03, 793.9, 2064, 1544, CamParam)
* 歪みなしのバージョン
gen_cam_par_area_scan_division (0.008, 0, 5.2e-06, 5.2e-06, \
                                1071.3, 793.9, 2064, 1544, CamParam)
```

姿勢ファイルはこうなっています（`Repose/Diff_Pose_01.dat`）：

```
# 3D POSE PARAMETERS: rotation and translation
f 0
r -0.198842898250703 4.0770661949706 7.42384111526009
t 1.49453245821141 1.25246473791518 -0.428090294941512
```

`r` は回転ベクトル（Rodrigues、単位 deg）、`t` は並進（単位 **m**）。
HALCON の `write_pose` が出力する形式そのままで、`read_pose` で直接読めます。

---

## 環境

| 項目 | バージョン / 型番 |
| --- | --- |
| HALCON | 21.05.0.0 Progress |
| 3D スキャナ | Photoneo PhoXi（HALCON 標準の `PhoXi` インターフェース） |
| ロボットアーム | 先端姿勢が読める 6 軸なら何でも |
| キャリブレーションボード | DataMatrix コード付きの平面ボード + 27.7mm の球 |

スクリプトは `.hdev`（HDevelop が保存する XML 形式）なので、HDevelop でそのまま開いて動かせます。

---

## つまずいたところ

- **PhoXiControl が起動していない** → `open_framegrabber` がエラー。まずソフトが動いていてデバイスが繋がっているか確認
- **点群の単位を混同** → HALCON は mm、キャリブレーションは m。1000 倍ずれると結果がおかしいとすら気づけないレベルになります
- **`find_data_code_2d` は byte 画像しか食べない** → `object_model_3d_to_xyz` + `set_grayval` で一度変換が必要
- **サンプルの質が揃わない** → マッチングに失敗したサンプルは必ず除外。数が減っても汚染するよりまし
- **テクスチャ画像のコントラスト** → 低すぎると DataMatrix コードが読めないので手で調整

---

<div align="center">

<sub>2022 年にやったことの記録です (´-ω-`)</sub>

</div>
