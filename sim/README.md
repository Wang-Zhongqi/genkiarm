# MuJoCo 场景

在项目根目录运行：

```powershell
.venv\Scripts\python.exe -m mujoco.viewer --mjcf=sim/play_chess.xml
.venv\Scripts\python.exe -m mujoco.viewer --mjcf=sim/arm/preview.xml
.venv\Scripts\python.exe -m mujoco.viewer --mjcf=sim/chess/preview.xml
```

`play_chess.xml` 组合机械臂、棋盘和 32 颗可运动棋子，包含地面、办公室照明近似及全局相机。场景本身不包含自动下棋逻辑。

## 资源组织

| 路径 | 内容 |
| --- | --- |
| `arm/assets/arm.xml` | 机械臂网格引用、默认类、运动学树、腕部相机、接触排除和执行器 |
| `arm/meshes/` | 机械臂 STL 网格 |
| `arm/preview.xml` | 机械臂独立预览 |
| `chess/assets/board.xml` | 棋盘网格、贴图、材质和几何默认类 |
| `chess/assets/pieces.xml` | 棋子贴图、材质和原生圆柱默认类 |
| `chess/assets/grid_map.xml` | 棋盘本体和 90 个交点 |
| `chess/assets/layout.xml` | 32 颗棋子的开局实例 |
| `chess/preview.xml` | 棋盘和棋子的独立预览 |

机械臂资源的旋转使用四元数，不依赖父场景的角度单位。组合场景通过 include 引用公共资源。

## 场景布局

在 `play_chess.xml` 的 `chess_placement` frame 中统一调整棋盘、交点和开局棋子的位置及朝向。当前棋盘中心为 `[-0.20, 0, 0.0002]` 米，绕 Z 轴旋转 90 度，红方朝向机械臂。地面位于 Z=0，棋盘上移 0.2 mm，避免底面与地面共面产生条纹。机械臂基座位于原点。

组合场景配置 `compiler.meshdir="arm/meshes"` 和 `compiler.texturedir="chess/textures"`。迁入其他场景时，按入口 XML 所在目录调整资源路径。

## 遥操作和双视角

```powershell
.venv\Scripts\python.exe sim/real_teleop_sim.py --port COM3
```

默认加载 `sim/play_chess.xml`，可通过 `--mjcf` 指定其他场景。主窗口为可交互的全局视角；独立的 `Wrist camera - OV2735` 窗口显示 640 × 360 腕部相机画面，随腕部移动。在腕部窗口按 Q、Esc，或关闭任一窗口，可结束遥操作。

主手编码器角度写入六个仿真执行器的目标值，通过物理步进驱动机械臂，不直接覆盖关节位置。碰撞时实际姿态可能与目标角度不同，棋子由接触力驱动。步进按帧间隔累计，单帧最多追赶 0.1 秒。

以下命令仅检查模型和校准文件，不打开串口或预览窗口：

```powershell
.venv\Scripts\python.exe sim/real_teleop_sim.py --check
```

## 碰撞和夹持

碰撞类别为机械臂 1、棋盘 2、棋子 4、地面 8。机械臂可接触棋盘、棋子和地面，保留非相邻连杆的自碰撞；相邻部件保留接触排除。棋子之间及棋子与棋盘、地面之间可碰撞。棋盘视觉网格、相机支架、摄像头和 USB 插头不参与碰撞。

夹爪采用分段凸包和独立盒状指腹，避免整块凸包将棋子挤出。碰撞代理位于默认隐藏的 group 3，质量为零；原视觉网格保留质量和惯量。需要检查碰撞形状时，可临时开启 group 3，此时会与视觉表面重叠。

`arm/assets/arm.xml` 中的 `grip_contact` 为指腹配置 `condim=6`，摩擦参数为 `2 .004 .0002`，接触时间常数为 `.004`。它不增加棋盘表面的摩擦。夹爪执行器 servo6 使用 kp=1、ki=0、kv=0.05，输出力矩限制为 ±0.15 Nm。

组合场景和机械臂预览使用 Newton 求解器、椭圆摩擦锥及 impratio=50。这些参数用于仿真，未经实机摩擦和力矩标定。

## 办公室照明近似

组合场景使用四个固定方向的中性顶光和一个墙面补光方向，提供均匀底光和较弱的明暗变化。灯光不随观察相机移动，相机头灯关闭，场景灯不投射硬阴影。此配置近似办公室漫射照明，不是 HDRI 或全局光照求解。

调整照明后应检查全局、棋盘近景及腕部画面，确认模型可见、格线清楚且没有大面积过曝。
