# yj233_split67

自制 RP2040 分体键盘（左右各一半，共 67 键）。左右两半用**同一份固件**，通过 `EE_HANDS` 把「左/右」标志写进各自的 EEPROM 来区分。指点设备（TPS43 触摸板 / PS2 TrackPoint，开机自动识别）**左右两半都支持**，装哪边都行。

- MCU：RP2040（Pro Micro 引脚）
- 分体通信：全双工 PIO UART（两半均 GP4=TX / GP5=RX，**排线交叉**）
- 主机判定：`SPLIT_USB_DETECT`（哪半先枚举 USB，哪半就是 master）
- 键位：Vial（改键在 Vial GUI 里进行，无需重新编译）

## 串口接线（⚠ 别接错）

两半的固件都用 **GP4 发（TX）、GP5 收（RX）**，所以排线必须**交叉**：

```
左半 GP4 (TX) ──────╲╱────── 右半 GP5 (RX)
左半 GP5 (RX) ──────╱╲────── 右半 GP4 (TX)
GND ───────────────────────── GND
```

正因为是交叉线，master 和 slave 的收发引脚角色相同，`config.h` 里**不能**定义 `SERIAL_USART_PIN_SWAP`。在 RP2040 的 PIO 串口驱动里（`platforms/chibios/drivers/vendor/RP/RP2040/serial_vendor.c` 的 `serial_transport_driver_master_init`），这个宏**只把 master 一侧的 TX/RX 对调**，是给 `GP4↔GP4 / GP5↔GP5` 的**直连线**用的；交叉线上启用它会导致两半完全通信不上（插 USB 那半能打字，另一半整半无反应）。

如果哪天换成直连排线，就要反过来把 `SERIAL_USART_PIN_SWAP` 加回去。

## 编译

```bash
qmk compile -kb yj233_split67 -km vial
```

这条命令只用来检查能不能编过。**它编出来的固件不带左右标记，不要直接拿去烧**——见下面的烧录说明。

## 烧录（关键：左右各用不同 target）

因为启用了 `EE_HANDS`，必须用带 `uf2-split-left` / `uf2-split-right` 的 target 烧录。

这两个 target **不只是"烧录时写一个标志"**：`platforms/chibios/flash.mk` 会给编译加上 `-DINIT_EE_HANDS_LEFT` / `-DINIT_EE_HANDS_RIGHT`，所以**左右两半的固件二进制其实是不同的**，而且这份固件**每次开机都会把左/右标志重新写回 EEPROM**（`quantum/split_common/split_util.c` 的 `is_keyboard_left_impl`）。这一点很重要，见下面的「Bootmagic 警告」。

**进入 bootloader**：本板支持双击 `RESET` 进入 UF2 模式（会挂载成 U 盘 `RPI-RP2`）。

### 左半

```bash
# 左半插 USB，双击 RESET 进 bootloader
qmk flash -kb yj233_split67 -km vial -bl uf2-split-left
```

### 右半

```bash
# 右半插 USB，双击 RESET 进 bootloader
qmk flash -kb yj233_split67 -km vial -bl uf2-split-right
```

> ⚠️ 千万别把 `left` 的固件烧到右半（或反之）——会导致该半用错矩阵引脚，整半按键错乱。若烧反了，重新用正确的 target 烧一次即可。

**建议以后一直用这两条命令烧录**，不要用不带 `-bl` 的普通烧录。原因见下。

### ⚠️ Bootmagic 警告：开机按住 Esc 会清掉左右标志

`bootmagic` 是开的，触发键是矩阵 `[0,0]`（左半的 **Esc**）。开机时按住它会走 `eeconfig_disable()` → 重启 → `eeconfig_init()` → `nvm_eeconfig_erase()`，把整块模拟 EEPROM 擦掉，**其中就包含 offset 14 的 handedness 字节**。

后果：左半会读到 handedness = 0（右半），于是用错矩阵引脚，整半按键错乱。

**规避办法**：只要每次都用 `-bl uf2-split-left` / `-bl uf2-split-right` 烧录，固件里就带着 `INIT_EE_HANDS_*`，每次开机都会把标志重新写回去，**对 Bootmagic 免疫**。反之，如果烧的是不带 `-bl` 的普通固件，一旦触发 Bootmagic 就必须重新用 `-bl uf2-split-*` 烧一次才能恢复。

### 手动拖 UF2（不用 qmk flash）

`qmk compile` **没有 `-bl` 参数**（`-bl` 只有 `qmk flash` 有），而 `uf2-split-left` 这个 make target 本身就依赖 `flash`，所以没法用它「只编译不烧录」。要手工生成带左右标记的 uf2，直接把宏传进去：

```bash
qmk compile -kb yj233_split67 -km vial -e EXTRAFLAGS=-DINIT_EE_HANDS_LEFT    # 生成后拖给左半
qmk compile -kb yj233_split67 -km vial -e EXTRAFLAGS=-DINIT_EE_HANDS_RIGHT   # 重新生成后拖给右半
```

编译输出里出现 `#pragma message: Faking EE_HANDS for left hand` 就说明宏生效了。

两次会覆盖同名 `.uf2`，请「编一个、拖一个」，别搞混。

## 验证

1. 左右都烧好、串口线接好后，插任意一半到电脑。
2. 左半按键出左边字符、右半出右边字符即为正常。
3. 用 **Vial → Matrix tester** 可逐键复核矩阵映射。
4. 若某半整体错位或无反应，多半是该半的 `split-left/right` 烧反或没烧到。
5. 若**插 USB 那半正常、另一半整半没反应**，先查串口排线是不是接成了直连（见「串口接线」一节）。
6. 若**按一个键、同一列的键跟着一起触发**（字符是对的，只是多出幽灵键），是 `MATRIX_UNSELECT_DRIVE_HIGH` 没生效——见「矩阵扫描」一节。

## ⚠ Vial 改键只存在「当时插 USB 的那半」

`SPLIT_USB_DETECT` 让两半都能当主机，但 QMK **不会跨分体同步 EEPROM**。以下数据都只写在**当时作为 master 的那一半**的 flash 模拟 EEPROM 里，另一半有完全独立的一套：

- Vial / VIA 的动态键位、宏、combo、tap dance
- QMK Settings
- RGB 的当前模式 / 颜色 / 亮度（开机默认值）

也就是说：**左半插 USB 时在 Vial 里改的键位，换成右半插 USB 后不会生效**。

实际使用建议二选一：

- **固定用同一半插 USB**（推荐，省心）；
- 或者改完键位后，把另一半也插上电脑用 Vial 改一遍（两边设置各自保存）。

同理，`Bootmagic`（按住某键上电）清 EEPROM 也只清当前这一半的。

## 硬件差异说明

左右两半是**两块不同的 PCB**，矩阵引脚不同（见 `info.json`）：

- 顶层 `matrix_pins` = 左半（同时是默认值），和非分体板 `yj233_tp` 的引脚完全一致
- `split.matrix_pins.right` = 右半

串口（GP4/GP5）、RGB（GP10 数据 / GP11 使能）、指点设备（GP0/GP1/GP2）这几组引脚**编号**左右是一样的，但**不要因此假设两半的外围电路等价**——已经踩过两次坑：

1. 右半的 RGB 使能曾经因为焊盘虚焊而整半不亮（左半正常），排查时一度误判为固件的分体同步问题；
2. 右半的前两条行线 GP27/GP26 是 MCU 的 ADC 焊盘，上面挂了额外的模拟外围，导致矩阵扫描出错——见下一节。

改动涉及右半外围时，先对着**右半自己的原理图**确认，别拿左半的结论外推。

## ⚠ 矩阵扫描：未选中的行必须主动驱动高

`config.h` 里的这一行**不能删**：

```c
#define MATRIX_UNSELECT_DRIVE_HIGH
```

### 为什么

QMK 的 COL2ROW 扫描每行做三件事（`quantum/matrix.c` 的 `matrix_read_cols_on_row()`）：选中行 → 逐列读 → 取消选中。默认的取消选中走 `gpio_atomic_set_pin_input_high()`，也就是**输入 + RP2040 内部上拉，约 50–80kΩ**。这里有个巨大的不对称：**拉低是推挽强驱动（~50Ω），回高只靠一个几十 kΩ 的弱上拉**。

右半的前两条行线是 **GP27 / GP26 = MCU 的 ADC1 / ADC0**，PCB 在这两个焊盘上挂了额外的模拟外围。MCU 一松手，这条行线就被按在低位回不来，于是：

- 选中该行时强拉低 → 目标交点读得没错，**打出的字符是对的**；
- 该行"取消选中"后依然是低 → 按住的键让它那条列线在**每个**行窗口都读到低；
- 结果 `current_matrix[r]` 每一行都置上这一位 → **按一个键，整列跟着触发**。

只有 GP27/GP26 两行有这个毛病，普通数字脚 GP22/GP20/GP23 正常，左半也正常。

开了这个宏之后，未选中的行被主动驱动到 3.3V：驱动阻抗 ~50Ω，下拉/滤波电容都打不赢；按住的键把列线接到一条 **3.3V** 的行线上，二极管两端等电位不导通，列线读到高（未按下）。

### ⚠️ 这个宏依赖「每颗轴都有二极管」

本板全键都有二极管，所以安全。**没有二极管绝对不能开**：同列不同行同时按下两个键时，一条被驱动到高的行和一条被驱动到低的行会通过这两颗开关直连，形成 GPIO 对 GPIO 短路。以后改板子时这条约束是承重的，别把二极管省掉。

### 经验

**RP2040 的 GP26–GP29 是 ADC 焊盘**，设计上很容易被挂上分压、滤波电容或电池采样，是最不该拿来做矩阵行线的四个脚。真要用，就必须配 `MATRIX_UNSELECT_DRIVE_HIGH`。

排查同类问题时记住一点：`info.json` 的行列表写错**只能让一个键报成另一个单独的键**，凑不出"一次闭合出现在多行"。看到一键触发多键，直接跳过定义去查电气。

## 指点设备（左右等价）

用的是 `POINTING_DEVICE_COMBINED`：**两半开机都会跑一遍自动探测**（先 I2C ping TPS43，没应答再切到 PIO PS/2 找 TrackPoint），检测到模块的那半产生报告，另一半返回全零，最后由 master 合并（`pointing_device_combine_reports`）后发给电脑。所以：

- 模块装左半、装右半，甚至两半各装一个，都能用；
- 和「哪半插 USB」无关，从机的报告会通过 split 事务转发给主机。

代价：**没装模块的那一半开机会多花约 1.6–2 秒**：TPS43 复位 110ms + I2C 探测 3 次失败 ~75ms + `PS2_MOUSE_INIT_DELAY` 死等 1000ms + PS/2 探测 3 次失败（每次 `ps2_host_send` / `ps2_host_recv_response` 各有 100ms 超时，见 `platforms/chibios/drivers/vendor/RP/RP2040/ps2_vendor.c`）450–750ms。

再叠加 `SPLIT_USB_TIMEOUT`（默认 2000ms，从机要等这么久才确认自己不是主机），所以**从机那半大约 4 秒后才完全就绪**。表现是：插上 USB 后主机那半立刻能打字，另一半过几秒才活过来。这不影响最终功能，介意的话可以调小 `POINTING_DETECT_RETRIES` 或 `SPLIT_USB_TIMEOUT`。

若某天把模块装在右半且安装方向和左半不同，用 `POINTING_DEVICE_ROTATION_*_RIGHT` / `POINTING_DEVICE_INVERT_*_RIGHT` 单独校正右半，不会影响左半。

## RGB（左右同步）

`info.json` 的 `rgb_matrix.split_count: [31, 36]` 会生成 `RGB_MATRIX_SPLIT`，于是：

- 两半各自跑 `rgb_matrix_task()`（不受 master/slave 限制），只渲染属于自己的 LED 区间（左 index 0–30，右 index 31–66 映射到本地 0–35）；
- master 通过 `PUT_RGB_MATRIX` 事务把整个 `rgb_matrix_config`（开关 / 模式 / 色相 / 饱和 / 亮度 / 速度）和休眠状态同步给 slave。

所以 Fn 层的 `RGB_TOG` / `RGB_VAI` 等在**任意一半**按下，左右两半都会同时变化；Vial 里改 RGB 同理。

### LED 的 x / y 是什么意思

`rgb_matrix.layout` 里每颗灯的 `x` / `y` 是它在**整块拼好的键盘**上的物理位置，用的是 QMK 固定的坐标空间：

- `x`：`0` = 整板最左边，`224` = 整板最右边
- `y`：`0` = 最上一排，`64` = 最下一排

**不是像素，也不是每半各自从 0 开始。** 所有带方向的动效（`cycle_left_right`、`static_gradient`、`cycle_up_down` 等）直接拿这两个数算颜色/相位，所以左右两半必须落在**不同的 x 区间**，否则每半各跑一遍完整渐变，看起来是左右镜像重复，而不是从左连续扫到右。

本板的换算（键位列号见 `layouts.LAYOUT`，整板 0–16 列）：

| | 键位列 | LED x |
|---|---|---|
| 左半 | 0 – 6 | 0 – 84 |
| 右半 | 9 – 16 | 126 – 224 |

即 `x = 列号 × 14`，`y = 矩阵行号 × 16`。中间 84–126 的空档就是两半之间的物理间隙。

`layout` 数组的**顺序**是 WS2812 的实际走线顺序（前 31 个是左半灯带、后 36 个是右半），改坐标可以，**不能重排顺序**，否则灯位全错。
