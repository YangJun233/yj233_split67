#!/usr/bin/env bash
# 用法: ./flash_wsl.sh left|right
#
# WSL2 专用。`qmk flash -bl uf2-split-*` 在 WSL 下会卡在 "Waiting for drive to
# deploy..."：util/uf2conv.py 的 get_drives() 只扫 /media 和 /run/media/$USER，
# 而 RPI-RP2 这个 U 盘是 Windows 挂的，WSL 侧看不到（且 WSL2 不自动挂载热插拔盘）。
# 这里改成编译 + 通过 powershell.exe 复制到 Windows 盘符，不需要 sudo mount。
set -euo pipefail

case "${1:-}" in
    left)  MACRO=-DINIT_EE_HANDS_LEFT  ;;
    right) MACRO=-DINIT_EE_HANDS_RIGHT ;;
    *) echo "用法: $0 left|right" >&2; exit 1 ;;
esac
SIDE=$1

cd "$(dirname "$0")/../.."
qmk compile -kb yj233_split67 -km vial -e "EXTRAFLAGS=$MACRO"

UF2=$(wslpath -w "$PWD/yj233_split67_vial.uf2")

echo
echo ">>> 请把【$SIDE 半】插上 USB 并双击 RESET 进 bootloader ..."
DRIVE=""
for _ in $(seq 1 120); do
    DRIVE=$(powershell.exe -NoProfile -Command \
        "Get-Volume | Where-Object {\$_.FileSystemLabel -eq 'RPI-RP2'} | Select-Object -First 1 -ExpandProperty DriveLetter" \
        2>/dev/null | tr -d '\r\n ')
    [ -n "$DRIVE" ] && break
    sleep 1
done

if [ -z "$DRIVE" ]; then
    echo "超时：没找到 RPI-RP2 盘。确认双击 RESET 的间隔够快，且 Windows 里能看到该盘。" >&2
    exit 1
fi

echo ">>> 找到 ${DRIVE}: — 正在写入 $SIDE 半固件"
powershell.exe -NoProfile -Command \
    "Copy-Item -LiteralPath '$UF2' -Destination '${DRIVE}:\\NEW.UF2'" 2>/dev/null || true

echo ">>> 完成。板子会自动重启（复制末尾报「设备已断开」是正常的）。"
