#!/usr/bin/env bash
set -euo pipefail

if (( $# > 1 )); then
    echo '用法: bash download_imu_tool.sh [保存目录，默认当前目录]' >&2
    exit 1
fi
if [[ $(uname -s) != Linux ]]; then
    echo '仅支持 Linux。' >&2
    exit 1
fi
arch=$(uname -m)
case "$arch" in
    x86_64|amd64) name=imu-tool; machine=3e00 ;;
    aarch64|arm64) name=imu-tool-arm; machine=b700 ;;
    *) echo "不支持的架构: $arch（仅支持 x86_64 / ARM64）" >&2; exit 1 ;;
esac
base_url=${IMU_DOWNLOAD_BASE_URL:-https://download.bxirobotics.cn/d/ELF3/bxi_imu_config}
url=${base_url%/}/$name
destination=${1:-.}
mkdir -p -- "$destination"
temporary=$(mktemp "$destination/.imu-tool.XXXXXX")
trap 'rm -f -- "$temporary"' EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

echo "下载 $arch: $url"
if command -v curl >/dev/null 2>&1; then
    curl -fL --connect-timeout 10 --max-time 120 --retry 2 -o "$temporary" "$url"
elif command -v wget >/dev/null 2>&1; then
    wget --timeout=30 --tries=3 -O "$temporary" "$url"
else
    echo '下载需要 curl 或 wget，任选其一。' >&2
    exit 1
fi
# Reject web pages and binaries for the wrong CPU before replacing a working copy.
header=$(od -An -v -tx1 -N64 -- "$temporary" | tr -d ' \n')
if [[ ${#header} != 128 || $header != 7f454c46020101* || ${header:36:4} != "$machine" ]]; then
    echo '下载文件不是当前架构的 ELF64 程序，已保留原文件。请检查下载站文件与直链。' >&2
    exit 1
fi
chmod 755 "$temporary"
mv -fT -- "$temporary" "$destination/imu-tool"
echo "已保存: $destination/imu-tool"
echo "运行: $destination/imu-tool"
