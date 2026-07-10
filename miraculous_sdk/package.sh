#!/bin/bash
#
# package.sh — 打包 SDK 为标准发布目录
#
# 使用方法:
#   ./package.sh [平台标签]
#
# 默认平台标签: 自动检测 (如 x86_64_linux_gnu, aarch64_linux_gnu)
# 示例:
#   ./package.sh                           -> miraculous_sdk_x86_64_linux_gnu_20260608/
#   ./package.sh x86_64_linux_gnu          -> 指定平台
#
set -euo pipefail

# --- 自动检测平台 ---
detect_arch() {
    local arch
    case "$(uname -m)" in
        x86_64)  arch="x86_64" ;;
        aarch64) arch="aarch64" ;;
        armv7l)  arch="armv7l" ;;
        *)       arch="$(uname -m)" ;;
    esac
    echo "${arch}_linux_gnu"
}

PLATFORM="${1:-$(detect_arch)}"
DATE_TAG="$(date +%Y%m%d)"
SDK_NAME="miraculous_sdk_${PLATFORM}_${DATE_TAG}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"
PKG_DIR="${SCRIPT_DIR}/${SDK_NAME}"

echo "=== Packaging ${SDK_NAME} ==="
echo "  Source:    ${SCRIPT_DIR}"
echo "  Build:     ${BUILD_DIR}"
echo "  Output:    ${PKG_DIR}"
echo ""

# --- 1. 编译 ---
mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"
cmake "${SCRIPT_DIR}" -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF
make -j$(nproc)

# --- 2. 安装到目标目录 ---
rm -rf "${PKG_DIR}"
DESTDIR="${PKG_DIR}" cmake --install "${BUILD_DIR}" --prefix "/"

# --- 3. 生成 Doxygen 文档 (输出到 build/doc, 避免污染源码目录) ---
DOXYGEN_OUT="${BUILD_DIR}/doxygen_doc"
if command -v doxygen &>/dev/null; then
    rm -rf "${DOXYGEN_OUT}"
    mkdir -p "${DOXYGEN_OUT}"
    cd "${SCRIPT_DIR}"
    doxygen Doxyfile 2>/dev/null || true
    # 如果 doxygen 输出到 doc/html (默认配置), 移到 build 目录
    if [ -d "${SCRIPT_DIR}/doc/html" ]; then
        mv "${SCRIPT_DIR}/doc/html" "${DOXYGEN_OUT}/"
        rmdir "${SCRIPT_DIR}/doc/html" 2>/dev/null || true
    fi
    if [ -d "${DOXYGEN_OUT}/html" ]; then
        mkdir -p "${PKG_DIR}/doc"
        cp -r "${DOXYGEN_OUT}/html" "${PKG_DIR}/doc/"
        echo "Documentation generated: doc/html/"
    fi
else
    echo "Warning: doxygen not found, skipping API docs"
fi

# --- 4. 验证 ---
echo ""
echo "=== Package created: ${SDK_NAME}/ ==="
echo ""
echo "Directory structure:"
find "${PKG_DIR}" -type f | sort | sed "s|${PKG_DIR}|${SDK_NAME}|"
echo ""
echo "Library:"
ls -lh "${PKG_DIR}/lib/" 2>/dev/null || echo "  (empty)"
echo "Include:"
ls -lh "${PKG_DIR}/include/" 2>/dev/null || echo "  (empty)"
echo "Examples:"
ls "${PKG_DIR}/example/"*.c 2>/dev/null | head -5 || echo "  (empty)"
echo "..."
