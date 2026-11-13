#!/bin/bash
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
    echo "[error] This script must be run as root: sudo bash scripts/ubuntu_clang_setup.sh" >&2
    exit 1
fi

apt install wget curl unzip zip ninja-build make git

wget https://apt.llvm.org/llvm.sh
chmod +x llvm.sh
# 只装基础组件集（clang/lld/clangd），不用 "all"：
# "all" 会带进 libunwind-21-dev（LLVM unwinder），与 ceres 依赖链
# （libceres-dev → libgoogle-glog-dev → libunwind-dev，glibc unwinder）
# 冲突，导致 apt 反复横跳：llvm.sh 装完 ceres 被卸，重装 ceres 又卸 libunwind-21-dev。
./llvm.sh 21 -m https://mirrors.tuna.tsinghua.edu.cn/llvm-apt
# update-alternatives 需要的其余工具（clang-query/clang-include-fixer 在 clang-tools-21 里）
apt install -y clang-format-21 clang-tidy-21 clang-tools-21

# 清理旧机器上由 llvm.sh all 残留的冲突包（新机器上是无害的空操作）
apt remove -y libunwind-21-dev

apt install -y libgtest-dev nlohmann-json3-dev libceres-dev libusb-1.0-0-dev

# update default linker, compiler to llvm-21
update-alternatives --install /usr/bin/clang clang /usr/bin/clang-21 100
update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-21 100
update-alternatives --install /usr/bin/ld ld /usr/bin/lld-21 100

# clangd, clang-format, clang-tidy, etc...
# update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-21 100
update-alternatives --install /usr/bin/clang-format clang-format /usr/bin/clang-format-21 100
update-alternatives --install /usr/bin/clang-tidy clang-tidy /usr/bin/clang-tidy-21 100
update-alternatives --install /usr/bin/clang-query clang-query /usr/bin/clang-query-21 100
update-alternatives --install /usr/bin/clang-include-fixer clang-include-fixer /usr/bin/clang-include-fixer-21 100

add-apt-repository ppa:ubuntu-toolchain-r/test
# replace /etc/apt/sources.list.d/ubuntu-toolchain-r-ubuntu-test-*.list ppa.launchpadcontent.net to launchpad.proxy.ustclug.org

for f in /etc/apt/sources.list.d/ubuntu-toolchain-r-ubuntu-test-*.list; do
    [[ -e "$f" ]] || continue
    cp "$f" "$f.bak"
    sed -i 's/ppa.launchpadcontent.net/launchpad.proxy.ustclug.org/g' "$f"
done

for f in /etc/apt/sources.list.d/ubuntu-toolchain-r-ubuntu-test-*.sources; do
    [[ -e "$f" ]] || continue
    cp "$f" "$f.bak"
    sed -i 's/ppa.launchpadcontent.net/launchpad.proxy.ustclug.org/g' "$f"
done

apt update
apt install -y g++-13 gcc-13
apt install libgcc-13-dev libstdc++-13-dev
