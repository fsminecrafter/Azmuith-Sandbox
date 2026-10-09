#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STATE_FILE="$ROOT_DIR/.setup-state"
ENV_FILE="$HOME/.azmuith-sandbox-env.sh"
PHYSX_GPU="OFF"
BUILD_TYPE="Release"
VULKAN_SDK_ROOT="${VULKAN_SDK:-}"
CUDA_ROOT="${CUDA_HOME:-${CUDA_PATH:-}}"
SEARCH_ROOT=""

load_state() {
    [[ -f "$STATE_FILE" ]] || return 0
    while IFS='=' read -r key value; do
        case "$key" in
            PHYSX_GPU) [[ "$value" == "ON" ]] && PHYSX_GPU="ON" || PHYSX_GPU="OFF" ;;
            BUILD_TYPE) [[ "$value" == "Debug" ]] && BUILD_TYPE="Debug" || BUILD_TYPE="Release" ;;
            VULKAN_SDK_ROOT) VULKAN_SDK_ROOT="$value" ;;
            CUDA_ROOT) CUDA_ROOT="$value" ;;
        esac
    done < "$STATE_FILE"
}

save_state() {
    printf 'PHYSX_GPU=%s\nBUILD_TYPE=%s\nVULKAN_SDK_ROOT=%s\nCUDA_ROOT=%s\n' \
        "$PHYSX_GPU" "$BUILD_TYPE" "$VULKAN_SDK_ROOT" "$CUDA_ROOT" > "$STATE_FILE"
}

load_state

find_vulkan_header() {
    local root
    for root in "$VULKAN_SDK_ROOT" /usr /usr/local /opt/vulkan-sdk; do
        [[ -n "$root" && -f "$root/include/vulkan/vulkan.h" ]] && { printf '%s\n' "$root/include/vulkan/vulkan.h"; return 0; }
    done
    [[ -f /usr/include/vulkan/vulkan.h ]] && { printf '%s\n' /usr/include/vulkan/vulkan.h; return 0; }
    return 1
}

find_vulkan_library() {
    local root candidate
    for root in "$VULKAN_SDK_ROOT" /usr /usr/local /opt/vulkan-sdk; do
        [[ -n "$root" ]] || continue
        for candidate in "$root/lib/libvulkan.so" "$root/lib64/libvulkan.so" \
            "$root/lib/x86_64-linux-gnu/libvulkan.so" "$root/Lib/vulkan-1.lib"; do
            [[ -f "$candidate" ]] && { printf '%s\n' "$candidate"; return 0; }
        done
    done
    if command -v ldconfig >/dev/null 2>&1; then
        ldconfig -p 2>/dev/null | awk '/libvulkan\.so[[:space:]]/ { print $NF; exit }'
        return "${PIPESTATUS[0]}"
    fi
    return 1
}

find_glslc() {
    if [[ -n "$VULKAN_SDK_ROOT" ]]; then
        for candidate in "$VULKAN_SDK_ROOT/bin/glslc" "$VULKAN_SDK_ROOT/Bin/glslc"; do
            [[ -x "$candidate" ]] && { printf '%s\n' "$candidate"; return 0; }
        done
    fi
    command -v glslc 2>/dev/null || return 1
}

find_nvcc() {
    if [[ -n "$CUDA_ROOT" ]]; then
        for candidate in "$CUDA_ROOT/bin/nvcc" "$CUDA_ROOT/bin/nvcc.exe"; do
            [[ -x "$candidate" ]] && { printf '%s\n' "$candidate"; return 0; }
        done
    fi
    command -v nvcc 2>/dev/null || return 1
}

dependency_report() {
    local header library glslc nvcc
    printf '\nDependency check\n'
    for tool in cmake c++ git; do
        if command -v "$tool" >/dev/null 2>&1; then
            printf '  [ok] %-16s %s\n' "$tool" "$(command -v "$tool")"
        else
            printf '  [missing] %s\n' "$tool"
        fi
    done
    if header="$(find_vulkan_header)"; then printf '  [ok] Vulkan headers   %s\n' "$header"; else printf '  [missing] Vulkan headers\n'; fi
    if library="$(find_vulkan_library)" && [[ -n "$library" ]]; then printf '  [ok] Vulkan loader    %s\n' "$library"; else printf '  [missing] Vulkan loader\n'; fi
    if glslc="$(find_glslc)"; then printf '  [ok] glslc            %s\n' "$glslc"; else printf '  [optional] glslc not found; viewport uses CPU projection\n'; fi
    if nvcc="$(find_nvcc)"; then printf '  [ok] CUDA compiler    %s\n' "$nvcc"; else printf '  [optional] nvcc not found; PhysX GPU dynamics unavailable\n'; fi
    printf '\nPhysX source and the C++ libraries are fetched by CMake on first configure.\n'
}

find_sdk_roots() {
    local search_root header nvcc_path
    search_root="${1:-$HOME}"
    if [[ ! -d "$search_root" ]]; then
        printf 'Directory not found: %s\n' "$search_root"
        return 1
    fi
    printf 'Searching %s for Vulkan and CUDA SDKs...\n' "$search_root"

    header=""
    if [[ -f "$search_root/include/vulkan/vulkan.h" ]]; then
        header="$search_root/include/vulkan/vulkan.h"
    else
        header="$(find "$search_root" -maxdepth 7 -type f -path '*/include/vulkan/vulkan.h' -print -quit 2>/dev/null || true)"
    fi
    if [[ -n "$header" ]]; then
        VULKAN_SDK_ROOT="$(dirname "$(dirname "$(dirname "$header")")")"
        printf '  Vulkan SDK: %s\n' "$VULKAN_SDK_ROOT"
    else
        printf '  Vulkan SDK: not found\n'
    fi

    if [[ -x "$search_root/bin/nvcc" ]]; then
        nvcc_path="$search_root/bin/nvcc"
    else
        nvcc_path="$(find "$search_root" -maxdepth 7 -type f -name nvcc -perm -u+x -print -quit 2>/dev/null || true)"
    fi
    if [[ -n "$nvcc_path" ]]; then
        CUDA_ROOT="$(dirname "$(dirname "$nvcc_path")")"
        printf '  CUDA Toolkit: %s\n' "$CUDA_ROOT"
    else
        printf '  CUDA Toolkit: not found\n'
    fi
    save_state
}

add_sdk_paths() {
    local choice root
    read -r -p 'Add which SDK? [V]ulkan / [C]UDA: ' choice
    case "${choice,,}" in
        v)
            root="$VULKAN_SDK_ROOT"
            if [[ -z "$root" || ! -d "$root" ]]; then
                read -r -p 'Vulkan SDK root: ' root
            fi
            if [[ ! -f "$root/include/vulkan/vulkan.h" ]]; then
                printf 'No Vulkan headers found under %s\n' "$root"
                return 1
            fi
            VULKAN_SDK_ROOT="$root"
            export VULKAN_SDK="$root"
            export PATH="$root/bin:$root/Bin:$PATH"
            ;;
        c)
            root="$CUDA_ROOT"
            if [[ -z "$root" || ! -x "$root/bin/nvcc" ]]; then
                read -r -p 'CUDA Toolkit root: ' root
            fi
            if [[ ! -x "$root/bin/nvcc" ]]; then
                printf 'nvcc not found under %s/bin\n' "$root"
                return 1
            fi
            CUDA_ROOT="$root"
            export CUDA_HOME="$root"
            export PATH="$root/bin:$PATH"
            export LD_LIBRARY_PATH="$root/lib64:${LD_LIBRARY_PATH:-}"
            ;;
        *) printf 'Choose V or C.\n'; return 1 ;;
    esac
    save_state
    printf 'SDK paths added to this setup session.\n'
    read -r -p 'Persist these paths in ~/.bashrc for future terminals? [y/N] ' choice
    if [[ "${choice,,}" == "y" ]]; then persist_sdk_paths; fi
}

persist_sdk_paths() {
    local bashrc="$HOME/.bashrc"
    {
        printf '#!/usr/bin/env bash\n'
        if [[ -n "$VULKAN_SDK_ROOT" ]]; then
            printf 'export VULKAN_SDK=%q\n' "$VULKAN_SDK_ROOT"
            printf 'export PATH=%q:"$PATH"\n' "$VULKAN_SDK_ROOT/bin"
            printf 'export PATH=%q:"$PATH"\n' "$VULKAN_SDK_ROOT/Bin"
        fi
        if [[ -n "$CUDA_ROOT" ]]; then
            printf 'export CUDA_HOME=%q\n' "$CUDA_ROOT"
            printf 'export PATH=%q:"$PATH"\n' "$CUDA_ROOT/bin"
            printf 'export LD_LIBRARY_PATH=%q:"${LD_LIBRARY_PATH:-}"\n' "$CUDA_ROOT/lib64"
        fi
    } > "$ENV_FILE"
    if [[ ! -f "$bashrc" ]] || ! grep -Fq 'azmuith-sandbox-env.sh' "$bashrc"; then
        printf '\n# Azmuith Sandbox SDK paths\nsource %q\n' "$ENV_FILE" >> "$bashrc"
    fi
    printf 'Saved SDK environment to %s and added it to ~/.bashrc.\n' "$ENV_FILE"
}

install_dependencies() {
    local -a packages=(build-essential cmake git pkg-config libvulkan-dev libglfw3-dev \
        libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev \
        libwayland-dev wayland-protocols libxkbcommon-dev glslc mesa-vulkan-drivers)
    if ! command -v apt-get >/dev/null 2>&1; then
        printf 'Automatic dependency installation is currently supported on Debian/Ubuntu (apt-get).\n'
        return 1
    fi
    printf 'Will install: %s\n' "${packages[*]}"
    read -r -p 'Continue? [y/N] ' answer
    [[ "${answer,,}" == "y" ]] || return 0
    if [[ "$EUID" -eq 0 ]]; then
        apt-get update && apt-get install -y "${packages[@]}"
    elif command -v sudo >/dev/null 2>&1; then
        sudo apt-get update && sudo apt-get install -y "${packages[@]}"
    else
        printf 'Install sudo or run setup.sh as root to install packages.\n'
        return 1
    fi
    printf "\nCUDA is an optional NVIDIA toolkit; install it from NVIDIA's CUDA repository if GPU PhysX is desired.\n"
}

build_project() {
    local -a cmake_args
    local library compiler nvcc
    if [[ "$PHYSX_GPU" == "ON" ]] && ! find_nvcc >/dev/null; then
        printf 'PHYSX GPU is ON, but nvcc was not found. Search for CUDA or toggle PHYSX GPU OFF.\n'
        return 1
    fi
    cmake_args=(-S "$ROOT_DIR" -B "$ROOT_DIR/build" "-DCMAKE_BUILD_TYPE=$BUILD_TYPE"
        "-DAZMUITH_ENABLE_PHYSX_GPU=$PHYSX_GPU")
    if [[ -n "$VULKAN_SDK_ROOT" ]]; then
        [[ -d "$VULKAN_SDK_ROOT/include" ]] && cmake_args+=("-DVulkan_INCLUDE_DIR=$VULKAN_SDK_ROOT/include")
        export VULKAN_SDK="$VULKAN_SDK_ROOT"
        export PATH="$VULKAN_SDK_ROOT/bin:$VULKAN_SDK_ROOT/Bin:$PATH"
    fi
    if library="$(find_vulkan_library)" && [[ -n "$library" ]]; then cmake_args+=("-DVulkan_LIBRARY=$library"); fi
    if compiler="$(find_glslc)"; then cmake_args+=("-DGLSLC_EXECUTABLE=$compiler"); fi
    if [[ "$PHYSX_GPU" == "ON" ]]; then
        nvcc="$(find_nvcc)"
        CUDA_ROOT="$(dirname "$(dirname "$nvcc")")"
        export CUDA_HOME="$CUDA_ROOT"
        export CUDAToolkit_ROOT="$CUDA_ROOT"
        export PATH="$CUDA_ROOT/bin:$PATH"
        cmake_args+=("-DCMAKE_CUDA_COMPILER=$nvcc")
    fi
    save_state
    printf '\nConfiguring with PHYSX_GPU=%s, BUILD_TYPE=%s\n' "$PHYSX_GPU" "$BUILD_TYPE"
    if ! cmake "${cmake_args[@]}"; then
        printf '\nCMake configuration failed. Use Dependency Check/Search to locate missing SDKs.\n'
        return 1
    fi
    cmake --build "$ROOT_DIR/build" --parallel "$(getconf _NPROCESSORS_ONLN 2>/dev/null || printf '2')"
}

pause_menu() {
    printf '\n'
    read -r -p 'Press Enter to return to the menu...' _
}

show_menu() {
    if [[ -t 1 ]] && command -v clear >/dev/null 2>&1; then clear; fi
    printf 'Azmuith Sandbox setup\n'
    printf '%s\n' '---------------------'
    printf 'PHYSX GPU  : %s  (core PhysX remains required)\n' "$PHYSX_GPU"
    printf 'Build type : %s\n' "$BUILD_TYPE"
    printf 'Vulkan SDK : %s\n' "${VULKAN_SDK_ROOT:-system search}"
    printf 'CUDA root  : %s\n\n' "${CUDA_ROOT:-system search}"
    printf '[B] Configure and build\n'
    printf '[P] Toggle PHYSX GPU ON/OFF\n'
    printf '[T] Toggle Release/Debug\n'
    printf '[D] Install Linux dependencies\n'
    printf '[C] Check dependencies\n'
    printf '[S] Search for Vulkan/CUDA SDKs\n'
    printf '[A] Add an SDK to PATH / persist it\n'
    printf '[Q] Quit\n\n'
    printf 'Select an action: '
}

run_menu() {
    local key search_path
    while true; do
        show_menu
        IFS= read -r -s -n 1 key || return 0
        printf '\n'
        case "${key,,}" in
            b) build_project || true; pause_menu ;;
            p) [[ "$PHYSX_GPU" == "ON" ]] && PHYSX_GPU="OFF" || PHYSX_GPU="ON"; save_state ;;
            t) [[ "$BUILD_TYPE" == "Release" ]] && BUILD_TYPE="Debug" || BUILD_TYPE="Release"; save_state ;;
            d) install_dependencies || true; pause_menu ;;
            c) dependency_report; pause_menu ;;
            s)
                read -r -p "Search root [${HOME}]: " search_path
                find_sdk_roots "${search_path:-$HOME}" || true
                pause_menu
                ;;
            a) add_sdk_paths || true; pause_menu ;;
            q) save_state; return 0 ;;
            *) printf 'Unknown key: %s\n' "$key"; pause_menu ;;
        esac
    done
}

case "${1:-}" in
    --check) dependency_report ;;
    --build) build_project ;;
    --install) install_dependencies ;;
    --help|-h)
        printf 'Usage: %s [--check|--build|--install|--help]\n' "${0##*/}"
        printf 'Run without arguments for the interactive setup TUI.\n'
        ;;
    "")
        if [[ ! -t 0 || ! -t 1 ]]; then
            printf 'setup.sh needs an interactive terminal for its TUI. Use --check, --build, or --install for non-TUI actions.\n' >&2
            exit 2
        fi
        run_menu
        ;;
    *) printf 'Unknown option: %s (try --help)\n' "$1" >&2; exit 2 ;;
esac