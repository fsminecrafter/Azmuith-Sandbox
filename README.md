# Azmuith Physics Lab

A native C++ physics playground built around NVIDIA PhysX, Vulkan, GLFW, and Dear ImGui. The editor includes a perspective scene view, primitive placement, transform gizmos, generated gears, axle/bearing snapping, fixed-link structures, glue bonds, and scene persistence.

## Requirements

- CMake 3.24 or newer and a C++20 compiler
- Vulkan 1.1 SDK, loader, and a working desktop Vulkan driver
- `glslc` from the Vulkan SDK for the depth-tested GPU viewport; without it, the viewport uses CPU projection
- CUDA Toolkit and a CUDA-capable NVIDIA GPU for GPU PhysX
- Network access on first configure to fetch GLFW, GLM, Dear ImGui, nlohmann/json, and PhysX 5.6.1

## Build

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
./build/azmuith_sandbox
```

Run `./setup.sh` for the interactive setup menu. Press `B` to configure/build, `P` to toggle CUDA-backed PhysX dynamics, `D` to install Debian/Ubuntu dependencies, and `S`/`A` to search for SDKs and add their paths.

On Windows, run `powershell -ExecutionPolicy Bypass -File .\setup.ps1` for the equivalent PowerShell menu. It uses `winget` for prerequisites and can persist SDK paths in the user environment.

On Windows, run `powershell -ExecutionPolicy Bypass -File .\setup.ps1` for the equivalent PowerShell TUI. It uses `winget` to install Windows prerequisites and persists SDK paths in the user environment when requested.

When CMake detects a CUDA compiler, it enables PhysX GPU targets automatically. Force the CPU-only build with `-DAZMUITH_ENABLE_PHYSX_GPU=OFF`, or request the GPU build explicitly with `-DAZMUITH_ENABLE_PHYSX_GPU=ON`. At startup the application uses GPU dynamics when the CUDA context is valid and otherwise keeps the scene on CPU.

When `glslc` is available, the viewport renders its perspective scene into a Vulkan color/depth target and displays that target in ImGui. Without `glslc`, or if GPU target setup fails at runtime, it falls back to CPU projection while retaining the Vulkan-backed editor UI.

## Editing

- Choose Select, Box, Sphere, Cylinder, Gear, Axle, Bearing, Structure, or Glue from the scene tools. Click in the viewport to place; the active tool shows a ghost preview.
- Select an object and drag it, or use the axis gizmo. Press `E` for move, `R` for rotate, and `T` for scale; the inspector has matching controls.
- Configure gear teeth, module, thickness, and bore in the inspector. Gear placement snaps to nearby axles, axle placement snaps to bearings, and Snap Placements can be disabled.
- Select Structure, then click two endpoints to create anchor cubes and a fixed cylinder link. Glue balls create fixed bonds to parts they contact.
- Enter a scene path and use Save or Load. Autosave writes `scene_autosave.json` every 30 seconds.
- Open `Settings` → `Graphics` to toggle ambient occlusion, FXAA, and smooth PCF shadows. These effects run in the GPU viewport; the CPU projection fallback remains available without `glslc`.