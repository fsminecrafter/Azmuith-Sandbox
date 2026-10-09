param(
    [switch]$Check,
    [switch]$Build,
    [switch]$Install,
    [switch]$Help
)

$ErrorActionPreference = 'Stop'
$script:RootDir = $PSScriptRoot
$script:StateFile = Join-Path $script:RootDir '.setup-state'
$script:PhysxGpu = 'OFF'
$script:BuildType = 'Release'
$script:BuildExitCode = 0
$script:VulkanSdkRoot = $env:VULKAN_SDK
$script:CudaRoot = $env:CUDA_PATH
if (-not $script:CudaRoot) { $script:CudaRoot = $env:CUDA_HOME }

function Load-State {
    if (-not (Test-Path -LiteralPath $script:StateFile)) { return }
    foreach ($line in Get-Content -LiteralPath $script:StateFile) {
        $separator = $line.IndexOf('=')
        if ($separator -lt 1) { continue }
        $key = $line.Substring(0, $separator)
        $value = $line.Substring($separator + 1)
        switch ($key) {
            'PHYSX_GPU' { if ($value -eq 'ON') { $script:PhysxGpu = 'ON' } else { $script:PhysxGpu = 'OFF' } }
            'BUILD_TYPE' { if ($value -eq 'Debug') { $script:BuildType = 'Debug' } else { $script:BuildType = 'Release' } }
            'VULKAN_SDK_ROOT' { if ($value) { $script:VulkanSdkRoot = $value } }
            'CUDA_ROOT' { if ($value) { $script:CudaRoot = $value } }
        }
    }
}

function Save-State {
    @(
        "PHYSX_GPU=$script:PhysxGpu"
        "BUILD_TYPE=$script:BuildType"
        "VULKAN_SDK_ROOT=$script:VulkanSdkRoot"
        "CUDA_ROOT=$script:CudaRoot"
    ) | Set-Content -LiteralPath $script:StateFile -Encoding ASCII
}

function Find-VulkanHeader {
    $roots = @($script:VulkanSdkRoot, $env:VULKAN_SDK, "$env:ProgramFiles\VulkanSDK") | Where-Object { $_ }
    foreach ($root in $roots) {
        if (Test-Path -LiteralPath (Join-Path $root 'Include\vulkan\vulkan.h')) {
            return (Join-Path $root 'Include\vulkan\vulkan.h')
        }
        if (Test-Path -LiteralPath (Join-Path $root 'include\vulkan\vulkan.h')) {
            return (Join-Path $root 'include\vulkan\vulkan.h')
        }
    }
    return $null
}

function Find-VulkanLibrary {
    $roots = @($script:VulkanSdkRoot, $env:VULKAN_SDK, "$env:ProgramFiles\VulkanSDK") | Where-Object { $_ }
    foreach ($root in $roots) {
        foreach ($relativePath in @('Lib\vulkan-1.lib', 'lib\vulkan-1.lib')) {
            $candidate = Join-Path $root $relativePath
            if (Test-Path -LiteralPath $candidate) { return $candidate }
        }
    }
    return $null
}

function Find-Glslc {
    if ($script:VulkanSdkRoot) {
        foreach ($relativePath in @('Bin\glslc.exe', 'bin\glslc.exe')) {
            $candidate = Join-Path $script:VulkanSdkRoot $relativePath
            if (Test-Path -LiteralPath $candidate) { return $candidate }
        }
    }
    $command = Get-Command glslc.exe -ErrorAction SilentlyContinue
    if ($command) { return $command.Source }
    return $null
}

function Find-Nvcc {
    if ($script:CudaRoot) {
        $candidate = Join-Path $script:CudaRoot 'bin\nvcc.exe'
        if (Test-Path -LiteralPath $candidate) { return $candidate }
    }
    $command = Get-Command nvcc.exe -ErrorAction SilentlyContinue
    if ($command) { return $command.Source }
    return $null
}

function Show-DependencyReport {
    Write-Host "`nDependency check"
    foreach ($tool in @('cmake.exe', 'git.exe')) {
        $command = Get-Command $tool -ErrorAction SilentlyContinue
        if ($command) { Write-Host "  [ok] $($tool.PadRight(16)) $($command.Source)" }
        else { Write-Host "  [missing] $tool" }
    }
    $compiler = Get-Command cl.exe -ErrorAction SilentlyContinue
    $vswhere = $null
    if (${env:ProgramFiles(x86)}) { $vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe' }
    if ($compiler) { Write-Host "  [ok] C++ compiler    $($compiler.Source)" }
    elseif ($vswhere -and (Test-Path -LiteralPath $vswhere)) { Write-Host '  [ok] Visual Studio C++ Build Tools installed' }
    else { Write-Host '  [missing] Visual Studio C++ Build Tools' }
    $header = Find-VulkanHeader
    if ($header) { Write-Host "  [ok] Vulkan headers   $header" } else { Write-Host '  [missing] Vulkan headers' }
    $library = Find-VulkanLibrary
    if ($library) { Write-Host "  [ok] Vulkan loader    $library" } else { Write-Host '  [missing] Vulkan SDK loader/import library' }
    $glslc = Find-Glslc
    if ($glslc) { Write-Host "  [ok] glslc            $glslc" } else { Write-Host '  [optional] glslc not found; viewport uses CPU projection' }
    $nvcc = Find-Nvcc
    if ($nvcc) { Write-Host "  [ok] CUDA compiler    $nvcc" } else { Write-Host '  [optional] nvcc not found; PhysX GPU dynamics unavailable' }
    Write-Host "`nPhysX and C++ dependencies are fetched by CMake on first configure."
}

function Find-SdkRoots {
    $searchRoot = Read-Host "Search directory [$HOME]"
    if (-not $searchRoot) { $searchRoot = $HOME }
    if (-not (Test-Path -LiteralPath $searchRoot -PathType Container)) {
        Write-Host "Directory not found: $searchRoot"
        return
    }
    Write-Host "Searching $searchRoot for Vulkan and CUDA SDKs..."

    $header = $null
    if (Test-Path -LiteralPath (Join-Path $searchRoot 'Include\vulkan\vulkan.h')) {
        $header = Join-Path $searchRoot 'Include\vulkan\vulkan.h'
    } elseif (Test-Path -LiteralPath (Join-Path $searchRoot 'include\vulkan\vulkan.h')) {
        $header = Join-Path $searchRoot 'include\vulkan\vulkan.h'
    } else {
        $header = Get-ChildItem -LiteralPath $searchRoot -Filter vulkan.h -File -Recurse -ErrorAction SilentlyContinue |
            Where-Object { $_.FullName -match '[\\/]include[\\/]vulkan[\\/]vulkan\.h$' } |
            Select-Object -First 1
        if ($header) { $header = $header.FullName }
    }
    if ($header) {
        $script:VulkanSdkRoot = Split-Path (Split-Path (Split-Path $header -Parent) -Parent) -Parent
        Write-Host "  Vulkan SDK: $script:VulkanSdkRoot"
    } else { Write-Host '  Vulkan SDK: not found' }

    $nvcc = $null
    $directNvcc = Join-Path $searchRoot 'bin\nvcc.exe'
    if (Test-Path -LiteralPath $directNvcc) { $nvcc = $directNvcc }
    else {
        $nvcc = Get-ChildItem -LiteralPath $searchRoot -Filter nvcc.exe -File -Recurse -ErrorAction SilentlyContinue |
            Select-Object -First 1
        if ($nvcc) { $nvcc = $nvcc.FullName }
    }
    if ($nvcc) {
        $script:CudaRoot = Split-Path (Split-Path $nvcc -Parent) -Parent
        Write-Host "  CUDA Toolkit: $script:CudaRoot"
    } else { Write-Host '  CUDA Toolkit: not found' }
    Save-State
}

function Persist-SdkPaths {
    if ($script:VulkanSdkRoot) {
        [Environment]::SetEnvironmentVariable('VULKAN_SDK', $script:VulkanSdkRoot, 'User')
        Add-UserPathEntry (Join-Path $script:VulkanSdkRoot 'Bin')
    }
    if ($script:CudaRoot) {
        [Environment]::SetEnvironmentVariable('CUDA_PATH', $script:CudaRoot, 'User')
        [Environment]::SetEnvironmentVariable('CUDA_HOME', $script:CudaRoot, 'User')
        Add-UserPathEntry (Join-Path $script:CudaRoot 'bin')
    }
    Write-Host 'SDK environment variables and PATH entries saved for this Windows user.'
}

function Add-UserPathEntry {
    param([string]$entry)
    if (-not (Test-Path -LiteralPath $entry -PathType Container)) { return }
    $userPath = [Environment]::GetEnvironmentVariable('Path', 'User')
    $entries = @($userPath -split ';' | Where-Object { $_ })
    if (-not ($entries | Where-Object { $_.TrimEnd('\') -ieq $entry.TrimEnd('\') })) {
        $entries += $entry
        [Environment]::SetEnvironmentVariable('Path', ($entries -join ';'), 'User')
    }
}

function Add-SdkPaths {
    $choice = (Read-Host 'Add which SDK? [V]ulkan / [C]UDA').ToUpperInvariant()
    if ($choice -eq 'V') {
        $root = $script:VulkanSdkRoot
        if (-not $root -or -not (Find-VulkanHeader)) { $root = Read-Host 'Vulkan SDK root' }
        if (-not $root) { return }
        $headerPath = Join-Path $root 'Include\vulkan\vulkan.h'
        if (-not (Test-Path -LiteralPath $headerPath)) { $headerPath = Join-Path $root 'include\vulkan\vulkan.h' }
        if (-not (Test-Path -LiteralPath $headerPath)) {
            Write-Host "No Vulkan headers found under $root"
            return
        }
        $script:VulkanSdkRoot = $root
        $env:VULKAN_SDK = $root
        $env:PATH = "$(Join-Path $root 'Bin');$(Join-Path $root 'Bin32');$env:PATH"
    } elseif ($choice -eq 'C') {
        $root = $script:CudaRoot
        if (-not $root -or -not (Test-Path -LiteralPath (Join-Path $root 'bin\nvcc.exe'))) { $root = Read-Host 'CUDA Toolkit root' }
        if (-not $root) { return }
        if (-not (Test-Path -LiteralPath (Join-Path $root 'bin\nvcc.exe'))) {
            Write-Host "nvcc.exe not found under $root\bin"
            return
        }
        $script:CudaRoot = $root
        $env:CUDA_PATH = $root
        $env:CUDA_HOME = $root
        $env:PATH = "$(Join-Path $root 'bin');$env:PATH"
    } else {
        Write-Host 'Choose V or C.'
        return
    }
    Save-State
    Write-Host 'SDK paths added to this setup session.'
    if ((Read-Host 'Persist paths in the Windows user environment? [y/N]').ToLowerInvariant() -eq 'y') {
        Persist-SdkPaths
    }
}

function Install-Dependencies {
    $winget = Get-Command winget.exe -ErrorAction SilentlyContinue
    if (-not $winget) {
        Write-Host 'winget was not found. Install App Installer, then rerun this action.'
        return
    }
    $packages = @(
        @{ Id = 'Kitware.CMake'; Name = 'CMake' },
        @{ Id = 'Git.Git'; Name = 'Git' },
        @{ Id = 'Microsoft.VisualStudio.2022.BuildTools'; Name = 'Visual Studio C++ Build Tools' }
    )
    Write-Host "`nThe following Windows build dependencies will be installed:"
    foreach ($package in $packages) { Write-Host "  $($package.Name) [$($package.Id)]" }
    Write-Host 'CUDA is optional and should be installed from NVIDIA if PhysX GPU is needed.'
    if ((Read-Host 'Continue? [y/N]').ToLowerInvariant() -ne 'y') { return }

    foreach ($package in $packages) {
        Write-Host "`nInstalling $($package.Name)..."
        $arguments = @('install', '--id', $package.Id, '--exact', '--accept-source-agreements', '--accept-package-agreements')
        if ($package.Id -eq 'Microsoft.VisualStudio.2022.BuildTools') {
            $arguments += @('--override', '--wait --quiet --add Microsoft.VisualStudio.Workload.VCTools --includeRecommended')
        }
        & $winget.Source @arguments
        if ($LASTEXITCODE -ne 0) { Write-Warning "winget failed for $($package.Name) (exit $LASTEXITCODE)." }
    }

    Write-Host "`nSearching the configured winget source for a Vulkan SDK package..."
    & $winget.Source search --query 'Vulkan SDK' --source winget
    if ($LASTEXITCODE -eq 0) {
        $vulkanId = Read-Host 'Enter the exact Vulkan SDK package ID shown above, or press Enter to skip'
        if ($vulkanId) {
            & $winget.Source install --id $vulkanId --exact --source winget --accept-source-agreements --accept-package-agreements
            if ($LASTEXITCODE -ne 0) {
                Write-Warning "winget could not install '$vulkanId' (exit $LASTEXITCODE)."
                Show-VulkanDownloadFallback
            }
        } else {
            Write-Host 'Vulkan SDK installation skipped.'
        }
    } else {
        Write-Warning 'The configured winget source did not return Vulkan SDK packages.'
        Show-VulkanDownloadFallback
    }
    Refresh-ProcessEnvironment
    Write-Host "`nRestart the terminal after installing SDKs so PATH changes are visible."
}

function Show-VulkanDownloadFallback {
    $downloadUrl = 'https://vulkan.lunarg.com/sdk/home#windows'
    Write-Host "Install the official Vulkan SDK from: $downloadUrl"
    if ((Read-Host 'Open the official LunarG download page? [y/N]').ToLowerInvariant() -eq 'y') {
        Start-Process $downloadUrl
    }
    Write-Host 'After installation, rerun setup.ps1 and choose A to add the Vulkan SDK path.'
}

function Refresh-ProcessEnvironment {
    $machinePath = [Environment]::GetEnvironmentVariable('Path', 'Machine')
    $userPath = [Environment]::GetEnvironmentVariable('Path', 'User')
    $pathParts = @()
    foreach ($value in @($machinePath, $userPath, $env:Path)) {
        if ($value) { $pathParts += $value -split ';' }
    }
    $env:Path = ($pathParts | Where-Object { $_ } | Select-Object -Unique) -join ';'
    $vulkanSdk = [Environment]::GetEnvironmentVariable('VULKAN_SDK', 'User')
    if (-not $vulkanSdk) { $vulkanSdk = [Environment]::GetEnvironmentVariable('VULKAN_SDK', 'Machine') }
    if ($vulkanSdk) {
        $env:VULKAN_SDK = $vulkanSdk
        $script:VulkanSdkRoot = $vulkanSdk
    }
    $cudaRoot = [Environment]::GetEnvironmentVariable('CUDA_PATH', 'User')
    if (-not $cudaRoot) { $cudaRoot = [Environment]::GetEnvironmentVariable('CUDA_PATH', 'Machine') }
    if ($cudaRoot) {
        $env:CUDA_PATH = $cudaRoot
        $script:CudaRoot = $cudaRoot
    }
}

function Build-Project {
    $script:BuildExitCode = 0
    $arguments = @('-S', $script:RootDir, '-B', (Join-Path $script:RootDir 'build'), "-DCMAKE_BUILD_TYPE=$script:BuildType",
        "-DAZMUITH_ENABLE_PHYSX_GPU=$script:PhysxGpu")
    if ($script:VulkanSdkRoot) {
        $headerRoot = Join-Path $script:VulkanSdkRoot 'Include'
        if (-not (Test-Path -LiteralPath $headerRoot)) { $headerRoot = Join-Path $script:VulkanSdkRoot 'include' }
        $library = Join-Path $script:VulkanSdkRoot 'Lib\vulkan-1.lib'
        if (-not (Test-Path -LiteralPath $library)) { $library = Join-Path $script:VulkanSdkRoot 'lib\vulkan-1.lib' }
        $glslc = Join-Path $script:VulkanSdkRoot 'Bin\glslc.exe'
        if (-not (Test-Path -LiteralPath $glslc)) { $glslc = Join-Path $script:VulkanSdkRoot 'bin\glslc.exe' }
        if (Test-Path -LiteralPath $headerRoot) { $arguments += "-DVulkan_INCLUDE_DIR=$headerRoot" }
        if (Test-Path -LiteralPath $library) { $arguments += "-DVulkan_LIBRARY=$library" }
        if (Test-Path -LiteralPath $glslc) { $arguments += "-DGLSLC_EXECUTABLE=$glslc" }
        $env:VULKAN_SDK = $script:VulkanSdkRoot
        $env:PATH = "$(Join-Path $script:VulkanSdkRoot 'Bin');$env:PATH"
    }
    if ($script:PhysxGpu -eq 'ON') {
        $nvcc = Find-Nvcc
        if (-not $nvcc) {
            Write-Host 'PHYSX GPU is ON, but nvcc.exe was not found. Search for CUDA or toggle PHYSX GPU OFF.'
            $script:BuildExitCode = 1
            return
        }
        $script:CudaRoot = Split-Path (Split-Path $nvcc -Parent) -Parent
        $env:CUDA_PATH = $script:CudaRoot
        $env:CUDAToolkit_ROOT = $script:CudaRoot
        $env:PATH = "$(Join-Path $script:CudaRoot 'bin');$env:PATH"
        $arguments += "-DCMAKE_CUDA_COMPILER=$nvcc"
    }
    Save-State
    Write-Host "`nConfiguring with PHYSX_GPU=$script:PhysxGpu, BUILD_TYPE=$script:BuildType"
    & cmake.exe @arguments
    if ($LASTEXITCODE -ne 0) {
        Write-Host 'CMake configuration failed. Use Dependency Check/Search to locate missing SDKs.'
        $script:BuildExitCode = $LASTEXITCODE
        return
    }
    & cmake.exe --build (Join-Path $script:RootDir 'build') --config $script:BuildType --parallel ([Environment]::ProcessorCount)
    $script:BuildExitCode = $LASTEXITCODE
}

function Pause-Menu {
    [void](Read-Host 'Press Enter to return to the menu')
}

function Show-Menu {
    Clear-Host
    Write-Host 'Azmuith Sandbox setup'
    Write-Host '---------------------'
    Write-Host "PHYSX GPU  : $script:PhysxGpu  (core PhysX remains required)"
    Write-Host "Build type : $script:BuildType"
    Write-Host "Vulkan SDK : $(if ($script:VulkanSdkRoot) { $script:VulkanSdkRoot } else { 'system search' })"
    Write-Host "CUDA root  : $(if ($script:CudaRoot) { $script:CudaRoot } else { 'system search' })`n"
    Write-Host '[B] Configure and build'
    Write-Host '[P] Toggle PHYSX GPU ON/OFF'
    Write-Host '[T] Toggle Release/Debug'
    Write-Host '[D] Install Windows dependencies'
    Write-Host '[C] Check dependencies'
    Write-Host '[S] Search for Vulkan/CUDA SDKs'
    Write-Host '[A] Add an SDK to PATH / persist it'
    Write-Host '[Q] Quit'
    Write-Host ''
    Write-Host -NoNewline 'Select an action: '
}

function Start-Menu {
    while ($true) {
        Show-Menu
        try { $key = [Console]::ReadKey($true).Key.ToString().ToLowerInvariant() }
        catch { $key = (Read-Host 'Select an action').ToLowerInvariant() }
        switch ($key) {
            'b' { Build-Project; if ($script:BuildExitCode -ne 0) { Write-Host "Build failed (exit $script:BuildExitCode)." }; Pause-Menu }
            'p' { if ($script:PhysxGpu -eq 'ON') { $script:PhysxGpu = 'OFF' } else { $script:PhysxGpu = 'ON' }; Save-State }
            't' { if ($script:BuildType -eq 'Release') { $script:BuildType = 'Debug' } else { $script:BuildType = 'Release' }; Save-State }
            'd' { Install-Dependencies; Pause-Menu }
            'c' { Show-DependencyReport; Pause-Menu }
            's' { Find-SdkRoots; Pause-Menu }
            'a' { Add-SdkPaths; Pause-Menu }
            'q' { Save-State; return }
            default { Write-Host "Unknown key: $key"; Pause-Menu }
        }
    }
}

Load-State
if ($Help) {
    Write-Host 'Usage: .\setup.ps1 [-Check|-Build|-Install|-Help]'
    Write-Host 'Run without switches for the interactive setup TUI.'
} elseif ($Check) {
    Show-DependencyReport
} elseif ($Install) {
    Install-Dependencies
} elseif ($Build) {
    Build-Project
    if ($script:BuildExitCode -ne 0) { exit $script:BuildExitCode }
} else {
    Start-Menu
}