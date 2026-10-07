# Assemble a portable SDR++ folder from an SDR++ build tree plus this plugin.
#
#   .\scripts\make_sdrpp_bundle.ps1 -SdrppSource C:\path\SDRPlusPlus -SdrppBuild C:\path\SDRPlusPlus\build `
#       -VcpkgInstalled C:\path\vcpkg\installed\x64-windows -VolkBin C:\path\volk-install\bin -Out C:\path\sdrpp_rx888
#
# Then run:  <Out>\sdrpp.exe -r <Out>
param(
    [Parameter(Mandatory = $true)][string]$SdrppSource,
    [Parameter(Mandatory = $true)][string]$SdrppBuild,
    [Parameter(Mandatory = $true)][string]$VcpkgInstalled,
    [Parameter(Mandatory = $true)][string]$VolkBin,
    [Parameter(Mandatory = $true)][string]$Out,
    [string]$PluginBuild = (Join-Path $PSScriptRoot "..\build\Release"),
    [string]$Config = "Release"
)
$ErrorActionPreference = "Stop"

New-Item -ItemType Directory -Force $Out | Out-Null
New-Item -ItemType Directory -Force (Join-Path $Out "modules") | Out-Null

# Resources (res/), SDR++ executable and core
Copy-Item -Recurse -Force (Join-Path $SdrppSource "root\*") $Out
Copy-Item -Force (Join-Path $SdrppBuild "$Config\*") $Out

# Runtime dependencies
foreach ($dll in @("fftw3f.dll", "glfw3.dll", "zstd.dll", "rtaudio.dll", "libusb-1.0.dll")) {
    $p = Join-Path $VcpkgInstalled "bin\$dll"
    if (Test-Path $p) { Copy-Item -Force $p $Out }
}
Copy-Item -Force (Join-Path $VolkBin "volk.dll") $Out

# SDR++ modules that were built
foreach ($kind in @("source_modules", "sink_modules", "decoder_modules", "misc_modules")) {
    $dir = Join-Path $SdrppBuild $kind
    if (-not (Test-Path $dir)) { continue }
    Get-ChildItem -Path $dir -Directory | ForEach-Object {
        $mod = Join-Path $_.FullName "$Config\$($_.Name).dll"
        if (Test-Path $mod) { Copy-Item -Force $mod (Join-Path $Out "modules") }
    }
}

# This plugin
Copy-Item -Force (Join-Path $PluginBuild "rx888_mkii_source.dll") (Join-Path $Out "modules")
if (Test-Path (Join-Path $PluginBuild "rx888_tool.exe")) {
    Copy-Item -Force (Join-Path $PluginBuild "rx888_tool.exe") $Out
}

Write-Host "SDR++ bundle ready in $Out"
Write-Host "Run: `"$Out\sdrpp.exe`" -r `"$Out`""
