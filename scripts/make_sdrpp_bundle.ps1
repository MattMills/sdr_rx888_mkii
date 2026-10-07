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

# config.json: SDR++'s Windows defaults are "./modules" and "./res", relative to the working
# directory rather than the executable, and its default module list has no RX888 entry. So
# "sdrpp.exe -r <Out>" from elsewhere fails with "Resource directory doesn't exist!", and a
# fresh config never loads this plugin. Write absolute paths and the instance; SDR++ fills in
# every other key on first start. An existing, readable config keeps its settings.
$outAbs = (Resolve-Path $Out).Path -replace '\\', '/'
$configPath = Join-Path $Out "config.json"
$conf = $null
if (Test-Path $configPath) {
    try { $conf = Get-Content -Raw $configPath | ConvertFrom-Json } catch { $conf = $null }
}
if ($null -eq $conf) {
    # SDR++'s default instances (core/src/core.cpp), kept only where the module was built
    $known = [ordered]@{
        "airspy_source" = "Airspy Source"; "airspyhf_source" = "AirspyHF+ Source"
        "audio_source" = "Audio Source"; "bladerf_source" = "BladeRF Source"
        "file_source" = "File Source"; "hackrf_source" = "HackRF Source"
        "hermes_source" = "Hermes Source"; "limesdr_source" = "LimeSDR Source"
        "network_source" = "Network Source"; "plutosdr_source" = "PlutoSDR Source"
        "rfspace_source" = "RFspace Source"; "rtl_sdr_source" = "RTL-SDR Source"
        "rtl_tcp_source" = "RTL-TCP Source"; "sdrplay_source" = "SDRplay Source"
        "sdrpp_server_source" = "SDR++ Server Source"; "spectran_http_source" = "Spectran HTTP Source"
        "spyserver_source" = "SpyServer Source"; "usrp_source" = "USRP Source"
        "audio_sink" = "Audio Sink"; "network_sink" = "Network Sink"; "radio" = "Radio"
        "frequency_manager" = "Frequency Manager"; "recorder" = "Recorder"; "rigctl_server" = "Rigctl Server"
    }
    $instances = [ordered]@{}
    foreach ($mod in $known.Keys) {
        if (Test-Path (Join-Path $Out "modules\$mod.dll")) {
            $instances[$known[$mod]] = [ordered]@{ module = $mod; enabled = $true }
        }
    }
    $conf = [pscustomobject]@{ moduleInstances = [pscustomobject]$instances; source = ""; frequency = 7100000.0 }
}
$conf | Add-Member -Force NoteProperty modulesDirectory "$outAbs/modules"
$conf | Add-Member -Force NoteProperty resourcesDirectory "$outAbs/res"
if (-not ($conf.moduleInstances.PSObject.Properties.Name -contains "RX888 mkII Source")) {
    $conf.moduleInstances | Add-Member NoteProperty "RX888 mkII Source" ([pscustomobject]@{ module = "rx888_mkii_source"; enabled = $true })
}
if (-not $conf.source) { $conf | Add-Member -Force NoteProperty source "RX888 mkII" }
# UTF-8 without a BOM
[System.IO.File]::WriteAllText($configPath, ($conf | ConvertTo-Json -Depth 20), (New-Object System.Text.UTF8Encoding $false))

Write-Host "SDR++ bundle ready in $Out"
Write-Host "Run: `"$Out\sdrpp.exe`"  (or from anywhere: `"$Out\sdrpp.exe`" -r `"$Out`")"
Write-Host "Close SDR++ normally; killing it while it saves can truncate config.json."
