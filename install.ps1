# irm https://raw.githubusercontent.com/Strtus/rvd-railbench/main/install.ps1 | iex
& {
    $ErrorActionPreference = 'Stop'
    $ProgressPreference = 'SilentlyContinue'

    function Get-Setting($name, $default) { $value = [Environment]::GetEnvironmentVariable($name); if ($value) { $value } else { $default } }
    $Repo = Get-Setting 'RVD_REPO' 'Strtus/rvd-railbench'
    $Version = Get-Setting 'RVD_VERSION' 'latest'          # latest | vX.Y.Z | source
    $Branch = Get-Setting 'RVD_BRANCH' 'main'
    $Prefix = Get-Setting 'RVD_HOME' (Join-Path $env:LOCALAPPDATA 'rvd-railbench')
    $Data = Get-Setting 'RVDLAB_HOME' (Join-Path $HOME '.rvdlab')
    $BinDir = Join-Path $Prefix 'bin'

    function Say($message) { Write-Host "rvd-railbench: $message" }
    function Fail($message) { throw "rvd-railbench: $message" }
    function Have($command) { [bool](Get-Command $command -ErrorAction SilentlyContinue) }

    if (Have 'py') { $pyExe = 'py'; $pyArgs = @('-3') }
    elseif (Have 'python') { $pyExe = 'python'; $pyArgs = @() }
    else { Fail 'Python 3.10 or newer is required (https://www.python.org/downloads/)' }
    & $pyExe @pyArgs -c 'import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)'
    if ($LASTEXITCODE -ne 0) { Fail 'Python 3.10 or newer is required (https://www.python.org/downloads/)' }

    $tmp = Join-Path ([IO.Path]::GetTempPath()) ('rvd-railbench-' + [guid]::NewGuid().ToString('N'))
    New-Item -ItemType Directory -Path $tmp | Out-Null
    try {
        $stage = Join-Path $tmp 'rvd-railbench'
        $asset = 'rvd-railbench-windows-x64.zip'
        $archive = Join-Path $tmp $asset
        $prebuilt = $false
        if ($Version -ne 'source') {
            $base = if ($Version -eq 'latest') { "https://github.com/$Repo/releases/latest/download" } else { "https://github.com/$Repo/releases/download/$Version" }
            Say "downloading $base/$asset"
            $downloaded = $true
            try { Invoke-WebRequest -Uri "$base/$asset" -OutFile $archive -UseBasicParsing } catch { $downloaded = $false }
            if ($downloaded) {
                $sums = Join-Path $tmp 'SHA256SUMS'
                $haveSums = $true
                try { Invoke-WebRequest -Uri "$base/SHA256SUMS" -OutFile $sums -UseBasicParsing } catch { $haveSums = $false }
                if ($haveSums) {
                    $line = Get-Content $sums | Where-Object { $_ -match " $([regex]::Escape($asset))$" } | Select-Object -First 1
                    $actual = (Get-FileHash $archive -Algorithm SHA256).Hash.ToLowerInvariant()
                    if ($line -and $actual -ne $line.Split(' ')[0]) { Fail "checksum mismatch for $asset" }
                }
                try {
                    Expand-Archive -Path $archive -DestinationPath $tmp -Force
                    & (Join-Path $stage 'build\release\rail_main.exe') --help *> $null
                    $prebuilt = ($LASTEXITCODE -eq 0)
                } catch { $prebuilt = $false }
            }
            if (-not $prebuilt) {
                Say 'no usable prebuilt package, building from source'
                if (Test-Path $stage) { Remove-Item -Recurse -Force $stage }
            }
        }

        if (-not $prebuilt) {
            if (-not (Have 'git')) { Fail 'git is required to build from source' }
            if (-not (Have 'cmake')) { Fail 'CMake 3.16 or newer and a C++17 compiler are required to build from source' }
            $ref = if ($Version -in @('latest', 'source')) { $Branch } else { $Version }
            Say "building $ref from source"
            git clone --quiet --depth 1 --branch $ref "https://github.com/$Repo.git" $stage
            if ($LASTEXITCODE -ne 0) { Fail 'git clone failed' }
            $build = Join-Path $stage 'build\release'
            $generators = @()
            if (Have 'ninja') { $generators += 'Ninja' }
            $generators += ''
            $configured = $false
            foreach ($generator in $generators) {
                if (Test-Path $build) { Remove-Item -Recurse -Force $build }
                $genArgs = if ($generator) { @('-G', $generator) } else { @() }
                cmake -S $stage -B $build @genArgs -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF *> $null
                if ($LASTEXITCODE -eq 0) { $configured = $true; break }
            }
            if (-not $configured) { Fail 'CMake configure failed: install Visual Studio Build Tools (C++), or MinGW-w64 with Ninja' }
            cmake --build $build --config Release --parallel *> $null
            if ($LASTEXITCODE -ne 0) { Fail 'build failed' }
            Remove-Item -Recurse -Force (Join-Path $stage '.git')
        }

        # Profiles and experiments live in $Data, so replacing $Prefix keeps them.
        if (Test-Path $Prefix) { Remove-Item -Recurse -Force $Prefix }
        New-Item -ItemType Directory -Force -Path (Split-Path $Prefix) | Out-Null
        Move-Item $stage $Prefix
    } finally {
        Remove-Item -Recurse -Force $tmp -ErrorAction SilentlyContinue
    }

    & $pyExe @pyArgs -m venv (Join-Path $Prefix '.venv')
    if ($LASTEXITCODE -ne 0) { Fail 'creating a Python venv failed' }
    & (Join-Path $Prefix '.venv\Scripts\python.exe') -m pip install --quiet --disable-pip-version-check (Join-Path $Prefix 'tui')
    if ($LASTEXITCODE -ne 0) { Fail 'installing the terminal interface failed' }

    New-Item -ItemType Directory -Force -Path $BinDir, $Data | Out-Null
    # Default locations are written as %VARIABLES% so the launcher stays ASCII.
    function Portable($path) {
        foreach ($pair in @(@($env:LOCALAPPDATA, '%LOCALAPPDATA%'), @($HOME, '%USERPROFILE%'))) {
            if ($path.StartsWith($pair[0], [StringComparison]::OrdinalIgnoreCase)) { return $pair[1] + $path.Substring($pair[0].Length) }
        }
        $path
    }
    $launcher = @(
        '@echo off'
        "if not defined RVDLAB_PROFILES set `"RVDLAB_PROFILES=$(Portable $Data)\profiles`""
        "if not defined RVDLAB_EXPERIMENTS set `"RVDLAB_EXPERIMENTS=$(Portable $Data)\experiments`""
        "`"$(Portable $Prefix)\.venv\Scripts\rvdlab.exe`" --root `"$(Portable $Prefix)`" %*"
    ) -join "`r`n"
    try { $encoding = [Text.Encoding]::GetEncoding([Globalization.CultureInfo]::CurrentCulture.TextInfo.OEMCodePage) }
    catch { $encoding = [Text.Encoding]::ASCII }
    [IO.File]::WriteAllText((Join-Path $BinDir 'rvdlab.cmd'), $launcher + "`r`n", $encoding)

    $version = (Select-String -Path (Join-Path $Prefix 'CMakeLists.txt') -Pattern 'project\(rvd_railbench VERSION ([0-9.]+)').Matches[0].Groups[1].Value
    Say "installed RVD-RailBench $version in $Prefix ($(if ($prebuilt) { 'prebuilt' } else { 'built from source' }))"
    $userPath = [Environment]::GetEnvironmentVariable('Path', 'User')
    if (@($userPath -split ';') -contains $BinDir) {
        Say 'run: rvdlab'
    } elseif ([Environment]::GetEnvironmentVariable('RVD_NO_MODIFY_PATH')) {
        Say "add $BinDir to PATH, then run: rvdlab"
    } else {
        $parts = @(@($userPath -split ';') | Where-Object { $_ }) + $BinDir
        [Environment]::SetEnvironmentVariable('Path', [string]::Join(';', $parts), 'User')
        $env:Path = "$env:Path;$BinDir"
        Say 'added to PATH (new terminals pick it up); run: rvdlab'
    }
}
