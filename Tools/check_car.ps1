param([string]$Keil = '')
$ErrorActionPreference = 'Stop'
$carRoot = Split-Path -Parent $PSScriptRoot
$carOutput = Join-Path $carRoot 'build'
New-Item -ItemType Directory -Force -Path $carOutput | Out-Null
Push-Location -LiteralPath $carRoot
try {
    if ([string]::IsNullOrWhiteSpace($Keil)) {
        $keilCandidates = @(
            'C:\Keil_v5\UV4\UV4.exe',
            'D:\Keil_v5\UV4\UV4.exe',
            'D:\keil_v5\UV4\UV4.exe',
            'C:\Keil_v5\core\UV4\UV4.exe',
            'D:\Keil_v5\core\UV4\UV4.exe',
            'D:\keil_v5\core\UV4\UV4.exe'
        )
        $Keil = $keilCandidates | Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
    }
    if (!$Keil -or !(Test-Path -LiteralPath $Keil)) {
        $tried = if ($keilCandidates) { " Tried: $($keilCandidates -join ', ')" } else { '' }
        throw "Keil executable missing: '$Keil'.$tried Pass -Keil <path-to-UV4.exe> to override."
    }
    python -m unittest discover -s Tests -p 'test_*.py' -v
    if ($LASTEXITCODE -ne 0) { throw 'Python tests failed' }
    clang -std=c99 -Wall -Wextra -Werror -I CODE Tests/test_car_core.c CODE/Car_Core.c CODE/Car_PID.c CODE/Car_Reference.c -o build/test_car_core.exe
    if ($LASTEXITCODE -ne 0) { throw 'Core test compilation failed' }
    & (Join-Path $carOutput 'test_car_core.exe')
    if ($LASTEXITCODE -ne 0) { throw 'Core tests failed' }
    clang -std=c99 -Wall -Wextra -Werror -I CODE Tests/test_car_pid.c CODE/Car_PID.c CODE/Car_Reference.c -o build/test_car_pid.exe
    if ($LASTEXITCODE -ne 0) { throw 'PID/reference test compilation failed' }
    & (Join-Path $carOutput 'test_car_pid.exe')
    if ($LASTEXITCODE -ne 0) { throw 'PID/reference tests failed' }
    clang -std=c99 -Wall -Wextra -Werror -I CODE Tests/test_car_vision.c CODE/Car_Vision.c -o build/test_car_vision.exe
    if ($LASTEXITCODE -ne 0) { throw 'Vision test compilation failed' }
    & (Join-Path $carOutput 'test_car_vision.exe')
    if ($LASTEXITCODE -ne 0) { throw 'Vision tests failed' }
    $carProject = Join-Path $carRoot 'MDK-ARM/NEUQ_STM32F407VET6_Opensource_Library.uvprojx'
    $carLog = Join-Path $carOutput 'keil_car.log'
    if (!(Test-Path -LiteralPath $Keil)) { throw "Keil executable missing: $Keil" }
    $carProcess = Start-Process -FilePath $Keil -ArgumentList @('-r', ('"{0}"' -f $carProject), '-o', ('"{0}"' -f $carLog)) -WindowStyle Hidden -Wait -PassThru
    Get-Content -LiteralPath $carLog
    if ($carProcess.ExitCode -ne 0) { throw "Keil build failed: $($carProcess.ExitCode)" }
    if (!(Select-String -LiteralPath $carLog -SimpleMatch '0 Error(s), 0 Warning(s).')) { throw 'Keil clean-build summary missing' }
} finally {
    Pop-Location
}
