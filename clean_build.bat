@echo off
setlocal

rem ============================================================
rem  NEUQ_STM32F407VET6 project cleanup script
rem  Removes intermediate files generated during compilation
rem  to reduce size when packaging / sharing the project.
rem
rem  Usage: put this file in the project root (same folder as
rem         .gitignore), then double-click it.
rem  It never touches source files (.c .h .s .sct .ioc ...).
rem ============================================================

set "ROOT=%~dp0"

echo.
echo ============================================
echo   NEUQ STM32F407 project cleanup
echo ============================================
echo.

rem --- 1. Build output directories ---
for %%D in (build bin obj out Objects Listings .cmsis) do (
    if exist "%ROOT%MDK-ARM\%%D" (
        echo   [dir ] MDK-ARM\%%D
        rd /s /q "%ROOT%MDK-ARM\%%D"
    )
)

rem --- 2. Keil user GUI layout files (contain local username) ---
if exist "%ROOT%MDK-ARM\*.uvguix.*" (
    echo   [file] MDK-ARM\*.uvguix.*
    del /f /q "%ROOT%MDK-ARM\*.uvguix.*"
)

rem --- 3. Debugger / editor cache files ---
for %%F in (JLinkLog.txt JLinkSettings.ini Thumbs.db Desktop.ini) do (
    if exist "%ROOT%MDK-ARM\%%F" (
        echo   [file] MDK-ARM\%%F
        del /f /q "%ROOT%MDK-ARM\%%F"
    )
)

rem --- 4. Scattered compiler artifacts (recursive, whole project) ---
rem     *.axf / *.elf / *.hex / *.bin are the final firmware images;
rem     remove those four tokens below if you need to keep release binaries.
set "ARTIFACTS=*.o *.d *.crf *.dep *.lst *.map *.axf *.elf *.hex *.bin *.htm *.lnp"
for %%A in (%ARTIFACTS%) do (
    del /f /s /q "%ROOT%*.%%A" 2>nul
)

echo.
echo Cleanup finished.
echo.
pause
