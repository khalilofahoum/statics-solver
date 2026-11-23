@echo off
setlocal EnableExtensions
title Statics Solver - setup
rem Installs Statics Solver for the current user, downloads the formula fonts (KaTeX) and puts an icon on the Desktop.
rem No administrator rights are needed. To remove it, delete the folder below and the Desktop icon.

if not defined SS_DEST set "SS_DEST=%LOCALAPPDATA%\StaticsSolver"
if not defined SS_NAME set "SS_NAME=Statics Solver"
set "KATEX=https://cdn.jsdelivr.net/npm/katex@0.16.9/dist"
set "FAILED="

if not exist "%~dp0app\index.html" (
  echo.
  echo The "app" folder was not found next to this file.
  echo Extract the whole zip first ^(right-click it, Extract All^), then run install.bat again.
  echo.
  if not defined SS_QUIET pause
  exit /b 1
)

echo.
echo  Statics Solver setup
echo  --------------------
echo  Installing to: %SS_DEST%
echo.

echo [1/3] Copying the program files ...
if not exist "%SS_DEST%" mkdir "%SS_DEST%"
xcopy "%~dp0app" "%SS_DEST%" /E /I /Y /Q >nul
if errorlevel 1 (
  echo       Could not copy the files to "%SS_DEST%".
  if not defined SS_QUIET pause
  exit /b 1
)

echo [2/3] Downloading the formula fonts ^(KaTeX 0.16.9, about 600 KB^) ...
if not exist "%SS_DEST%\vendor\katex\fonts" mkdir "%SS_DEST%\vendor\katex\fonts"
set "HAVE_CURL="
where curl >nul 2>nul && set "HAVE_CURL=1"
call :get "%KATEX%/katex.min.js" "%SS_DEST%\vendor\katex\katex.min.js"
call :get "%KATEX%/katex.min.css" "%SS_DEST%\vendor\katex\katex.min.css"
for %%F in (KaTeX_AMS-Regular.woff2 KaTeX_Caligraphic-Bold.woff2 KaTeX_Caligraphic-Regular.woff2 KaTeX_Fraktur-Bold.woff2 KaTeX_Fraktur-Regular.woff2 KaTeX_Main-Bold.woff2 KaTeX_Main-BoldItalic.woff2 KaTeX_Main-Italic.woff2 KaTeX_Main-Regular.woff2 KaTeX_Math-BoldItalic.woff2 KaTeX_Math-Italic.woff2 KaTeX_SansSerif-Bold.woff2 KaTeX_SansSerif-Italic.woff2 KaTeX_SansSerif-Regular.woff2 KaTeX_Script-Regular.woff2 KaTeX_Size1-Regular.woff2 KaTeX_Size2-Regular.woff2 KaTeX_Size3-Regular.woff2 KaTeX_Size4-Regular.woff2 KaTeX_Typewriter-Regular.woff2) do call :get "%KATEX%/fonts/%%F" "%SS_DEST%\vendor\katex\fonts\%%F"
if defined FAILED (
  echo       Some files could not be downloaded ^(no internet connection?^).
  echo       The program still works; formulas are shown as plain text until you run install.bat again online.
) else (
  echo       Done.
)

echo [3/3] Creating the Desktop icon ...
powershell -NoProfile -ExecutionPolicy Bypass -Command "$d=[Environment]::GetFolderPath('Desktop'); $s=(New-Object -ComObject WScript.Shell).CreateShortcut((Join-Path $d ($env:SS_NAME + '.lnk'))); $s.TargetPath=(Join-Path $env:SS_DEST 'index.html'); $s.WorkingDirectory=$env:SS_DEST; $s.IconLocation=(Join-Path $env:SS_DEST 'icon.ico'); $s.Description='Statics Solver - beams, frames and trusses'; $s.Save()"
if errorlevel 1 (
  echo       Could not create the Desktop icon. Open "%SS_DEST%\index.html" directly instead.
) else (
  echo       Done.
)

echo.
echo  Finished. Double-click "%SS_NAME%" on your Desktop to start.
echo.
if not defined SS_QUIET (
  start "" "%SS_DEST%\index.html"
  pause
)
exit /b 0

:get
rem %1 = address, %2 = file to write
if defined HAVE_CURL (
  curl -fsSL --retry 2 -o "%~2" "%~1"
) else (
  powershell -NoProfile -ExecutionPolicy Bypass -Command "try { [Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12; (New-Object Net.WebClient).DownloadFile('%~1', '%~2') } catch { exit 1 }"
)
if errorlevel 1 (
  set "FAILED=1"
  if exist "%~2" del "%~2"
)
exit /b 0
