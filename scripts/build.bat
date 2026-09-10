@echo off
REM Configure and build Lockstep with the MSVC toolchain.
REM Usage:  scripts\build.bat [RelWithDebInfo|Debug|Release]
setlocal

set CFG=%1
if "%CFG%"=="" set CFG=RelWithDebInfo

set VCVARS="C:\Program Files (x86)\Microsoft Visual Studio\2019\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
if not exist %VCVARS% (
  echo [build] vcvars64.bat not found. Edit VCVARS in this script to point at your
  echo         Visual Studio Build Tools installation.
  exit /b 1
)
call %VCVARS% >nul
if errorlevel 1 exit /b 1

cd /d "%~dp0.."
cmake -S . -B build -G "NMake Makefiles" -DCMAKE_BUILD_TYPE=%CFG% || exit /b 1
cmake --build build || exit /b 1
echo [build] ok
