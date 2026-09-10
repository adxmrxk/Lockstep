@echo off
REM Build, then run the full suite including the negative compile tests.
setlocal
set VCVARS="C:\Program Files (x86)\Microsoft Visual Studio\2019\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
call %VCVARS% >nul
cd /d "%~dp0.."
cmake --build build || exit /b 1
ctest --test-dir build --output-on-failure
