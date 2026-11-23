@echo off
rem Build the interactive Windows preview: tools\play\out\moonskim_pc.exe
setlocal
set "HERE=%~dp0"
where cl.exe >nul 2>nul
if not errorlevel 1 goto have_cl
set "VCVARS=C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
if not exist "%VCVARS%" (
  echo [play] ERROR: MSVC not found ^(Visual Studio 2022 Build Tools, C++ workload^)
  exit /b 1
)
call "%VCVARS%" >nul 2>nul
:have_cl
pushd "%HERE%"
if not exist out\obj mkdir out\obj
cl /nologo /std:c++17 /O2 /EHsc /W4 /MP /DJET_SIM=1 /I ..\..\src\core /I ..\..\src ^
   ..\..\src\core\*.cpp ..\sim\synth_imu.cpp play_win.cpp ^
   /Foout\obj\ /Fdout\obj\ /Fe:out\moonskim_pc.exe
if errorlevel 1 (
  echo [play] BUILD FAILED
  popd
  exit /b 1
)
echo [play] OK: "%HERE%out\moonskim_pc.exe"
popd
exit /b 0
