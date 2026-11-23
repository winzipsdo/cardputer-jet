@echo off
rem ===========================================================================
rem  MOONSKIM PC simulator build (DESIGN.md section 18). OWNER: group E.
rem
rem  usage: build.bat [debug] [wx] [test] [clean]
rem    (no args)  release build: /O2 /W4 -> tools\sim\out\jetsim.exe
rem    debug      /Od /Zi instead of /O2
rem    wx         treat warnings as errors (/WX)
rem    test       after building: --selftest, --model, every scripts\*.jsc, the determinism
rem               check (determinism.jsc twice with --hash, hashes compared) and --bench 600
rem    clean      delete tools\sim\out and exit
rem
rem  Works from any current directory and with spaces in the project path (all compiler
rem  paths are relative to this folder). Uses the MSVC already on PATH if there is one,
rem  otherwise VS 2022 BuildTools vcvars64.bat, otherwise vswhere.
rem ===========================================================================
setlocal
rem %~dp0 must be captured before any "shift" (shift also moves %0)
set "HERE=%~dp0"

set "OPT=/O2"
set "WX="
set "DO_TEST=0"
set "DO_CLEAN=0"
:args
if "%~1"=="" goto args_done
if /i "%~1"=="debug" set "OPT=/Od /Zi"
if /i "%~1"=="wx" set "WX=/WX"
if /i "%~1"=="test" set "DO_TEST=1"
if /i "%~1"=="clean" set "DO_CLEAN=1"
shift
goto args
:args_done

rem ---- MSVC environment (before pushd: vcvars may change the directory)
where cl.exe >nul 2>nul
if not errorlevel 1 goto have_cl
set "VCVARS=C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
if exist "%VCVARS%" goto call_vcvars
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" goto no_msvc
set "VSINST="
for /f "usebackq delims=" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSINST=%%i"
if not defined VSINST goto no_msvc
set "VCVARS=%VSINST%\VC\Auxiliary\Build\vcvars64.bat"
if not exist "%VCVARS%" goto no_msvc
:call_vcvars
call "%VCVARS%" >nul 2>nul
where cl.exe >nul 2>nul
if errorlevel 1 goto no_msvc
:have_cl

pushd "%HERE%"
if errorlevel 1 (
  echo [build] ERROR: cannot enter the simulator folder "%HERE%"
  exit /b 1
)

if "%DO_CLEAN%"=="1" (
  if exist out rmdir /s /q out
  echo [build] cleaned "%HERE%out"
  popd
  exit /b 0
)

if not exist ..\..\src\core\*.cpp (
  echo [build] ERROR: no src\core\*.cpp found next to "%HERE%..\..\src\core"
  popd
  exit /b 1
)
if not exist out\obj mkdir out\obj
if errorlevel 1 (
  echo [build] ERROR: cannot create "%HERE%out\obj"
  popd
  exit /b 1
)

echo [build] compiling src\core\*.cpp + tools\sim\*.cpp  (%OPT% /W4 %WX%)
cl /nologo /std:c++17 %OPT% /EHsc /W4 %WX% /MP /DJET_SIM=1 /I ..\..\src\core /I ..\..\src ^
   ..\..\src\core\*.cpp sim_main.cpp script.cpp synth_imu.cpp png_write.cpp wav_write.cpp ^
   /Foout\obj\ /Fdout\obj\ /Fe:out\jetsim.exe
if errorlevel 1 (
  echo.
  echo [build] FAILED - see the compiler messages above.
  popd
  exit /b 1
)
echo.
echo [build] OK: "%HERE%out\jetsim.exe"
echo [build] run:  out\jetsim.exe scripts\night_cruise.jsc     (shots -^> "%HERE%out\night_cruise\")
echo [build]       out\jetsim.exe --selftest ^| --model ^| --bench 600 ^| --help

if not "%DO_TEST%"=="1" goto done

rem =========================================================================== test
set /a FAILS=0
set /a RUNS=0
echo.
echo [test] selftest
out\jetsim.exe --selftest > out\selftest.log 2>&1
call :result selftest
echo [test] model
out\jetsim.exe --model > out\model.log 2>&1
call :result model
for %%s in (scripts\*.jsc) do call :runscript "%%s"
call :determinism
echo [test] bench 600
out\jetsim.exe --bench 600 --quiet > out\bench.log 2>&1
call :result bench
echo.
echo [test] %RUNS% runs, %FAILS% failed. Logs: "%HERE%out\*.log", images: "%HERE%out\<script>\"
popd
if %FAILS% gtr 0 exit /b 1
exit /b 0

:runscript
if /i "%~n1"=="determinism" goto :eof
echo [test] %~n1
out\jetsim.exe "%~1" --wav --quiet > "out\%~n1.log" 2>&1
call :result %~n1
goto :eof

:determinism
echo [test] determinism (twice, --hash)
out\jetsim.exe scripts\determinism.jsc --hash --quiet --out out\det_a > out\determinism_a.log 2>&1
call :result determinism_a
out\jetsim.exe scripts\determinism.jsc --hash --quiet --out out\det_b > out\determinism_b.log 2>&1
call :result determinism_b
findstr /l /c:"[HASH]" out\determinism_a.log > out\det_a.hash
findstr /l /c:"[HASH]" out\determinism_b.log > out\det_b.hash
set /a RUNS+=1
fc /b out\det_a.hash out\det_b.hash >nul 2>nul
if errorlevel 1 (
  set /a FAILS+=1
  echo [test]   FAIL determinism: hashes differ ^(out\det_a.hash vs out\det_b.hash^)
) else (
  echo [test]   ok   determinism: identical hashes
)
goto :eof

:result
set "RC=%errorlevel%"
set /a RUNS+=1
if not "%RC%"=="0" (
  set /a FAILS+=1
  echo [test]   FAIL %1  ^(see out\%1.log^)
) else (
  echo [test]   ok   %1
)
goto :eof

:done
popd
exit /b 0

:no_msvc
echo [build] ERROR: MSVC not found. Install "Visual Studio 2022 Build Tools" (C++ workload)
echo [build]        or run this from a "x64 Native Tools Command Prompt".
exit /b 1
