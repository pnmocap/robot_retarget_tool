@echo off
setlocal
set BUILD_DIR=build
set CONFIG=Release
set GENERATOR="Visual Studio 17 2022"
set ARCH=x64

echo [+] Configuring with %GENERATOR% (%ARCH%)...
cmake -S . -B %BUILD_DIR% -G %GENERATOR% -A %ARCH% || goto :error

echo [+] Building %CONFIG%...
cmake --build %BUILD_DIR% --config %CONFIG% || goto :error

echo [+] Build succeeded.
goto :eof

:error
echo [!] Build failed.
exit /b 1
