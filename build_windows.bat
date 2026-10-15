@echo off
setlocal

cd /d "%~dp0"
python -m pip install -r requirements.txt
if errorlevel 1 exit /b 1

pyinstaller --noconfirm --clean --onefile --windowed ^
  --name ADS1115_Acquisition ^
  src\interfaz.py
if errorlevel 1 exit /b 1

echo.
echo Built: %CD%\dist\ADS1115_Acquisition.exe
endlocal
