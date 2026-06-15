@echo off
setlocal

cd /d "%~dp0"

set "VENV_DIR=%~dp0.venv"
set "VENV_PYTHON=%VENV_DIR%\Scripts\python.exe"
set "ACTIVATE=%VENV_DIR%\Scripts\activate.bat"

echo ========================================
echo  PC Control - setup and run
echo ========================================
echo.

where py >nul 2>nul
if %ERRORLEVEL% EQU 0 (
    set "PY_CMD=py -3"
) else (
    set "PY_CMD=python"
)

if not exist "%VENV_PYTHON%" (
    echo [1/4] Tao moi truong ao .venv...
    %PY_CMD% -m venv "%VENV_DIR%"
    if errorlevel 1 (
        echo.
        echo Loi: Khong tao duoc moi truong ao.
        echo Hay cai Python 3 va dam bao python/py co trong PATH.
        pause
        exit /b 1
    )
) else (
    echo [1/4] Da co moi truong ao .venv.
)

echo [2/4] Kich hoat moi truong ao...
call "%ACTIVATE%"
if errorlevel 1 (
    echo.
    echo Loi: Khong kich hoat duoc moi truong ao.
    pause
    exit /b 1
)

echo [3/4] Cai/cap nhat thu vien...
python -m pip install --upgrade pip
if errorlevel 1 (
    echo.
    echo Loi: Khong cap nhat duoc pip.
    pause
    exit /b 1
)

python -m pip install -r requirements.txt
if errorlevel 1 (
    echo.
    echo Loi: Khong cai duoc thu vien tu requirements.txt.
    pause
    exit /b 1
)

echo.
echo [4/4] Chay main.py...
python main.py
set "APP_EXIT_CODE=%ERRORLEVEL%"

echo.
if not "%APP_EXIT_CODE%"=="0" (
    echo Chuong trinh ket thuc voi ma loi %APP_EXIT_CODE%.
)

pause
exit /b %APP_EXIT_CODE%
