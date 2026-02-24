@echo off
setlocal EnableDelayedExpansion

:: --- KRİTİK DÜZELTME: Scriptin çalıştığı klasöre git ---
cd /d "%~dp0"

title Teknofest 2026 - AKILLI BASLATICI
color 1F
cls

echo ========================================================
echo        TEKNOFEST 2026 - IHA SISTEMI (FIXED)
echo ========================================================
echo.
echo Calisma Dizini: "%~dp0"
echo.

:: --- ADIM 1: NODE.JS KONTROLU ---
echo [1/5] Node.js kontrol ediliyor...
node -v >nul 2>&1
if %errorlevel% neq 0 (
    :: Belki kuruldu ama PATH'e gelmedi, manuel kontrol edelim
    if exist "C:\Program Files\nodejs\node.exe" (
        echo [BILGI] Node.js bulundu ama PATH guncellenmemis. Gecici olarak ekleniyor...
        set "PATH=%PATH%;C:\Program Files\nodejs"
    ) else (
        echo [UYARI] Node.js bulunamadi! Kurulum baslatiliyor...
        if exist "installers\node-setup.msi" (
            echo Lutfen acilan pencerede kuruluma onay verin...
            msiexec /i "installers\node-setup.msi" /passive
            
            echo.
            echo [ONEMLI] Node.js kuruldu. Windows'un bunu algilamasi icin
            echo lutfen bu pencereyi KAPATIP tekrar BASLAT.bat dosyasini calistirin.
            pause
            exit
        ) else (
            echo [HATA] 'installers\node-setup.msi' dosyasi bulunamadi!
            echo Lutfen klasor yapisinin dogru oldugundan emin olun.
            pause
            exit
        )
    )
)
echo [OK] Node.js hazir.
echo.

:: --- ADIM 2: PYTHON KONTROLU ---
echo [2/5] Python kontrol ediliyor...
python --version >nul 2>&1
if %errorlevel% neq 0 (
    :: Belki Python yeni kuruldu, PATH kontrolü
    if exist "%LOCALAPPDATA%\Programs\Python\Python3*\python.exe" (
         echo [BILGI] Python bulundu, PATH'e ekleniyor...
         set "PATH=%PATH%;%LOCALAPPDATA%\Programs\Python\Python310;%LOCALAPPDATA%\Programs\Python\Python311;%LOCALAPPDATA%\Programs\Python\Python312"
    ) else (
        echo [UYARI] Python bulunamadi! Kurulum baslatiliyor...
        if exist "installers\python-setup.exe" (
            echo Python kuruluyor...
            :: Sessiz kurulum ve PATH ekleme
            "installers\python-setup.exe" /quiet InstallAllUsers=1 PrependPath=1 Include_test=0
            
            echo.
            echo [ONEMLI] Python kuruldu. Windows'un bunu algilamasi icin
            echo lutfen bu pencereyi KAPATIP tekrar BASLAT.bat dosyasini calistirin.
            pause
            exit
        ) else (
            echo [HATA] 'installers\python-setup.exe' dosyasi bulunamadi!
            pause
            exit
        )
    )
)
echo [OK] Python hazir.
echo.

:: --- ADIM 3: NODE MODULLERI ---
echo [3/5] Node.js paketleri yukleniyor...
if not exist "node_modules" (
    call npm install express
) else (
    echo [OK] Node modulleri hazir.
)
echo.

:: --- ADIM 4: PYTHON KUTUPHANELERI ---
echo [4/5] Python kutuphaneleri (Numpy, Matplotlib) yukleniyor...
pip install numpy matplotlib scikit-learn
if %errorlevel% neq 0 (
    echo [UYARI] Pip kutuphane yuklemesinde sorun oldu. 
    echo Eger internet yoksa ve daha once yuklediyseniz bu hatayi yoksayin.
)
echo.

:: --- ADIM 5: BASLATMA ---
echo [5/5] Sistem baslatiliyor...
echo.
echo --------------------------------------------------------
echo   TARAYICI ACILIYOR: http://localhost:3000
echo   SUNUCU AKTIF. KAPATMAK ICIN BU PENCEREYI KAPATIN.
echo --------------------------------------------------------

timeout /t 3 >nul
start http://localhost:3000

node server.js
pause