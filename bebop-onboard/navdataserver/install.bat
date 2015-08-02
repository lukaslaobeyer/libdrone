@echo off
echo Please make sure you are running this from an administrative command prompt.
echo This will install the AutoFlight extensions for full navigation data on your Bebop drone.
echo The script will connect to your drone, upload files and modify critical system configuration potentially bricking your drone.
echo Written on 31/7/2015. Last tested with Bebop firmware 2.0.29.

setlocal
:PROMPT
SET /P AREYOUSURE=Are you sure you want to continue (Y/[N])? 
IF /I "%AREYOUSURE%" NEQ "Y" GOTO END

echo Preparing...
cscript .\windows\preupload.vbs
timeout /t 2 /nobreak
echo Uploading files...
call .\windows\upload.bat
echo Running onboard installation...
cscript .\windows\runinstall.vbs
timeout /t 4 /nobreak
echo Done. Please restart your drone now.

:END

echo Abort.

endlocal
