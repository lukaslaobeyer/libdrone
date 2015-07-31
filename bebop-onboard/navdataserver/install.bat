echo "This will install the AutoFlight extensions for full navigation data on your Bebop drone."
echo "The script will connect to your drone, upload files and modify critical system configuration potentially bricking your drone."
echo "Written on 31/7/2015. Last tested with Bebop firmware 2.0.29."
@echo off
setlocal
:PROMPT
SET /P AREYOUSURE=Are you sure you want to continue (Y/[N])?
IF /I "%AREYOUSURE%" NEQ "Y" GOTO END

echo "Uploading files..."
./windows/upload.bat
echo "Running onboard installation..."
./windows/runinstall.bat
echo "Done. Please restart your drone now."

:END

echo "Abort."

endlocal
