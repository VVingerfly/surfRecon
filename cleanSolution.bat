del /q ".\MiniMeshFrame\Release\*"
FOR /D %%p IN (".\MiniMeshFrame\Release\*.*") DO rmdir "%%p" /s /q

del /q ".\MiniMeshFrame\Debug\*"
FOR /D %%p IN (".\MiniMeshFrame\Debug\*.*") DO rmdir "%%p" /s /q

del /q ".\bin\MiniMeshFrame.pdb"

::FOR /F "tokens=*" %%G IN ('DIR /B /AD /S *debug*') DO RMDIR /S /Q %%G
::FOR /F "tokens=*" %%G IN ('DIR /B /AD /S *release*') DO RMDIR /S /Q %%G
::FOR /F "tokens=*" %%G IN ('DIR /B /S *.ncb*') DO DEL %%G
::FOR /F "tokens=*" %%G IN ('DIR /B /S *.sdf*') DO DEL %%G
::RMDIR ./MiniMeshFrame/Debug /S /Q
:: del C:\DOWNLOAD\*.*

 
 
:: @echo off
::cd /d E:\LiweiFiles\MyProjects\Rib-Shell\Code\MultiMeshFrame\MiniMeshFrame\Debug\
::for /d %%i in (*) do (
::    rd /s /q "%%i"
::    del /a /f /q *.*
::)
::echo delete successed!
::@pause