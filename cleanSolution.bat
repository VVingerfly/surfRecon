del /q ".\x64\Release\*"
FOR /D %%p IN (".\x64\Release\*.*") DO rmdir "%%p" /s /q

del /q ".\x64\Debug\*"
FOR /D %%p IN (".\x64\Debug\*.*") DO rmdir "%%p" /s /q

del /q "surfRecon.VC.db"
del /q ".\bin\surfRecon.pdb"
del /q ".\bin\surfRecon.ipdb"
del /q ".\bin\surfRecon.iobj"
