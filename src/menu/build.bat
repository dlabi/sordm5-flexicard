@set ASM="D:\_DATA\Sord\TOOLS\pasmo_assembler\pasmo0.6.0.exe"

%ASM% -v --equ ROM=0 --listing %2.lst %1 %2.bin 
%ASM% -v --equ ROM=1 --listing %2_rom.lst %1 %2.rom

"D:\_DATA\Sord\TOOLS\bin2cas\bin2cas.exe" -s %3 %2.bin %2.cas
"D:\_DATA\Sord\TOOLS\castool\castool.exe" convert sordm5 %2.cas %2.wav


if %ERRORLEVEL% == -1 pause