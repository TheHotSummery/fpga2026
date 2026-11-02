@echo off
chcp 65001 >nul
set DRIVE=%1
if "%DRIVE%"=="" set DRIVE=G:
python -c "from pathlib import Path; import shutil, sys; root=Path(r'%~dp0'); d=Path(sys.argv[1]+'/'); assert d.exists(), f'Drive {d} not mounted'; src=root/'TF-Audio'/'RAIL.WAV'; dst=d/'RAIL.WAV'; bak=root/'TF-Backup-G'; bak.mkdir(exist_ok=True); shutil.copy2(dst, bak/'RAIL_before_bgm.WAV') if dst.exists() and not (bak/'RAIL_before_bgm.WAV').exists() else None; data=src.read_bytes(); assert len(data)==1920104; f=open(dst,'r+b' if dst.exists() and dst.stat().st_size==1920104 else 'wb'); f.seek(0); f.write(data); f.flush(); f.close(); print('Successfully updated', dst, len(data), 'bytes')" %DRIVE%
if errorlevel 1 pause & exit /b 1
python "%~dp0TF-Check\check_tf.py" %DRIVE%/
pause
