@echo off
chcp 65001 >nul
set PYTHONUTF8=1
if exist "C:\Python314\python.exe" (
  "C:\Python314\python.exe" "%~dp0check_tf.py" F:/
) else (
  py -3 "%~dp0check_tf.py" F:/
)
echo.
echo 只读检查不会修改、删除或格式化TF卡。
echo 报告在本包根目录TF_card_readonly_report.json。若显示拒绝访问，请保留错误报告。
pause
