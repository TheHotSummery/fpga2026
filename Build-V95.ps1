param([string]$Compiler)
Set-StrictMode -Version Latest
$ErrorActionPreference='Stop'
try {
 $voiceManifest=Join-Path $PSScriptRoot 'TF-Audio\audio_manifest.json'
 if(!(Test-Path -LiteralPath $voiceManifest)){throw 'Correct audio 2 and 5 and a READY audio manifest are required before building this release.'}
 $voiceInfo=Get-Content -LiteralPath $voiceManifest -Raw -Encoding UTF8 | ConvertFrom-Json
 if($voiceInfo.status -ne 'READY'){throw 'Audio pack is not READY. Do not use placeholder recordings.'}
 $voiceFile=Join-Path $PSScriptRoot 'TF-Audio\RAIL.WAV'
 if(!(Test-Path -LiteralPath $voiceFile)){throw 'Missing TF-Audio\RAIL.WAV.'}
 if((Get-FileHash -LiteralPath $voiceFile -Algorithm SHA256).Hash -ne $voiceInfo.master_sha256){throw 'RAIL.WAV differs from the matching audio manifest.'}
 $voiceRtl=Join-Path $PSScriptRoot 'src\user_source\hdl_source\rail_announcement_player.v'
 if((Get-FileHash -LiteralPath $voiceRtl -Algorithm SHA256).Hash -ne $voiceInfo.rtl_sha256){throw 'Audio offsets do not match this release. Regenerate the audio and player together.'}
 if(-not $Compiler){$Compiler=$env:V95_TD}
 if(-not $Compiler){
  foreach($cand in @(
   (Join-Path $env:USERPROFILE 'Documents\Codex\td63_runtime\88ECE14\bin\td_commands_prompt.exe'),
   'E:\software\TD_6.2.1_Engineer\bin\td_commands_prompt.exe'
  )){
   if(Test-Path -LiteralPath $cand){$Compiler=$cand;break}
  }
 }
 if(-not $Compiler){$found=Get-Command td_commands_prompt.exe -ErrorAction SilentlyContinue;if($found){$Compiler=$found.Source}}
 if(-not $Compiler){$Compiler=Read-Host 'Full path to TD td_commands_prompt.exe'}
 $Compiler=$Compiler.Trim('"')
 if(-not (Test-Path -LiteralPath $Compiler -PathType Leaf)){throw 'TD compiler was not found.'}
 $project=Join-Path $PSScriptRoot 'src\td_project'
 $rom=Join-Path $PSScriptRoot 'src\user_source\hdl_source\pip_scale_coefficients.hex'
 if(-not (Test-Path -LiteralPath $rom -PathType Leaf)){throw 'Missing source-local scale coefficient ROM. Re-extract the complete package.'}
 $release=Join-Path $PSScriptRoot 'release_v95'
 $null=New-Item -ItemType Directory -Force -Path $release
 foreach($extension in 'bit','bin','rbf'){
  $old=Join-Path $release "Metro_V95.$extension"
  $prev=Join-Path $release "Metro_V95.previous.$extension"
  if((Test-Path -LiteralPath $old) -and -not (Test-Path -LiteralPath $prev)){Move-Item -LiteralPath $old -Destination $prev -Force}
 }
 $env:QT_QPA_PLATFORM='offscreen'
 $log=Join-Path $project 'v95_build.log';$errors=Join-Path $project 'v95_build_errors.log'
 Write-Host "Building V95. Progress log: $log"
 # Own the process handle from Start(), so even an immediately terminating
 # process has a reliable exit code in Windows PowerShell 5.1.
 $proc=New-Object System.Diagnostics.Process
 $proc.StartInfo.FileName=$Compiler;$proc.StartInfo.Arguments='-b run_v95.tcl'
 $proc.StartInfo.WorkingDirectory=$project;$proc.StartInfo.UseShellExecute=$false
 $proc.StartInfo.CreateNoWindow=$true
 $proc.StartInfo.RedirectStandardOutput=$true;$proc.StartInfo.RedirectStandardError=$true
 $outWriter=[IO.StreamWriter]::new($log,$false);$errWriter=[IO.StreamWriter]::new($errors,$false)
 $outWriter.AutoFlush=$true;$errWriter.AutoFlush=$true
 $null=$proc.Start()
 $watch=[Diagnostics.Stopwatch]::StartNew();$lastOutputAt=0.0
 $outRead=$proc.StandardOutput.ReadLineAsync();$errRead=$proc.StandardError.ReadLineAsync()
 $outEnd=$false;$errEnd=$false
 while(-not ($outEnd -and $errEnd)){
  while(-not $outEnd -and $outRead.IsCompleted){
   $line=$outRead.GetAwaiter().GetResult()
   if($null-eq $line){$outEnd=$true}else{$outWriter.WriteLine($line);Write-Host $line;$lastOutputAt=$watch.Elapsed.TotalSeconds;$outRead=$proc.StandardOutput.ReadLineAsync()}
  }
  while(-not $errEnd -and $errRead.IsCompleted){
   $line=$errRead.GetAwaiter().GetResult()
   if($null-eq $line){$errEnd=$true}else{$errWriter.WriteLine($line);Write-Host $line;$lastOutputAt=$watch.Elapsed.TotalSeconds;$errRead=$proc.StandardError.ReadLineAsync()}
  }
  if(-not $proc.HasExited -and ($watch.Elapsed.TotalSeconds-$lastOutputAt)-ge 30){
   $minutes=[int][Math]::Floor($watch.Elapsed.TotalMinutes)
   Write-Host "Build is still running ($minutes min). TD may optimize without new log lines." -ForegroundColor DarkCyan
   $lastOutputAt=$watch.Elapsed.TotalSeconds
  }
  if(-not ($outEnd -and $errEnd)){Start-Sleep -Milliseconds 100}
 }
 $outWriter.Dispose();$errWriter.Dispose()
 $proc.WaitForExit()
 if($proc.ExitCode-ne 0 -or -not (Select-String -LiteralPath $log -SimpleMatch 'V95_BITGEN_COMPLETE' -Quiet) -or -not (Test-Path -LiteralPath (Join-Path $release 'Metro_V95.bit'))){throw 'Build failed. Previous files are named previous; do not use them as this build result.'}
 if(@(Select-String -LiteralPath @($log,$errors) -Pattern 'HDL-5105').Count -gt 0){throw 'Build log reports a missing memory initialization file. Do not download this result.'}
 foreach($name in 'v95_build.log','v95_timing_summary.rpt','v95_synth_area.rpt','v95_routed_area.rpt'){Copy-Item -LiteralPath (Join-Path $project $name) -Destination $release -Force}
 Write-Host 'COMPLETE. Use release_v95\Metro_V95.bit for PROGRAM_SRAM, then PROGRAM_FLASH after board validation.' -ForegroundColor Green
 exit 0
}catch{Write-Host "BUILD FAILED: $_" -ForegroundColor Red;exit 1}
