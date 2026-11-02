from pathlib import Path
import hashlib,json,os,subprocess,time
r=Path(__file__).resolve().parents[2];q=Path(__file__).resolve().parent;p=r/'outputs/铁路异常恢复优化_V95'
ivl=Path('C:/Users/Administrator/Documents/Codex/ivl67/bin/iverilog.exe');env=os.environ.copy();env['PATH']=str(ivl.parent)+';C:/msys64/ucrt64/bin;'+env.get('PATH','')
source=p/'src/user_source/hdl_source/SD/sd_card_bmp.v';seq=p/'src/user_source/hdl_source/exhibit_fire_sequence.v'
recipes=[('audio_recovery_integration',q/'audio_recovery_integration_tb.v','audio_recovery_integration_tb',8,[]),
 ('startup',r/'work/v94_flow_audit/startup_recovery_tb.v','startup_recovery_tb',17,[]),
 ('writer_integrity',q/'writer_integrity_tb.v','tb_abort_integrity',10,[p/'src/user_source/hdl_source/SD/frame_fifo_write.v'])]
evidence=[]
for name,tb,top,count,extra in recipes:
 model=q/(name+'_'+str(time.time_ns())+'.vvp')
 files=[source,seq,tb]+extra
 c=subprocess.run([str(ivl),'-g2012','-i','-s',top,'-o',str(model)]+[str(f) for f in files],cwd=r,env=env,capture_output=True,text=True,errors='replace')
 (q/(name+'_compile.log')).write_text(c.stdout+c.stderr,encoding='utf8');assert model.exists() and model.stat().st_size>0,(name,c.returncode,c.stderr)
 s=subprocess.run([str(ivl.with_name('vvp.exe')),str(model)],cwd=r,env=env,capture_output=True,text=True,errors='replace',timeout=90)
 (q/(name+'_results.log')).write_text(s.stdout+s.stderr,encoding='utf8')
 print(s.stdout,flush=True)
 assert s.returncode==0 and f'RESULT checks={count} failures=0' in s.stdout,(name,s.returncode,s.stderr)
 evidence.append({'test':name,'checks':count,'failures':0,'compiler_returncode':c.returncode,'simulator_returncode':s.returncode,'sources':[{'path':f.relative_to(r).as_posix(),'sha256':hashlib.sha256(f.read_bytes()).hexdigest()} for f in files]})
(q/'integration_evidence.json').write_text(json.dumps(evidence,indent=2),encoding='utf8')
