from pathlib import Path
import hashlib,json,os,subprocess,time
r=Path(__file__).resolve().parents[2];q=Path(__file__).resolve().parent;p=r/'outputs/铁路异常恢复优化_V95';ivl=Path('C:/Users/Administrator/Documents/Codex/ivl67/bin/iverilog.exe')
env=os.environ.copy();env['PATH']=str(ivl.parent)+';C:/msys64/ucrt64/bin;'+env.get('PATH','')
files=[p/'src/user_source/hdl_source/SD/sd_card_bmp.v',p/'src/user_source/hdl_source/exhibit_fire_sequence.v',q/'fire_manager_tb.v']
model=q/('fire_'+str(time.time_ns())+'.vvp')
c=subprocess.run([str(ivl),'-g2012','-i','-s','manager_tb','-o',str(model)]+[str(f) for f in files],cwd=r,env=env,capture_output=True,text=True,errors='replace')
(q/'fire_compile.log').write_text(c.stdout+c.stderr,encoding='utf8');assert model.exists() and model.stat().st_size>0,c.stderr
s=subprocess.run([str(ivl.with_name('vvp.exe')),str(model)],cwd=r,env=env,capture_output=True,text=True,errors='replace',timeout=90)
(q/'fire_results.log').write_text(s.stdout+s.stderr,encoding='utf8');print(s.stdout)
assert s.returncode==0 and 'MANAGER_RESULT checks=18 failures=0' in s.stdout
(q/'fire_evidence.json').write_text(json.dumps({'checks':18,'failures':0,'compiler_returncode':c.returncode,'simulator_returncode':s.returncode,'sources':[{'path':f.relative_to(r).as_posix(),'sha256':hashlib.sha256(f.read_bytes()).hexdigest()} for f in files]},indent=2),encoding='utf8')
