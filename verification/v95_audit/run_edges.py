from pathlib import Path
import hashlib,json,os,subprocess,sys,time
r=Path(__file__).resolve().parents[2];q=Path(__file__).resolve().parent
label=sys.argv[1];p=r/('outputs/铁路媒体启动恢复与温度同步_V94' if label=='before' else 'outputs/铁路异常恢复优化_V95')
ivl=Path('C:/Users/Administrator/Documents/Codex/ivl67/bin/iverilog.exe');env=os.environ.copy();env['PATH']=str(ivl.parent)+';C:/msys64/ucrt64/bin;'+env.get('PATH','')
model=q/(label+'_'+str(time.time_ns())+'.vvp');source=p/'src/user_source/hdl_source/SD/sd_card_bmp.v'
c=subprocess.run([str(ivl),'-g2012','-i','-s','recovery_edges_tb','-o',str(model),str(source),str(p/'src/user_source/hdl_source/exhibit_fire_sequence.v'),str(q/'recovery_edges_tb.v')],cwd=r,env=env,capture_output=True,text=True,errors='replace')
(q/(label+'_compile.log')).write_text(c.stdout+c.stderr,encoding='utf8');assert model.exists() and model.stat().st_size>0,(c.returncode,c.stdout,c.stderr)
s=subprocess.run([str(ivl.with_name('vvp.exe')),str(model)],cwd=r,env=env,capture_output=True,text=True,errors='replace',timeout=90)
(q/(label+'_results.log')).write_text(s.stdout+s.stderr,encoding='utf8');assert s.returncode==0,s.stderr
(q/(label+'_evidence.json')).write_text(json.dumps({'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'compiler_returncode':c.returncode,'simulator_returncode':s.returncode,'model_created_fresh':True},indent=2),encoding='utf8')
print(s.stdout)
if label!='before':assert 'RECOVERY_EDGES_RESULT checks=20 failures=0' in s.stdout
