from pathlib import Path
import hashlib,json,os,subprocess,time
r=Path(__file__).resolve().parents[1];d=r/'work/v95_mcp_proof';q=d;ivl=Path('C:/Users/Administrator/Documents/Codex/ivl67/bin/iverilog.exe');env=os.environ.copy();env['PATH']=str(ivl.parent)+';C:/msys64/ucrt64/bin;'+env.get('PATH','')
model=q/('all_formats_'+str(time.time_ns())+'.vvp');files=[q/'scaler.v',q/'reference.v',q/'bram_sim.v',q/'tb.v']
c=subprocess.run([str(ivl),'-g2012','-s','bilinear_tb','-o',str(model)]+[str(f) for f in files],cwd=r,env=env,capture_output=True,text=True,errors='replace')
(q/'compile.log').write_text(c.stdout+c.stderr,encoding='utf8');assert model.exists() and model.stat().st_size>0,c.stderr
t=subprocess.run([str(ivl.with_name('vvp.exe')),str(model)],cwd=r,env=env,capture_output=True,text=True,errors='replace',timeout=600)
(q/'results.log').write_text(t.stdout+t.stderr,encoding='utf8');print(t.stdout)
assert t.returncode==0 and 'pixels=2150400 failures=0 cycle_exact=1' in t.stdout
assert 'MCP_VALIDITY_RESULT checks=2150400 failures=0 formats=7 min_source_to_capture_cycles=3 constraint_cycles=2' in t.stdout
(q/'evidence.json').write_text(json.dumps({'mcp_capture_checks':2150400,'min_source_to_capture_cycles':3,'constraint_setup_cycles':2,'constraint_hold_cycles':1,'source_register':'p00','target_register':'h0','pixels_compared':2150400,'formats':7,'cycle_exact':True,'same_valid_pause_done_overrun':True,'compiler_returncode':c.returncode,'simulator_returncode':t.returncode,'source_hashes':[{'path':f.relative_to(d).as_posix(),'sha256':hashlib.sha256(f.read_bytes()).hexdigest()} for f in files]},indent=2),encoding='utf8')
