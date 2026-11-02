from pathlib import Path
import hashlib,json,os,shutil,subprocess,sys,time
r=Path(__file__).resolve().parents[2];q=Path(__file__).resolve().parent;p=r/'outputs/铁路异常恢复优化_V95';before=len(sys.argv)>1 and sys.argv[1]=='before';label='early_before' if before else 'early_after'
s=(q/'audio_recovery_integration_tb.v').read_text(encoding='utf8').replace('module audio_recovery_integration_tb;','module early_ir_tb;').replace('ir=1,audio_done=0','ir=0,audio_done=0')
s=s.replace('else if(pos==8)begin wav_found=1;pos=pos+1;end','else if(pos==8)begin if(wav_catalog_complete_fixture)wav_found=1;pos=pos+1;end')
s=s.replace('integer checks=0,failures=0,','integer owner_overlap=0;\nalways @(posedge clk)if(!rst && d.load_busy && (d.wav_load_busy || d.wav_phase2_busy))owner_overlap=owner_overlap+1;\ninteger checks=0,failures=0,')
s=s.replace(' $display("AUDIO_RECOVERY_INTEGRATION_RESULT', ' check(d.train_flow.use_audio,"cold occupied sensor waits for audio availability before selecting the spoken journey");\n check(owner_overlap==0,"picture, audio and resumed catalog never own the shared load path together");\n $display("EARLY_IR_RESULT')
tb=q/'early_ir_tb.v';tb.write_text(s,encoding='utf8')
source=r/'work/v95_before_startup_ownership/sd_card_bmp.v' if before else p/'src/user_source/hdl_source/SD/sd_card_bmp.v'
ivl=Path('C:/Users/Administrator/Documents/Codex/ivl67/bin/iverilog.exe');env=os.environ.copy();env['PATH']=str(ivl.parent)+';C:/msys64/ucrt64/bin;'+env.get('PATH','')
model=q/(label+'_'+str(time.time_ns())+'.vvp')
c=subprocess.run([str(ivl),'-g2012','-i','-s','early_ir_tb','-o',str(model),str(source),str(p/'src/user_source/hdl_source/exhibit_fire_sequence.v'),str(tb)],cwd=r,env=env,capture_output=True,text=True,errors='replace')
assert model.exists() and model.stat().st_size>0,c.stderr
t=subprocess.run([str(ivl.with_name('vvp.exe')),str(model)],cwd=r,env=env,capture_output=True,text=True,errors='replace',timeout=90)
(q/(label+'_results.log')).write_text(t.stdout+t.stderr,encoding='utf8');print(t.stdout)
if before:
 assert t.returncode==1 and 'failures=0' not in t.stdout
 shutil.copy2(source,q/'before_startup_ownership_manager.v')
else:assert t.returncode==0 and 'EARLY_IR_RESULT checks=10 failures=0' in t.stdout
(q/(label+'_evidence.json')).write_text(json.dumps({'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'test_sha256':hashlib.sha256(tb.read_bytes()).hexdigest(),'compiler_returncode':c.returncode,'simulator_returncode':t.returncode,'scope':'Actual manager with card/parser/memory completion fixtures; card initially reports no WAV, sensor occupied before initialization.'},indent=2),encoding='utf8')
