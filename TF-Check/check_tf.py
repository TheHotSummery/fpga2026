from pathlib import Path
import ctypes as c, ctypes.wintypes as w, struct,hashlib,json,sys,datetime
ROOT=Path(__file__).resolve().parents[1]
REPORT=ROOT/'TF_card_readonly_report.json'
k=c.WinDLL('kernel32',use_last_error=True)
k.CreateFileW.argtypes=[w.LPCWSTR,w.DWORD,w.DWORD,w.LPVOID,w.DWORD,w.DWORD,w.HANDLE];k.CreateFileW.restype=w.HANDLE
k.DeviceIoControl.argtypes=[w.HANDLE,w.DWORD,w.LPVOID,w.DWORD,w.LPVOID,w.DWORD,c.POINTER(w.DWORD),w.LPVOID];k.DeviceIoControl.restype=w.BOOL
k.CloseHandle.argtypes=[w.HANDLE]
def extents(p):
 h=k.CreateFileW(str(p),0x80000000,7,None,3,0,None)
 if h in (None,c.c_void_p(-1).value): return {'status':'UNKNOWN','error':c.get_last_error()}
 try:
  start=c.c_longlong(0);buf=c.create_string_buffer(65536);used=w.DWORD()
  ok=k.DeviceIoControl(h,0x90073,c.byref(start),8,buf,len(buf),c.byref(used),None)
  if not ok:return {'status':'UNKNOWN','error':c.get_last_error()}
  n=struct.unpack_from('<I',buf.raw)[0];vcn=struct.unpack_from('<q',buf.raw,8)[0];parts=[]
  for i in range(n):
   end,lcn=struct.unpack_from('<qq',buf.raw,16+16*i);parts.append({'vcn':vcn,'clusters':end-vcn,'lcn':lcn});vcn=end
  contiguous=bool(parts) and all(x['lcn']>=0 for x in parts) and all(a['lcn']+a['clusters']==b['lcn'] for a,b in zip(parts,parts[1:]))
  return {'status':'CONTIGUOUS' if contiguous else 'FRAGMENTED','extents':parts}
 finally:k.CloseHandle(h)
def main():
 drive=Path(sys.argv[1] if len(sys.argv)>1 else 'F:/')
 report={'timestamp':datetime.datetime.now().isoformat(),'drive':str(drive),'read_only':True,'files':[],'errors':[]}
 try:
  paths=list(drive.iterdir());report['root_names']=[x.name for x in paths]
  for p in paths:
   if not p.is_file() or p.suffix.lower() not in ['.bmp','.pv4','.wav']:continue
   try:
    data=p.read_bytes();item={'name':p.name,'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest(),'allocation':extents(p)}
    if data[:4]==b'PVF4':item['video']={'version':data[4],'width':data[5]+(data[13]<<8),'height':data[6]+(data[14]<<8),'frames':int.from_bytes(data[8:10],'little'),'display_frames_per_source_frame':data[7]}
    if data[:2]==b'BM':item['bmp']={'width':struct.unpack_from('<i',data,18)[0],'height':struct.unpack_from('<i',data,22)[0],'bpp':struct.unpack_from('<H',data,28)[0]}
    report['files'].append(item)
   except Exception as e:report['errors'].append({'file':p.name,'error':str(e)})
  report['expected_pip0_sha256']='8d6c8ef820988bdb59fb7f1eefa2e2733a0227a80bce37735da6f82bcadbe619'
  report['expected_rail_wav_sha256']=json.loads((ROOT/'TF-Audio/audio_manifest.json').read_text(encoding='utf-8'))['master_sha256']
  if (ROOT/'TF-Fire/poster_manifest.json').exists():report['expected_posters']=json.loads((ROOT/'TF-Fire/poster_manifest.json').read_text(encoding='utf-8'))['files']
 except Exception as e:report['errors'].append({'drive':str(drive),'error':str(e)})
 REPORT.parent.mkdir(parents=True,exist_ok=True);REPORT.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
 print('Read-only report saved:',REPORT)
 for f in report['files']:print(f['name'],f['bytes'],f['allocation']['status'])
 for e in report['errors']:print('ERROR:',e)
if __name__=='__main__':main()
