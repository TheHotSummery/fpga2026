"""Rebuild the existing demonstration floorplan from vector primitives/text.

Render typography directly at the board's 640x480 raster; no resampling of
small antialiased glyphs. Both the SVG source and every text bound are retained.
"""
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont
import json, hashlib, html, struct, argparse
import numpy as np

ap=argparse.ArgumentParser();ap.add_argument('--project');args=ap.parse_args()
P=Path(args.project).resolve() if args.project else None
O=Path(__file__).resolve().parent
O.mkdir(exist_ok=True)
im=Image.new('RGB',(640,480),'#172127');d=ImageDraw.Draw(im)
svg=['<svg xmlns="http://www.w3.org/2000/svg" width="640" height="480" viewBox="0 0 640 480">']
bounds=[]
def rect(box,fill,outline=None,width=1):
 d.rectangle(box,fill=fill,outline=outline,width=width)
 x,y,x1,y1=box;svg.append(f'<rect x="{x}" y="{y}" width="{x1-x+1}" height="{y1-y+1}" fill="{fill}" stroke="{outline or fill}" stroke-width="{width}"/>')
def poly(points,fill,outline=None,width=1):
 d.polygon(points,fill=fill)
 if outline:d.line(points+[points[0]],fill=outline,width=width)
 pts=' '.join(f'{x},{y}' for x,y in points);svg.append(f'<polygon points="{pts}" fill="{fill}" stroke="{outline or fill}" stroke-width="{width}"/>')
def text(s,x,y,size,color,box):
 font=ImageFont.truetype('C:/Windows/Fonts/msyhbd.ttc',size)
 b=font.getbbox(s);w,h=b[2]-b[0],b[3]-b[1]
 assert box[0]<=x and box[1]<=y and x+w<=box[2] and y+h<=box[3],(s,(x,y,w,h),box)
 # Use ink bounds, not the font ascent: the first ink row is exactly y.
 d.text((x-b[0],y-b[1]),s,font=font,fill=color)
 bounds.append({'text':s,'ink_box':[x,y,x+w,y+h],'safe_box':box,'native_font_size':size,'font':'Microsoft YaHei Bold'})
 svg.append(f'<text x="{x}" y="{y+h}" fill="{color}" font-family="Microsoft YaHei" font-weight="700" font-size="{size}">{html.escape(s)}</text>')

rect((0,0,639,47),'#b61119')
text('应急疏散',16,13,26,'#ffffff',(8,6,142,42))
text('EMERGENCY EXIT',153,16,25,'#ffffff',(145,6,630,43))
rect((24,65,249,200),'#263b48','#bfd2dc',3)
rect((249,111,430,157),'#344d5b')
rect((431,65,615,200),'#b61119','#ff9090',3)
rect((24,221,399,359),'#283740','#9cacb7',2)
rect((431,221,615,359),'#007e50','#8be7b5',3)
rect((24,380,615,460),'#b61119')
text('STN-01  站台',40,82,22,'#ffdc45',(32,74,243,111))
d.ellipse((47,119,72,144),fill='#00cb83');svg.append('<circle cx="59.5" cy="131.5" r="12.5" fill="#00cb83"/>')
text('您在这里  YOU',83,122,18,'#ffffff',(77,113,243,150))
text('屏蔽门 / GATE',40,164,18,'#e1eaf0',(32,153,243,191))
text('通道 CONCOURSE',259,125,17,'#ffffff',(254,117,426,151))
rect((406,157,417,296),'#00dc89')
poly([(406,286),(420,286),(420,275),(434,292),(420,309),(420,298),(406,298)],'#00dc89')
text('出口 A',448,87,37,'#ffffff',(439,75,608,128))
text('封闭 CLOSED',448,141,17,'#ffffff',(439,133,608,170))
poly([(529,170),(536,170),(552,189),(545,189)],'#ffffff')
poly([(545,170),(552,170),(536,189),(529,189)],'#ffffff')
text('大厅 HALL',40,239,23,'#ffffff',(32,229,392,271))
poly([(84,319),(143,271),(201,319)],'#283740','#ffdc45',3)
text('禁止进入轨道',87,330,19,'#ffdc45',(32,325,392,354))
text('出口 B',448,243,37,'#ffffff',(439,230,608,285))
text('可走 OPEN',448,295,19,'#ffffff',(439,286,608,319))
poly([(520,328),(568,328),(568,319),(600,335),(568,353),(568,342),(520,342)],'#ffffff')
text('出口 A 封闭，请从出口 B 撤离',40,397,25,'#ffffff',(32,387,608,430))
text('EXIT A CLOSED · USE EXIT B',40,437,16,'#fff1d1',(32,430,608,454))

svg.append('</svg>');(O/'应急疏散_可编辑源.svg').write_text('\n'.join(svg),encoding='utf-8')
im.save(O/'0005.BMP');im.save(O/'应急疏散_预览.png')
data=(O/'0005.BMP').read_bytes();assert struct.unpack_from('<iiHHI',data,18)==(640,480,1,24,0)
assert len(data)==54+640*480*3
if P:(P/'tests/poster_bmp_bytes.hex').write_text(''.join('%02x\n'%v for v in data+bytes((-len(data))%512)),encoding='ascii')
a=np.asarray(im,dtype=np.uint16);q=(a[:,:,0]>>3)<<11|(a[:,:,1]>>2)<<5|(a[:,:,2]>>3)
if P:(P/'tests/poster_mem.hex').write_text(''.join('%04x\n'%int(v) for v in q.ravel()),encoding='ascii')
proof={'pass':True,'size':[640,480],'format':'positive-height 24-bit BI_RGB BMP','file_sha256':hashlib.sha256(data).hexdigest(),
 'method':'native raster bold typography with ink-bounds placement and explicit safe-box checks; no glyph image resizing',
 'all_text_bounds_checked':True,'text':bounds,'same_demo_route_as_original':False,'exit_A':'CLOSED_RED','exit_B':'OPEN_GREEN','TF_card_accessed':False}
(O/'海报检查.json').write_text(json.dumps(proof,ensure_ascii=False,indent=2),encoding='utf-8')
print('V85_POSTER_ASSET_PASS',proof['file_sha256'],len(bounds),'labels')
