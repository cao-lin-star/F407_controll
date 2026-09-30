"""Compile and execute the production TOF acceptance logic on the host."""
from pathlib import Path
import re
import subprocess

ROOT=Path(__file__).resolve().parents[1]

def test_invalid_frame_clears_measurement_without_claiming_a_disconnect(tmp_path):
    text=(ROOT/'F407/App/Function/Src/sensor_hub.c').read_text(encoding='utf-8')
    functions=[]
    for name in ('read_u24_le','sort_u32','tofsense_accept_frame'):
        functions.append(re.search(r'static [^\n]+ '+name+r'\(.*?\n}\n',text,re.S).group(0))
    defines='\n'.join(line for line in text.splitlines() if line.startswith('#define TOFSENSE_'))
    code=r'''
#include <stdint.h>
#include <assert.h>
#include <math.h>
#include "board_config.h"
typedef struct { uint8_t frame[400]; uint32_t last_update_ms,last_frame_ms; } tofsense_parser_t;
static struct { uint16_t valid_flags,sensor_fault_flags; } snapshot;
static uint32_t clock_ms;
static uint32_t HAL_GetTick(void) { return clock_ms; }
'''+defines+'\n'+'\n'.join(functions)+r'''
int main(void) {
 tofsense_parser_t p={0}; float distance=0; uint8_t zones=0,count=0; uint32_t frames=0;
 p.frame[8]=16;
 for(int i=0;i<16;++i) { unsigned offset=9+i*6; unsigned value=155000;
  p.frame[offset]=value&255; p.frame[offset+1]=(value>>8)&255;p.frame[offset+2]=(value>>16)&255; }
 clock_ms=100;tofsense_accept_frame(&p,&distance,&zones,&count,&frames,1,8);
 assert(snapshot.valid_flags&1);assert(frames==1 && zones==16);assert(fabsf(distance-.155f)<.000001f);
 for(int i=0;i<16;++i)p.frame[9+i*6+3]=1;
 clock_ms=120;tofsense_accept_frame(&p,&distance,&zones,&count,&frames,1,8);
 assert(!(snapshot.valid_flags&1));assert(snapshot.sensor_fault_flags&8);
 assert(p.last_frame_ms==120 && p.last_update_ms==100 && frames==2);
 for(int i=0;i<16;++i)p.frame[9+i*6+3]=0;
 clock_ms=140;tofsense_accept_frame(&p,&distance,&zones,&count,&frames,1,8);
 assert(snapshot.valid_flags&1);assert(!(snapshot.sensor_fault_flags&8));assert(p.last_update_ms==140);
 return 0;
}
'''
    source=tmp_path/'tof.c';source.write_text(code)
    binary=tmp_path/'tof'
    subprocess.run(['gcc','-DFOOTBATH_HOST_TEST','-std=c99','-Wall','-Wextra','-Werror','-I',str(ROOT/'include'),str(source),'-lm','-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
