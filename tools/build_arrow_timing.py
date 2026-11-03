"""Snapshot TD build with the official TimingHigh place/route strategy.

Same flow and gates as build_arrow.py, but for this near-full design the
60 MHz ISP path (u_awb -> sign_detect) is placement-sensitive: F99D73AC closed
it at +0.055 ns and the first event-cooldown build landed at -1.692 ns on the
same unchanged ISP logic. This variant keeps the AreaOptimizeHigh mapping that
makes the design fit, then applies the vendor TimingHigh placement and
post-route optimisation parameters. Never programs hardware.

New results default to runtime/, preserving the historical handoff evidence.
Full builds publish a bit only after timing passes; --synth-only is not timing
acceptance and does not produce a downloadable bit.
"""
import argparse
import os
import ctypes
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import time
import sys
sys.stdout.reconfigure(encoding='utf-8')
ROOT=Path(__file__).resolve().parents[1]
SRC=ROOT/'01_FPGA/SQ_anlogic2'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--synth-only',action='store_true');ap.add_argument('--name',default='arrow_'+time.strftime('%Y%m%d_%H%M%S'))
    ap.add_argument('--bit-name',default='arrow_rebuilt_'+time.strftime('%Y%m%d_%H%M%S')+'.bit')
    ap.add_argument('--td',help='Path to Linux td.sh or Windows td_commands_prompt.exe')
    ap.add_argument('--build-root',type=Path,default=Path(tempfile.gettempdir())/'anlogic_arrow_builds')
    ap.add_argument('--out',type=Path,help='New evidence directory (must not exist)')
    ap.add_argument('--bit-dir',type=Path,help='Accepted bit output directory (default: evidence directory)')
    ap.add_argument('--area-high',action='store_true',help='TD official AreaOptimizeHigh mapping parameters; routed timing still required')
    a=ap.parse_args()
    candidate=a.td or os.environ.get('TD_COMMANDS_PROMPT') or shutil.which('td_commands_prompt.exe')
    if not candidate and os.name!='nt':
        td_root=Path(os.environ.get('TD_INSTALL_DIR','/opt/Anlogic/TD'))
        candidate=shutil.which('td.sh') or (str(td_root/'bin/td.sh') if (td_root/'bin/td.sh').is_file() else None)
    if not candidate:
        legacy=Path(r'D:\Anlogic\TD_6.2.1_Engineer_6.2.168.116\bin\td_commands_prompt.exe')
        if legacy.is_file():candidate=str(legacy)
    if not candidate or not Path(candidate).is_file():raise SystemExit('Pass --td with the installed TD command executable')
    TD=Path(candidate)
    if not re.fullmatch(r'[A-Za-z0-9_]{1,60}',a.name):raise SystemExit('ASCII name required')
    if not re.fullmatch(r'[A-Za-z0-9_]{1,60}\.bit',a.bit_name):raise SystemExit('ASCII bit filename required')
    b=a.build_root.resolve()/a.name;b.mkdir(parents=True,exist_ok=False)
    manifest={p.relative_to(SRC).as_posix():sha(p) for d in ('user_source','td_project') for p in (SRC/d).rglob('*') if p.is_file()}
    (b/'source_manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    for d in ('user_source','td_project'):shutil.copytree(str(SRC/d),str(b/d))
    build_id=hashlib.sha256(json.dumps(manifest,sort_keys=True).encode()).hexdigest()[:8]
    top_snapshot=b/'user_source/hdl_source/design_top_wrapper.v'
    top_bytes=top_snapshot.read_bytes()
    top_bytes=re.sub(rb"(localparam \[31:0\] IQ_BUILD_ID=32'h)[0-9a-fA-F]{8}",
                     lambda m:m.group(1)+build_id.encode(),top_bytes)
    top_snapshot.write_bytes(top_bytes)
    if os.name=='nt':
        buf=ctypes.create_unicode_buffer(32768)
        if not ctypes.windll.kernel32.GetShortPathNameW(str(b),buf,len(buf)):raise SystemExit('No short path')
        alias=Path(buf.value)
    else:
        alias=b
    if not str(alias).isascii():raise SystemExit('ASCII build path needed')
    al=b/'td_project/camera_to_dsi_display.al';s=al.read_text(encoding='utf-8-sig')
    s=re.sub(r'(<Project\b[^>]*\bPath=")[^"]*',lambda m:m.group(1)+(alias/'td_project').as_posix(),s,count=1)
    al.write_text(s.replace('useDefaultScript="false"','useDefaultScript="true"'),encoding='utf-8')
    area_params=['set_param gate add_map_lut_struct high','set_param gate map_mode area_prior',
                 'set_param gate multi_thread_mapping partition','set_param rtl resource_sharing auto',
                 'set_param rtl shifter_style shifter_reg'] if a.area_high else []
    place_params=['set_param place effort high','set_param place opt_timing high',
                  'set_param place logic_level_opt on','set_param place aggressive_opt 20',
                  'set_param place aggr_neighbor_dist 5','set_param place enhanced_beeline_opt 3',
                  'set_param place timing_driven_mode on','set_param place shifter_opt on',
                  'set_param place crit_path_opt on','set_param place post_place_opt high',
                  'set_param place split_data_from_clock on']
    route_params=['set_param route post_route_opt high']
    cmds=['import_device ph1_35p.db -package PH1P35MDG324 -speed 3','cd td_project','open_project camera_to_dsi_display.al -single_run','cd ../direct_run']+area_params+[
      'set phase elaborate','elaborate -top design_top_wrapper','read_adc ../user_source/constraints_source/pin.adc',
      'read_ip_adc -ip ddr2 -file ../user_source/hdl_source/ph1p35_ddr/ddr2/src/adc/ddr2.adc','read_sdc ../td_project/camera_to_dsi_display.sdc',
      'read_sdc -ip ddr2 ../user_source/hdl_source/ph1p35_ddr/ddr2/src/sdc/ddr2.sdc',
      'set phase rtl','insert_debugger','optimize_rtl','report_area -file rtl.area','set phase gate','optimize_gate -packarea gate.area','legalize_phy_inst','export_db gate.db']
    if not a.synth_only:cmds+=place_params+['set phase place','place','update_timing -mode manhattan','report_timing_summary -file place.timing']+route_params+['set phase route','route',
      'report_area -io_info -file routed.area','update_timing -mode final','report_timing_status -file routed.status','report_timing_summary -file routed.timing',
      'report_timing_exception -file exceptions.timing','report_clock_utilization -file clock.txt','export_db routed.db','set phase bitgen','bitgen -bit arrow_v1.bit -compress']
    flow='set fh [open status.txt w]\nset phase import\nset rc [catch {\n'+'\n'.join(cmds)+'\nset phase completed\n} result]\nputs $fh "phase=$phase"\nputs $fh "catch_code=$rc"\nputs $fh $result\nclose $fh\nif {$rc} {puts stderr $result;exit 1}\nputs "ARROW BUILD COMPLETED"\nexit\n'
    (b/'run.tcl').write_text(flow,encoding='utf-8');(b/'direct_run').mkdir()
    evidence=(a.out or ROOT/'03_Verification/runtime/td_builds'/a.name).resolve()
    evidence.mkdir(parents=True,exist_ok=False)
    (evidence/'build_location.txt').write_text(str(b),encoding='utf-8')
    print('Frozen TD build: '+str(b),flush=True)
    with (b/'td.log').open('wb') as log:
        command=[str(TD)]+([] if os.name=='nt' else ['-b'])+[(alias/'run.tcl').as_posix()]
        env=dict(os.environ)
        if os.name!='nt':env.setdefault('QT_QPA_PLATFORM','offscreen')
        r=subprocess.run(command,cwd=str(alias),env=env,stdout=log,stderr=subprocess.STDOUT)
    status=(b/'status.txt').read_text(encoding='utf-8') if (b/'status.txt').exists() else ''
    log=(b/'td.log').read_text(encoding='utf-8',errors='replace')
    ready_driven=not re.search(r'SYN-5013 WARNING:[^\n]*u_isp_top/O_tready',log)
    passed=r.returncode==0 and 'phase=completed' in status and 'catch_code=0' in status and 'ARROW BUILD COMPLETED' in log and not re.search(r'\bERROR:',log) and ready_driven
    report=dict(passed=passed,programmed=False,build=str(b),synth_only=a.synth_only,
                build_id=build_id,td_executable=str(TD),command=command,source_manifest=manifest,isp_ready_not_floating=ready_driven,
                synthesis_strategy='AreaOptimizeHigh+TimingHigh' if a.area_high else 'TimingHigh',
                strategy_commands=area_params+place_params+route_params,
                build_script_sha256=sha(Path(__file__)))
    if passed and not a.synth_only:
        timing=(b/'direct_run/routed.timing').read_text(encoding='utf-8')
        report['timing_ns']={k:float(re.search(k+r':\s*([-\d.]+)ns',timing).group(1)) for k in ('SWNS','HWNS','STNS','HTNS')}
        t=report['timing_ns'];report['passed']=t['SWNS']>=0 and t['HWNS']>=0 and t['STNS']==0 and t['HTNS']==0
        area=(b/'direct_run/routed.area').read_text(encoding='utf-8')
        report['resources']={k:int(re.search(r'^#'+k+r'\s+(\d+)',area,re.M).group(1)) for k in ('slice','lut6','reg','eram','dsp')}
        if report['passed']:
            bit_dir=(a.bit_dir or evidence).resolve();bit_dir.mkdir(parents=True,exist_ok=True)
            bit=bit_dir/a.bit_name
            if bit.exists():raise RuntimeError('Refusing to overwrite existing bit: '+str(bit))
            shutil.copy2(str(b/'direct_run/arrow_v1.bit'),str(bit))
            report['bit_path']=str(bit)
            if ROOT in bit.parents:report['bit_relative_path']=bit.relative_to(ROOT).as_posix()
            report['bit_sha256']=sha(bit);report['bit_bytes']=bit.stat().st_size
    for name in ('td.log','run.tcl','status.txt','source_manifest.json'):
        if (b/name).exists():shutil.copy2(str(b/name),str(evidence/name))
    for p in (b/'direct_run').glob('*'):
        if p.suffix in ('.area','.timing','.status','.txt'):shutil.copy2(str(p),str(evidence/p.name))
    (evidence/'result.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k!='source_manifest'},indent=2),flush=True)
    if not report['passed']:raise SystemExit('TD build or timing failed; no accepted bit published')

if __name__=='__main__':main()
