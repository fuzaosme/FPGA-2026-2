"""Prepare a relocated TD working copy without changing the frozen RTL."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import tempfile
import time

ROOT=Path(__file__).resolve().parents[1]
SRC=ROOT/'01_FPGA/SQ_anlogic2'


def check_sources(project):
    text=project.read_text(encoding='utf-8-sig')
    refs=re.findall(r'<File\s+Path="([^"]+)"',text)
    missing=[n for n in refs if not (project.parent/n).is_file()]
    if missing:raise ValueError('Missing TD inputs: '+str(missing))
    return text,len(refs)


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--output-dir',type=Path,default=Path(tempfile.gettempdir())/('anlogic_arrow_td_'+time.strftime('%Y%m%d_%H%M%S')))
    ap.add_argument('--dry-run',action='store_true');a=ap.parse_args();out=a.output_dir.resolve()
    if not str(out).isascii():raise SystemExit('Use an ASCII directory, e.g. --output-dir C:\\FPGA\\arrow_work')
    text,count=check_sources(SRC/'td_project/camera_to_dsi_display.al')
    if a.dry_run:
        print(json.dumps(dict(passed=True,source_references=count,output=str(out),dry_run=True),indent=2));return
    if out.exists():raise SystemExit('Output already exists; choose a new directory')
    out.mkdir(parents=True)
    for folder in ('user_source','td_project'):shutil.copytree(str(SRC/folder),str(out/folder))
    text=re.sub(r'(<Project\b[^>]*\bPath=")[^"]*',lambda m:m.group(1)+(out/'td_project').as_posix(),text,count=1)
    project=out/'td_project/camera_to_dsi_display.al'
    project.write_text(text.replace('useDefaultScript="false"','useDefaultScript="true"'),encoding='utf-8')
    check_sources(project)
    print(json.dumps(dict(passed=True,td_project=str(project),source_references=count,
                         changed_only='project Path and useDefaultScript in working copy',compiled=False),indent=2))


if __name__=='__main__':main()
