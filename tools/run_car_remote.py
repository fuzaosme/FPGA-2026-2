#!/usr/bin/env python3
"""Open the monitor; optionally connect the unique Anlogic debug UART.

Connecting requests traffic OFF and never sends a movement command.
"""
import argparse,os,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'02_PC'))

def main():
    p=argparse.ArgumentParser();p.add_argument('--connect-fpga',action='store_true');args=p.parse_args()
    import tkinter as tk
    import serial.tools.list_ports
    from pc_monitor_pro import MonitorApp
    os.chdir(ROOT/'02_PC')
    root=tk.Tk();app=MonitorApp(root)
    if args.connect_fpga:
        def connect():
            ports=[p.device for p in serial.tools.list_ports.comports() if (p.vid,p.pid)==(0x0403,0x6042)]
            if len(ports)==1:
                app.port_var.set(ports[0]);app._connect()
            else:app.log('未唯一发现安路调试串口，请手动选择连接','alarm')
        root.after(200,connect)
    root.mainloop()
if __name__=='__main__':main()
