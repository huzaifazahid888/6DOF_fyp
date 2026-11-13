#!/usr/bin/env python3
"""Optional real Jazzy/Harmonic test. Requires graphics or headless EGL support.

This is deliberately not run by colcon's CPU-only unit tests. CI and developers
can invoke it explicitly. A pass means actual sensor streams and six physical
attach/carry/release cycles, not just Python syntax or a mocked action result.
"""
import os
import csv
from collections import Counter
from pathlib import Path
import signal
import subprocess
from ament_index_python.packages import get_package_share_directory

launch_log=Path('/tmp/pieas_factory_launch.log')
sort_log=Path('/tmp/pieas_factory_sort.log')
config=Path(get_package_share_directory('pieas_sorting'))/'config/cell.yaml'
with launch_log.open('w') as output:
    process=subprocess.Popen(['ros2','launch','pieas_sorting','factory.launch.py',
                              'rviz:=false','gui:=false'],stdout=output,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        subprocess.run(['ros2','run','pieas_sorting','check_cell','--ros-args','-p','use_sim_time:=true',
                        '-p','timeout:=90.0'],check=True,timeout=100)
        with sort_log.open('w') as result:
            subprocess.run(['ros2','run','pieas_sorting','sort_batch','--ros-args','-p','use_sim_time:=true',
                            '-p',f'config:={config}','-p','execute:=true','-p','cycles:=6',
                            '-p','log_directory:=/tmp/pieas_factory_batch'],stdout=result,stderr=subprocess.STDOUT,
                           check=True,timeout=2400)
        assert 'FACTORY: COMPLETE' in sort_log.read_text()
        records=list(Path('/tmp/pieas_factory_batch').glob('batch_*.csv'))
        assert len(records)==1, records
        with records[0].open() as stream:rows=list(csv.DictReader(stream))
        assert len(rows)==6 and all(row['outcome']=='verified_in_bin' for row in rows), rows
        assert Counter(row['shape'] for row in rows)=={'cube':2,'rectangle':2,'cylinder':2}, rows
        print('PASS: real sensors and six verified placements, two of each class.')
        print(records[0].read_text())
    finally:
        try:process.send_signal(signal.SIGINT)
        except ProcessLookupError:pass
        try:process.wait(timeout=20)
        except subprocess.TimeoutExpired:
            try:os.killpg(process.pid,signal.SIGKILL)
            except ProcessLookupError:pass
            process.wait()
        print('Launch log:',launch_log,'Task log:',sort_log)
        # Include diagnostics in the ordinary CI job log as well as artifacts.
        for path in [launch_log,sort_log]:
            if path.exists():
                print('\n'+str(path)+'\n'+path.read_text(errors='replace')[-40000:])
