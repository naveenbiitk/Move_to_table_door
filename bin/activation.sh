#!/usr/bin/env python3
import subprocess
import sys
import time

app = "gnome-terminal"
s = (" ").join(sys.argv[1:])

def get(cmd):
    return subprocess.check_output(cmd).decode("utf-8").strip()

def front(app):
    try:
        # see if gnome-terminal is running at all (raising error if not)
        app = get(["pgrep", app]) 
    except subprocess.CalledProcessError:
        app = False
    if app:
        # if so, see if the active window belongs to gnome-terminal comparing pids)
        active = get(["xdotool", "getwindowpid", get(["xdotool", "getactivewindow"])])
        return True if app == active else False

if front(app):
    # copy command to clipboard
    cm1 = ["/bin/bash", "-c", 'printf "'+s+'" | xclip -selection clipboard']
    # paste in terminal window
    cm2 = ["xdotool", "key", "Control_L+Shift_L+v"]
    # press return
    cm3 = ["xdotool", "key", "Return"]
    for cm in [cm1, cm2, cm3]:
        subprocess.call(cm)
