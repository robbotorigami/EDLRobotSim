import glob
import os
import os
sumat = 0
filetypes = ['cpp', 'h', 'ino',' py']
exclude = ['RobotSketch\\dubins.cpp',  'RobotSketch\\dubins.h']
files = []
for ft in filetypes:
    files += glob.glob('*.' + ft)
    files += glob.glob('**/*.' + ft)
for filename in files:
    if filename in exclude:
        continue
    with open(filename) as f:
        for i, l in enumerate(f):
            pass
    sumat += i + 1
    print(filename, i+1)
print(sumat)
os.system("pause")
