import subprocess
import os
import matlab.engine

eng = matlab.engine.start_matlab()
# get all the pcd files in the subdirectory
current_path = os.path.dirname(os.path.realpath(__file__))

files = []
# r=root, d=directories, f = files
for r, d, f in os.walk(current_path):
    for file in f:
        if '.txt' in file:
            files.append(os.path.join(r, file))
			
for file in files:
	m_rot = eng.zeros(3, 3)
	trans = eng.zeros(1, 3)
	lineList = [line.rstrip('\n') for line in open(file, "r")]
	for i in range(3):
		transform_line = lineList[i+1]
		values_split = transform_line.split(" ")
		eng.eval(m_rot(i+1, 1) = values_split[0];)
		eng.eval(m_rot(i+1, 2) = values_split[1];)
		eng.eval(m_rot(i+1, 3) = values_split[2];)
		eng.eval(trans(i+1) = values_split[3];)
	
	
		
		
		
		
