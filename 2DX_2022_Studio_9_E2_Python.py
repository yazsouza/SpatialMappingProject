import serial
import math
import open3d as o3d
import numpy as np


#Get data from Port

s = serial.Serial("COM3", 115200)

#Get XYZ

def toXYZ(distance, theta, x):
    x = x #x is down hallway
    y = distance*math.sin(theta) #y is up-down
    z = distance*math.cos(theta) #z is across
    
    print(x,y,z)
    return x,y,z

print(len(distances))
angleChange = 22.5
numOfPlanes = 8

#Loop with serial port 
#read distance from serial port and append to distances
#firs distance should be measured at theta = 0
out = []
distanceIndex = 0
f = open("demofile2dx.xyz", "w")    #create a new file for writing

for i in range(numOfPlanes):
    for j in range(int(360/angleChange)):
        theta = (2 - (j*angleChange)/180) * math.pi - math.pi #starting on left rotate cw 

        x = s.readline()
        y = x.decode()
        z = int(input.strip('\n') #take out new line character
        distance = z.split()[1]
        print(distance)
        
        x,y,z = toXYZ(distance, theta, i*0.3)
        out.append([x,y,z])
        distanceIndex += 1
        f.write('{0:.2f} {1:.2f} {2:.2f}\n'.format(x,y,z))    #write x,0,0 (xyz) to file as p1
    
f.close()    
#Read the test data in from the file we created        
print("Read in the prism point cloud data (pcd)")
pcd = o3d.io.read_point_cloud("demofile2dx.xyz", format="xyz")

#Lets see what our point cloud data looks like numerically       
print("The PCD array:")
print(np.asarray(pcd.points))

#Lets see what our point cloud data looks like graphically       
print("Lets visualize the PCD: (spawns seperate interactive window)")
#o3d.visualization.draw_geometries([pcd])

#add lines to connect vertices

#Give each vertex a unique number
yz_slice_vertex = []
for x in range(0,numOfPlanes*int(360/angleChange)):
    yz_slice_vertex.append([x])

#Define coordinates to connect lines in each yz slice        
lines = []  
for x in range(0,numOfPlanes*int(360/angleChange),int(360/angleChange)):
    lines.append([yz_slice_vertex[x], yz_slice_vertex[x+1]])
    lines.append([yz_slice_vertex[x+1], yz_slice_vertex[x+2]])
    lines.append([yz_slice_vertex[x+2], yz_slice_vertex[x+3]])
    lines.append([yz_slice_vertex[x+3], yz_slice_vertex[x+4]])
    lines.append([yz_slice_vertex[x+4], yz_slice_vertex[x+5]])
    lines.append([yz_slice_vertex[x+5], yz_slice_vertex[x+6]])
    lines.append([yz_slice_vertex[x+6], yz_slice_vertex[x+7]])
    lines.append([yz_slice_vertex[x+7], yz_slice_vertex[x+8]])
    lines.append([yz_slice_vertex[x+8], yz_slice_vertex[x+9]])
    lines.append([yz_slice_vertex[x+9], yz_slice_vertex[x+10]])
    lines.append([yz_slice_vertex[x+10], yz_slice_vertex[x+11]])
    lines.append([yz_slice_vertex[x+11], yz_slice_vertex[x + 12]])
    lines.append([yz_slice_vertex[x+12], yz_slice_vertex[x+13]])
    lines.append([yz_slice_vertex[x+13], yz_slice_vertex[x+14]])
    lines.append([yz_slice_vertex[x+14], yz_slice_vertex[x+15]])
    lines.append([yz_slice_vertex[x+15], yz_slice_vertex[x]]) #closes loop

#Define coordinates to connect lines between current and next yz slice
    
for x in range(0,(numOfPlanes-1)*int(360/angleChange),int(360/angleChange)):
    lines.append([yz_slice_vertex[x], yz_slice_vertex[x+16]])
    lines.append([yz_slice_vertex[x+1], yz_slice_vertex[x+17]])
    lines.append([yz_slice_vertex[x+2], yz_slice_vertex[x+18]])
    lines.append([yz_slice_vertex[x+3], yz_slice_vertex[x+19]])
    lines.append([yz_slice_vertex[x+4], yz_slice_vertex[x+20]])
    lines.append([yz_slice_vertex[x+5], yz_slice_vertex[x+21]])
    lines.append([yz_slice_vertex[x+6], yz_slice_vertex[x+22]])
    lines.append([yz_slice_vertex[x+7], yz_slice_vertex[x+23]])
    lines.append([yz_slice_vertex[x+8], yz_slice_vertex[x+24]])
    lines.append([yz_slice_vertex[x+9], yz_slice_vertex[x+25]])
    lines.append([yz_slice_vertex[x+10], yz_slice_vertex[x+26]])
    lines.append([yz_slice_vertex[x+11], yz_slice_vertex[x+27]])
    lines.append([yz_slice_vertex[x+12], yz_slice_vertex[x+28]])
    lines.append([yz_slice_vertex[x+13], yz_slice_vertex[x+29]])
    lines.append([yz_slice_vertex[x+14], yz_slice_vertex[x+30]])
    lines.append([yz_slice_vertex[x+15], yz_slice_vertex[x+31]])
    
#This line maps the lines to the 3d coordinate vertices
line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)),lines=o3d.utility.Vector2iVector(lines))

#Lets see what our point cloud data with lines looks like graphically       
o3d.visualization.draw_geometries([line_set])
print(out)
