#!/usr/bin/env python

from __future__ import division
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np
from PIL import Image
import random
import cv2
import collections


def bfs(graph, start, end):
    # maintain a queue of paths
    queue = []
    #visited = set([start])
    visited = []
    # push the first path into the queue
    queue.append(start) # [[25,25]]
    #queue= collections.deque(start)
    visited.append(start)
    w=[]
    l=0
    while len(queue)>0:
        # get the first path from the queue

        path = queue.pop(0)

        if (isinstance(path[0],int)):
            p = path
            l=1
        else:
            p=path[-1]
            l=0

    #xx.append(path)
    #print(new_path)
        # get the last node from the path
        #node = path[-1]
    #new_path = []
        # path found


        x= p[0]
        y= p[1]



        # enumerate all adjacent nodes, construct a new path and push it into the queue
    #new_path= list()
    #node x+1 y
    #print(x)
    #print(y)
        if x+1 < 100 and [x+1, y] not in visited and graph[x+1,y] != 0:
            if(l==1):
                q=[]
                q.append(path)
                q.append([x + 1, y])  # queue.append( path + [x+1,y])
                queue.append(q)
                if x + 1 == end[0] and y == end[1]:
                    print("ccc")

                    return q
            else:
                i=0
                new=[]
                while(i<=len(path)-1):
                    new.append(path[i])

                    i=i+1

                new.append([x + 1, y])
                queue.append(new)
                if x+1 == end[0] and y == end[1]:
                    print("ccc")

                    return new
            #new_path.append([x+1,y])


            visited.append([x+1,y])
    #node x+1 y+1
        if x+1< 100 and y+1 <40 and [x+1,y+1] not in visited and graph[x+1,y+1] != 0:
            if(l==1):
                q=[]
                q.append(path)
                q.append([x + 1, y+1])  # queue.append( path + [x+1,y])
                queue.append(q)
                if x + 1 == end[0] and y+1 == end[1]:
                    print("ccc")

                    return q
            else:
                i = 0
                new = []
                while (i <= len(path) - 1):
                    new.append(path[i])

                    i = i + 1

                new.append([x + 1, y+1])
                queue.append(new)
                if x+1 == end[0] and y+1 == end[1]:
                    print("ccc")

                    return new
            # new_path.append([x+1,y])
            visited.append([x+1,y+1])
    #node x y+1
        if y+1<40 and [x,y+1] not in visited and graph[x,y+1] != 0:

            if(l==1):
                q=[]
                q.append(path)
                q.append([x, y+1])  # queue.append( path + [x+1,y])
                queue.append(q)
                if x == end[0] and y+1 == end[1]:
                    print("ccc")

                    return q
            else:
                i = 0
                new = []
                while (i <= len(path) - 1):
                    new.append(path[i])

                    i = i + 1

                new.append([x, y+1])
                queue.append(new)
                if x == end[0] and y+1 == end[1]:
                    print("ccc")

                    return new
            visited.append([x,y+1])
    #node x-1 y+1
        if x-1>-1 and y+1 <40 and [x-1,y+1] not in visited and graph[x-1,y+1] != 0:
            if(l==1):
                q=[]
                q.append(path)
                q.append([x -1 , y+1])  # queue.append( path + [x+1,y])
                queue.append(q)
                if x - 1 == end[0] and y+1 == end[1]:
                    print("ccc")

                    return q
            else:
                i = 0
                new = []
                while (i <= len(path) - 1):
                    new.append(path[i])

                    i = i + 1

                new.append([x - 1, y+1])
                queue.append(new)
                if x-1 == end[0] and y+1 == end[1]:
                    print("ccc")

                    return new
            visited.append([x-1,y+1])

    #node x-1 y
        if x-1>-1 and [x-1,y] not in visited and graph[x-1,y] != 0:
            if (l == 1):
                q = []
                q.append(path)
                q.append([x -1, y])  # queue.append( path + [x+1,y])
                queue.append(q)
                if x - 1 == end[0] and y == end[1]:
                    print("ccc")

                    return q
            else:
                i = 0
                new = []
                while (i <= len(path) - 1):
                    new.append(path[i])

                    i = i + 1

                new.append([x - 1, y])
                queue.append(new)
                if x - 1 == end[0] and y == end[1]:
                    print("ccc")

                    return new
            # new_path.append([x+1,y])
            visited.append([x-1,y])
    #node x-1 y-1
        if x-1>-1 and y-1>-1 and [x-1,y-1] not in visited and graph[x-1,y-1] != 0:
            if(l==1):
                q=[]
                q.append(path)
                q.append([x - 1, y-1])  # queue.append( path + [x+1,y])
                queue.append(q)
                if x - 1 == end[0] and y-1 == end[1]:
                    print("ccc")

                    return q
            else:
                i = 0
                new = []
                while (i <= len(path) - 1):
                    new.append(path[i])

                    i = i + 1

                new.append([x - 1, y-1])
                queue.append(new)
                if x-1 == end[0] and y-1 == end[1]:
                    print("ccc")

                    return new
            visited.append([x-1,y-1])

    #node x y-1
        if y-1>-1 and [x,y-1] not in visited and graph[x,y-1] != 0:

            if(l==1):
                q=[]
                q.append(path)
                q.append([x, y-1])  # queue.append( path + [x+1,y])
                queue.append(q)
                if x == end[0] and y-1 == end[1]:
                    print("ccc")

                    return q
            else:
                i = 0
                new = []
                while (i <= len(path) - 1):
                    new.append(path[i])

                    i = i + 1

                new.append([x, y-1])
                queue.append(new)
                if x == end[0] and y-1 == end[1]:
                    print("ccc")

                    return new
            # new_path.append([x+1,y])
            visited.append([x,y-1])
    #node x+1 y-1
        if x+1< 100 and y-1 >-1 and [x+1,y-1] not in visited and graph[x+1,y-1] != 0:

            #new_path.append([x+1,y-1])
            if (l == 1):
                q = []
                q.append(path)
                q.append([x + 1, y-1])  # queue.append( path + [x+1,y])
                queue.append(q)
                if x + 1 == end[0] and y-1 == end[1]:
                    print("ccc")

                    return q
            else:
                i = 0
                new = []
                while (i <= len(path) - 1):
                    new.append(path[i])

                    i = i + 1

                new.append([x + 1, y-1])
                queue.append(new)
                if x + 1 == end[0] and y-1 == end[1]:
                    print("ccc")

                    return new
            visited.append([x+1,y-1])
    #print(len(queue))

    return None


def bfss(data, start, endRow, endCol):
    queue = collections.deque([[start]])
    seen = set(start)
    while queue:
        path = queue.popleft()
        x, y = path[-1]
        if x == endCol and y == endRow:
            return path
        # for x2, y2 in ((x,y+1), (x-1,y+1), (x-1,y), (x-1,y-1), (x,y-1), (x+1, y-1), (x+1,y), (x+1, y+1)):
        for x2, y2 in ((x, y + 1), (x - 1, y), (x, y - 1), (x + 1, y)):
            if 0 <= x2 < 100 and 0 <= y2 < 40 and data[x2,y2] != 0  and (x2, y2) not in seen:
                queue.append(path + [(x2, y2)])
                seen.add((x2, y2))


width = 40;
height= 100;
img = Image.new ('1', (height, width)) #array = np.zeros([height, width], dtype=np.uint)
pixels= img.load()
for i in range(img.size[0]):
    for j in range(img.size[1]):
        pixels[i,j] = 255;
        if j==27 or j==28 or j==29 or j==30 or j==31 or j==32 or j==33:
            if i>3 and i<17 or i>21 and i<33 or i==50 or i>57 and i<73 or i== 80 or i==90 or i==95:
                pixels[i,j] = 0;
        if j==18 or j==19 or j==20 or j==21 or j==22:
            if i>3 and i<17 or i==30 or i>33 and i<42 or i==50 or i>57 and i<73 or i>78 and i<92 or i==95:
                pixels[i,j] = 0;
        if j==8 or j==9 or j==10 or j==11 or j==12:
            if i>3 and i<17 or i==30 or i>57 and i<73 or i==90 or i==95 or i==50:
                pixels[i,j] = 0;

        if (i==13 or i==14 or i==15 or i==16 or i==17) and j>=8 and j<=32:
                pixels[i,j] = 0;

        if (i== 28 or i==29 or i==30 or i==31 or i==32) and j>=8 and j<=32:
                pixels[i,j] = 0;

        if (i==48 or i==49 or i==50 or i==51 or i==52) and j>=8 and j<=32:
                pixels[i,j] = 0;

        if (i== 58or i==59 or i== 60 or i==61 or i==62) and j>=18 and j<=32:
                pixels[i,j] = 0;

        if (i==68 or i== 69 or i== 70 or i==71 or i== 72) and j>=8 and j<=22:
                pixels[i,j] = 0;

        if (i==78 or i==79 or i==80 or i==81 or i==82) and j>=18 and j<=32:
                pixels[i,j] = 0;

        if (i==89 or i==90 or i== 91 ) and j>=8 and j<=32:
                pixels[i,j] = 0;

        if (i== 94 or i==95 or i== 96 ) and j>=8 and j<=32:
                pixels[i,j] = 0;
        if(i<100) and j==0 or j==1 or j==2 or j==37 or j== 38 or j==39:
            pixels[i, j] = 0;
        if (j < 40) and i == 0 or i == 1 or i == 2 or i == 98 or i == 99:
            pixels[i, j] = 0;
#ret, bw_img = cv2.threshold(img, 127,255, cv2.THRESH_BINARY)
#cv2.imshow("Binary Image", bw_img)
#kosomak= cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)[1]
#print(type(img))
#meshwar= bfss(pixels, [5,5], 30,37)
meshwar= bfs(pixels, [5,35], [85,5])
#print(pixels[201,100])
pixels[25,25]= 30;
a=0
if(meshwar is not None):
    while(a<=len(meshwar)-1):
        f=meshwar[a]
        pixels[f[0],f[1]]=0
        a =a+1
#pixels[100,90]= 200;
print (meshwar)
print(meshwar[1])
print((meshwar[1])[0])
print((meshwar[1])[0]/10)
print((meshwar[1])[1]/10)
img.show()





