#!/usr/bin/env python

import re

from genpy.message import fill_message_args
import rospy

from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry

import sys




l = 0
c = 0
r = 0

globalX = 0
globalY = 0

X = []
Y = []

def clbk_odom(msg):
    global globalX
    global globalY

    globalX = msg.pose.pose.position.x
    globalY = msg.pose.pose.position.y

def clbk_laser(msg):
    global l 
    global c 
    global r 

    l = msg.data[2]
    r = msg.data[0]
    c = msg.data[1]

class Cord:
    
    def __init__(self, xv, yv) :
        self.x = xv
        self.y = yv

    
class Cell:

    def __init__(self, pos) :
        self.visited = False
        self.n = False
        self.s = False
        self.e = False
        self.w = False

        self.val = 100

        self.position = Cord(pos.x,pos.y)


class Grid:
    
    def __init__(self, ensize):
        self.size = ensize

        self.grid = [[Cell(Cord(i,j)) for j in range(self.size.y)] for i in range(self.size.x)]

    def getCell(self,pos):
        return self.grid[pos.x][pos.y]

    def setCell(self, cell):
        self.grid[cell.position.x][cell.position.y] = cell

    def isvalid(self,pos):
        if (pos.x >= 0 and pos.x < self.size.x) and (pos.y >= 0 and pos.y < self.size.y):
            return True
        else:
            return False


class FloodFill:

    def __init__(self, start, dest):
        self.start = start
        self.dest = dest

        self.orien = 0

        self.arena = Grid(Cord(16,16))

    def updateGrid(self, cur):
        global l 
        global c 
        global r

        if self.orien == 0:
            self.arena.grid[cur.x][cur.y].n = c
            self.arena.grid[cur.x][cur.y].w = l
            self.arena.grid[cur.x][cur.y].e = r
        
        elif self.orien == 1:
            self.arena.grid[cur.x][cur.y].e = c
            self.arena.grid[cur.x][cur.y].n = l
            self.arena.grid[cur.x][cur.y].s = r
        
        elif self.orien == 2:
            self.arena.grid[cur.x][cur.y].s = c
            self.arena.grid[cur.x][cur.y].e = l
            self.arena.grid[cur.x][cur.y].w = r
        
        elif self.orien == 3:
            self.arena.grid[cur.x][cur.y].w = c
            self.arena.grid[cur.x][cur.y].s = l
            self.arena.grid[cur.x][cur.y].n = r

    def setOrient( self, code):
        if code < 0 and self.orien == 0:
            self.orien = 3

        else:
            self.orien += code
            self.orien = self.orien %4

    def canGo(self, fr , to):
	#print("checking cango for")
	#print(fr.x,fr.y)
	#print(to.x,to.y)
	rate = rospy.Rate(1000) # 10hz
        rate.sleep()
	#print("checking cango for")
	#print(fr.x,fr.y)
	#print(to.x,to.y)
        if (self.arena.isvalid(fr) == False) or (self.arena.isvalid(to) == False):
            return False
	
        xv = to.x - fr.x
        yv = to.y - fr.y
	
        frc = self.arena.getCell(fr)
        t = self.arena.getCell(to)

        if xv == 0:
            if yv> 0:
                return not (frc.n or t.s)
            elif yv <0 :
                return not (frc.s or t.n)

        elif yv == 0:
            if xv > 0:
                return not (frc.e or t.w)
            elif xv < 0:
                return not (frc.w or t.e)

        else:
            return False


    def Procfill(self, des):

        for i in range(self.arena.size.x):
            for j in range(self.arena.size.y):
                if(i == des.x and j == des.y):
                    continue
                
                self.arena.grid[i][j].val = 99

        q = []
        q.append(des)
        self.arena.grid[des.x][des.y].visited = True

        while len(q) > 0:
            v = q[0]
            q.pop(0)

            dirs = [Cord(0,1),Cord(0,-1), Cord(1,0), Cord(-1,0)]

            for i in dirs:
                if(self.arena.isvalid(Cord(v.x + i.x, v.y + i.y)) and self.canGo(v, Cord(v.x + i.x, v.y + i.y))):
                    curCell = self.arena.getCell(Cord(v.x + i.x, v.y + i.y))
                    if  curCell.visited == False:
                        curCell.visited = True
                        q.append(curCell.position)
                        curCell.val = self.arena.getCell(v).val+1
                        self.arena.setCell(curCell)


    def Fill(self):
        self.arena.grid[self.dest.x][self.dest.y].val = 0
        self.Procfill(self.dest)

        for i in range(self.arena.size.x):
            for j in range(self.arena.size.y):
                self.arena.grid[i][j].visited = False

    def hasWalls(self, cur):
        cel = self.arena.getCell(cur)

        return (cel.n or cel.s or cel.e or cel.w)

    def getDir(self, fr , to ):
        
        xv = to.x - fr.x
        yv = to.y - fr.y

        tdir = 0

        if xv == 0 and yv == 0:
            tdir = 0
        elif xv > 0:
            tdir = 1
        elif xv < 0 :
            tdir = 3
        elif yv < 0 : 
            tdir = 2
        elif yv > 0 :
            tdir = 0

        diff = tdir - self.orien

        if abs(diff) == 2:
            return 2
        elif diff == 1 :
            return 1
        elif diff == -1:
            return -1
        elif diff == 3:
            return -1
        elif diff == -3 :
            return 1
        elif diff == 0:
            return 0

    def moveto(self,fr , to, movePub):
        global globalX
        global globalY
        global l
        global c 
        global r
        print("to ", to.x, to.y)
	rate = rospy.Rate(100) # 10hz
	print("In Move to for")
	print(fr.x, fr.y)
	print(to.x, to.y)
        dir = self.getDir(fr, to)

        mo = Int32MultiArray()

        mo.data.append(dir)
        mo.data.append(to.x)
        mo.data.append(to.y)

        while abs(globalX-X[to.y])>0.015 or abs(globalY-Y[to.x]) > 0.015:
            movePub.publish(mo)
	    print("in moveto while loop ")
	    print(globalX,globalY,X[to.y],Y[to.x])
            rate.sleep()


        if dir == 2:
            self.setOrient(2)

        elif dir == 1:
            self.setOrient(1)

        elif dir == -1:
            self.setOrient(-1)

        elif dir==0:
            pass

	
	print("In Move to for")
	print(fr.x, fr.y)
	print(to.x, to.y)



    def findPath(self,movePub):
        self.curr = self.start

        self.Fill()
	rate = rospy.Rate(100) # 10hz
        rate.sleep()
        while (self.curr.x != self.dest.x) or (self.curr.y != self.dest.y):
	    
            self.updateGrid(self.curr)
            print("The cell is  : ", self.curr.x, self.curr.y)
            if self.hasWalls(self.curr):
                self.Fill()

            dirs = [Cord(0,1),Cord(0,-1), Cord(1,0), Cord(-1,0)]

            rate.sleep()
            for i in dirs:
                if self.arena.isvalid(Cord(self.curr.x + i.x, self.curr.y + i.y)) == False:
                    continue
                if(self.arena.getCell(self.curr).val > self.arena.getCell(Cord(self.curr.x + i.x, self.curr.y + i.y)).val):
		
        	    
                    if self.arena.isvalid(Cord(self.curr.x + i.x, self.curr.y + i.y)) == False:
                        continue
                    if self.canGo(self.curr, Cord(self.curr.x + i.x, self.curr.y + i.y)) == False:
                        continue
		    print("canGo(0,0 to 0,1)")
  		    
        	    
		    print(self.canGo(Cord(0,0), Cord(0,1)))
		    print("curr cell is :")
		    print(self.curr.x, self.curr.y)
		    print(self.arena.grid[self.curr.x][self.curr.y].n, self.arena.grid[self.curr.x][self.curr.y].e, self.arena.grid[self.curr.x][self.curr.y].s, self.arena.grid[self.curr.x][self.curr.y].w)
                    self.moveto(self.curr, Cord(self.curr.x + i.x, self.curr.y + i.y),movePub)

                    self.curr = Cord(self.curr.x + i.x, self.curr.y + i.y)
                    self.updateGrid(self.curr)
                    break



def getCocen():
    X1,X2 = [],[]
    cell = 0.168
    wall = 0.012
    X1.append(round(cell/2 + wall/2,3))
    X2.append(round(-1*cell/2 - wall/2,3))
    for i in range(7):
        X1.append(round(X1[-1] + cell + wall,3))
        X2.append(round(X2[-1] - cell - wall,3))
    #X_cen,Y_cen = [],[]
    global X
    global Y
    X2.reverse()
    X = X2 + X1
    Y = X[:]
    Y.reverse()
    #print(X)
    #print(Y)

def main():

    rospy.init_node("floodFill")

    sub = rospy.Subscriber('/laserWall', Int16MultiArray, clbk_laser)
    odsub = rospy.Subscriber('/odom', Odometry, clbk_odom)
    pub = rospy.Publisher('/moveToCell', Int32MultiArray, queue_size=100)
    getCocen()

    pro = FloodFill(Cord(0,0),Cord(7,7))
    pro.arena = Grid(Cord(16,16))
    pro.findPath(pub)

if __name__ == '__main__':
    main()

