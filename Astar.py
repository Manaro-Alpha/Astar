import matplotlib.pyplot as plt
import numpy as np
import math
import operator
from shapely.geometry import Point, Polygon, LineString
from itertools import product,permutations


class Node:
    def __init__(self,x,y,cost,parent,nearest_nodes,index):
        self.x=x
        self.y=y
        self.parent=parent
        self.cost=cost
        self.nearest_nodes=nearest_nodes
        self.index=index

class Astar:
    obstacle_list=[[(2, 10), (7, 10), (6, 7), (4, 7), (4, 9), (2, 9)],[(3, 1), (3, 6), (4, 6), (4, 1)],[(7, 3), (7, 8), (9, 8), (9, 3)]]
    
    def within(obstacle_list,rnd_node):
        #checking if point is valid
        isWithin=False
        point=Point(rnd_node[0],rnd_node[1])
        if(point.within(Polygon(obstacle_list[0]))==True or point.within(Polygon(obstacle_list[1]))==True or point.within(Polygon(obstacle_list[2]))==True):
            isWithin=True
        return isWithin

    def cross(obstacle_list,rnd_node,x,y):
        isCrosses=False
        point=Point(rnd_node[0],rnd_node[1])
        node=(x,y)
        coord=(rnd_node,node)
        line=LineString(coord)
        if(line.crosses(Polygon(obstacle_list[0]))==True or line.crosses(Polygon(obstacle_list[1]))==True or line.crosses(Polygon(obstacle_list[2]))==True or point.touches(Polygon(obstacle_list[0]))==True or point.touches(Polygon(obstacle_list[1])) ==True or point.touches(Polygon(obstacle_list[2]))==True):
            isCrosses=True
        return isCrosses


    start=(0,0)
    goal=(10,10)
    num_nodes=1000
    rate=0.8
    
        #Generating the node network
    nodes=[]
    nodes.append(Node(start[0],start[1],0,0,[],0))
    a=np.arange(0,10.5,0.5).tolist()
    b=list(product(a,repeat=2))
    for coord in b:
        if(within(obstacle_list,coord)==True):
            continue
        nodes.append(Node(coord[0],coord[1],0,0,[],len(nodes)))
        for i in range(0,len(nodes)-1):
            dist=math.sqrt((coord[0]-nodes[i].x)**2 + (coord[1]-nodes[i].y)**2)
            if(dist<rate and cross(obstacle_list,coord,nodes[i].x,nodes[i].y)==False):
                nodes[i].nearest_nodes.append(len(nodes)-1)
                nodes[len(nodes)-1].nearest_nodes.append(i)
        
    ''' nodes.append(Node(goal[0],goal[1],0,0,[],len(nodes)))
        for i in range(0,len(nodes)):
            dist=math.sqrt((nodes[i].x-goal[0])**2 + (nodes[i].y-goal[1])**2)
            if(dist<rate and cross(obstacle_list,goal,nodes[i].x,nodes[i].y)==False):
                nodes[i].nearest_nodes.append(len(nodes)-1)
                nodes[len(nodes)-1].nearest_nodes.append(i)
    '''
        
    ''' for i in range(0,num_nodes):
        rnd_node=(random.uniform(0,10),random.uniform(0,10))
        node=Node(rnd_node[0],rnd_node[1],0,0,[],len(nodes))

        if(within(obstacle_list,rnd_node)==True):
            continue
        nodes.append(node)

        for i in range(0,len(nodes)-1):
            dist=math.sqrt((rnd_node[0]-nodes[i].x)**2 + (rnd_node[0]-nodes[i].y)**2)
            if(dist<=rate and cross(obstacle_list,rnd_node,nodes[i].x,nodes[i].y)==False):
                nodes[i].nearest_nodes.append(len(nodes)-1)
                nodes[len(nodes)-1].nearest_nodes.append(i)

        nodes.append(Node(goal[0],goal[1],0,0,[],len(nodes)))
        for i in range(0,len(nodes)):
            dist=math.sqrt((nodes[i].x-goal[0])**2 + (nodes[i].y-goal[1])**2)
            if(dist<=rate and cross(obstacle_list,rnd_node,nodes[i].x,nodes[i].y)==False):
                nodes[i].nearest_nodes.append(len(nodes)-1)
                nodes[len(nodes)-1].nearest_nodes.append(i)

    '''
                     
        #Astar
        #start
    open_list=[]
    IsGoal=False
    for node in nodes:
        node.cost=1000000000
    nodes[0].cost=0
    for index in nodes[0].nearest_nodes:
        nodes[index].cost=math.sqrt((nodes[0].x-nodes[index].x)**2 + (nodes[0].y-nodes[index].y)**2) + math.sqrt((nodes[index].x-goal[0])**2 + (nodes[index].y-goal[1])**2)
        nodes[index].parent=0
        
    for node in nodes:
        open_list.append([node.index,node.cost])
    open_list.sort(key=lambda x:x[1])

    while(IsGoal==False and len(open_list)>0):
        if(open_list[0][0]==len(nodes)-1): 
            #len(nodes)-1 is the last nodes index 
            IsGoal=True
            
        for index in nodes[open_list[0][0]].nearest_nodes:
            cost=open_list[0][1] + math.sqrt((nodes[open_list[0][0]].x-nodes[index].x)**2 + (nodes[open_list[0][0]].y-nodes[index].y)**2) + math.sqrt((nodes[index].x-goal[0])**2 + (nodes[index].y-goal[1])**2)
            if(cost<nodes[index].cost):
                nodes[index].cost=cost
                nodes[index].parent=open_list[0][0]
            
        open_list.pop(0)
        for array in open_list:
            array[1]=nodes[array[0]].cost
        open_list.sort(key=lambda x:x[1])

    poly1=Polygon(obstacle_list[0])
    poly2=Polygon(obstacle_list[1])
    poly3=Polygon(obstacle_list[2])
    x1,y1=poly1.exterior.xy
    x2,y2=poly2.exterior.xy
    x3,y3=poly3.exterior.xy
    plt.plot(x1,y1)
    plt.plot(x2,y2)
    plt.plot(x3,y3)
    #plotting the nodes and path
    '''
    for node in nodes:
        for index in node.nearest_nodes:
            plt.scatter([node.x,nodes[index].x],[node.y,nodes[index].y],color='r')
    '''
        
    current=len(nodes)-1
    plot_list=[]

    while(current!=0):
        plot_list.append(nodes[current])
        current=nodes[current].parent

    plot_list.append(Node(start[0],start[1],0,0,[],0))
    plot_list.reverse()
    i=0
    while i<len(plot_list)-1:
        plt.plot([plot_list[i].x,plot_list[i+1].x],[plot_list[i].y,plot_list[i+1].y],color='blue')
        i=i+1
    for node in plot_list:
        plt.scatter(node.x,node.y,color='blue',s=30)
        
def main():
    print("start planning")
    astar=Astar()

if __name__=='__main__':
    print("start")
    main()
plt.show()
