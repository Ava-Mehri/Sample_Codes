#Ava Mehri, UIC, amehri2@uic.edu, 2021
#Cost Constrainted Shortest Paths Problem
#Determines the fastest way to get to a destination within a certain budget in a graph

# -*- coding: utf-8 -*-


import sys
from queue import PriorityQueue
from queue import LifoQueue

filename = sys.argv[1]
start = int(sys.argv[2])
destination = int(sys.argv[3])
budget = int(sys.argv[4])

in_file =  sys.argv[1]
#open the input file and store it
file = open(in_file, 'r').read()
#read the file line by line and store lines in a list
for line in file:
  lines = file.split('\n')
if (lines[-1] == ''):
    lines = lines[0:-1]
#remove spaces before each line
for i in range(len(lines)):
  lines[i] = lines[i].lstrip()
  lines[i] = lines[i].split(' ')
#transform strings to integers
for i in range(len(lines)):
  for j in range(len(lines[i])):
    lines[i][j] = int(lines[i][j])

graph = {}
P_vertices = {}
predecessor = {}
for i in range(len(lines)):
  u = lines[i][0]
  v = lines[i][1]
  c = lines[i][2]
  t = lines[i][3]
  #print( "graph:", graph)
  #print( "P_vertices", P_vertices)
  if not u in P_vertices:
    P_vertices.update({u:[]})
    predecessor.update({u:PriorityQueue()})
  if not v in P_vertices:
    P_vertices.update({v:[]})
    predecessor.update({v:PriorityQueue()})

  if not u in graph:
    graph.update({u:{v: (c,t)}})
  if u in graph:
    if not v in graph[u]:
      graph[u].update({v: (c,t)})

predecessor[start].put((0,0,start))

Q = PriorityQueue()
Q.put((0,0,start))
while not Q.empty():
  temp = Q.get()
  cost = temp[0]
  time = temp[1]
  u = temp[2]
  if not P_vertices[u]:
    P_vertices[u].append((cost, time, predecessor[u].get()[2]))
  else:
    if cost >= P_vertices[u][-1][0] and time < P_vertices[u][-1][1]:
      P_vertices[u].append((cost, time, predecessor[u].get()[2]))

  V_out = list(graph.keys())
  if u in V_out:
    g = graph[u]
    neighbors = list(g.keys())
    for v in neighbors:
      cost_v = cost + g[v][0]
      time_v = time + g[v][1]
      if not P_vertices[v]:
        Q.put((cost_v, time_v, v))
        predecessor[v].put((cost_v, time_v, u))
      else:
        if cost_v >= P_vertices[v][-1][0] and time_v < P_vertices[v][-1][1]:
          Q.put((cost_v, time_v, v))
          predecessor[v].put((cost_v, time_v, u))

#post-process to extarct the tradeoff curve
P = {}
for p in P_vertices:
  P[p] = []
  for i in range(len(P_vertices[p])):
    P[p].append((P_vertices[p][i][0], P_vertices[p][i][1]))

print("***************************************")
print(f'trade-off curves:\n{P}')
print("***************************************")

#post-process to extract all the paths found by the algoritm
allPaths = []
def findPathFrom( end, penultimate, totalCForEnd, totalTForEnd, path):
    print("-------------")
    print("From", penultimate)
    print("To", end)
    print("totalCForEnd", totalCForEnd)
    print("totalTForEnd", totalTForEnd)
    
    if penultimate == end and end == 0:
        return True
    print("individual edge",graph[penultimate][end] )
    if totalCForEnd >= graph[penultimate][end][0] and totalTForEnd >= graph[penultimate][end][1]:        
        tempPath = path + [end]
        print("tempPath", tempPath)
        for pct in P_vertices[penultimate]:
            if findPathFrom( penultimate, pct[2], totalCForEnd-graph[penultimate][end][0], totalTForEnd-graph[penultimate][end][1], tempPath):
                print("adding")
                print(tempPath + [0])
                allPaths.append( tempPath + [0])
    else:
        return False

dest = destination
allPaths = []
print("***************************************")
print("destination", dest)
for pct in P_vertices[dest]:
    findPathFrom( dest, pct[2], pct[0], pct[1], [])
print(allPaths)
print("***************************************")

#print fastest cost feasible path
minCost,minTime = float("Inf"),float("Inf")
minPath = None
for aPath in allPaths:
    cost, time = 0, 0
    for i in range( len( aPath)-1, 0, -1):
        cost += graph[aPath[i]][aPath[i-1]][0]
        time += graph[aPath[i]][aPath[i-1]][1]
    print(cost,time)
    if cost <= budget:
        if time < minTime:
            minCost = cost # not really the minCost
            minTime = time
            minPath = aPath
print(f'budget= {budget}')
print(minCost, minTime, minPath)
