#Ava Mehri, UIC, amehri2@uic.edu, 2021
#Markov Decision Process Problem
#Solves a Markov decision process problem for a generic grid world
#value iteration and modified policy iteration algorithms.



# -*- coding: utf-8 -*-


import numpy as np

file = open('input.txt', 'r').read()
#read the file line by line and store lines in a list
for line in file:
  lines = file.split('\n')

#parse input file into MDP class
N = int(lines[0].split()[1])
M = int(lines[0].split()[0])

in_wall = lines[1].split(',')
n_wall = len(in_wall)
wall = []
for i in range(n_wall):
  wall.append((int(in_wall[i].split()[1])-1, int(in_wall[i].split()[0])-1))

in_goal = lines[2].split(',')
n_goal = len(in_goal)
goal = []
for i in range(n_goal):
  goal.append((int(in_goal[i].split()[1])-1, int(in_goal[i].split()[0])-1, int(in_goal[i].split()[2])))

reward = float(lines[3])

probs = lines[4].split()
probs[0] = float(probs[0])
probs[1] = float(probs[1])
probs[2] = float(probs[2])
probs[3] = float(probs[3])

gama = float(lines[5])
epsilon = float(lines[6])

#given an action, this function computes the utility value of the state resulted from this action
def U_val(row_indx, column_indx, action):
  if (action == 0):
    p = row_indx+1
    q = column_indx
  if (action == 1):
    p = row_indx
    q = column_indx-1
  if (action == 2):
    p = row_indx-1
    q = column_indx
  if (action == 3):
    p = row_indx
    q = column_indx+1

  if (p==-1) or (p==N) or (q==-1) or (q==M):
    p = row_indx
    q = column_indx

  for n in range(len(wall)):
    if (p,q) == wall[n]:
      p = row_indx
      q = column_indx
  
  return U_prime[p][q]

#this function performs value iteration algorithm and returns the optimal policy
def Val_iteration():
  #initialize utility values table
  U_prime = np.zeros((N,M))
  for i in range(len(goal)):
    U_prime[goal[i][0]][goal[i][1]] = goal[i][2]
  for i in range(len(wall)):
    U_prime[wall[i][0]][wall[i][1]] = 0

  print(f'results for policy iteration algorithm:\nplease note that this U table is upside down, when the iterations are done I will print out the U table corresponding to the map of the grid world')
  for iteration in range (10):
  #while (True):
    U = U_prime
    delta = 0
    print(f'U = {U}')
    #compute utility value for each state
    for i in range(N):
      for j in range(M):
        up = probs[0]*U_val(i,j,0) + probs[1]*U_val(i,j,1) + probs[2]*U_val(i,j,3) + probs[3]*U_val(i,j,2)
        left = probs[0]*U_val(i,j,1) + probs[1]*U_val(i,j,0) + probs[2]*U_val(i,j,2) + probs[3]*U_val(i,j,3)
        down = probs[0]*U_val(i,j,2) + probs[1]*U_val(i,j,1) + probs[2]*U_val(i,j,3) + probs[3]*U_val(i,j,0)
        right = probs[0]*U_val(i,j,3) + probs[1]*U_val(i,j,0) + probs[2]*U_val(i,j,2) + probs[3]*U_val(i,j,1)
        U_prime[i][j] = reward + gama* max(up, left, down, right)
        #again change the utility values of goal cells and wall cells to their initila values
        for n in range(len(goal)):
          U_prime[goal[n][0]][goal[n][1]] = goal[n][2]
        for m in range(len(wall)):
          U_prime[wall[m][0]][wall[m][1]] = 0
      
        if abs(U_prime[i][j]-U[i][j]) > delta:
          delta = abs(U_prime[i][j]-U[i][j])

    #if delta > epsilon*(1-gama)/gama:
      #break

  #this part just transforms actions from number form to letter form
  action_list = ['U', 'L', 'D', 'R']
  action_dict = dict((i,c) for i, c in enumerate(action_list))
  pi_star = [[None for i in range(M)] for j in range(N)]
  for i in range(N):
    for j in range(M):
      up = probs[0]*U_val(i,j,0) + probs[1]*U_val(i,j,1) + probs[2]*U_val(i,j,3) + probs[3]*U_val(i,j,2)
      left = probs[0]*U_val(i,j,1) + probs[1]*U_val(i,j,0) + probs[2]*U_val(i,j,2) + probs[3]*U_val(i,j,3)
      down = probs[0]*U_val(i,j,2) + probs[1]*U_val(i,j,1) + probs[2]*U_val(i,j,3) + probs[3]*U_val(i,j,0)
      right = probs[0]*U_val(i,j,3) + probs[1]*U_val(i,j,0) + probs[2]*U_val(i,j,2) + probs[3]*U_val(i,j,1)
      pi_star[i][j] = action_dict[np.argmax([up, left, down, right])]

  for n in range(len(goal)):
    pi_star[goal[n][0]][goal[n][1]] = goal[n][2]
  for m in range(len(wall)):
    pi_star[wall[m][0]][wall[m][1]] = 'w'

  #print U table on the map of the grid world
  print(f'U on the map=')
  for i in range(N-1,-1,-1):
    print(f'{U[i]}')
  #print optimal policy on the map of the grid world
  print(f'optimal policy on the map=')
  for i in range(N-1,-1,-1):
    print(f'{pi_star[i]}')

#this function performs modified policy iteration algorithm and returns the optimal policy
def Policy_iteration():
  #initialize utility values table
  U_prime = np.zeros((N,M))
  for i in range(len(goal)):
    U_prime[goal[i][0]][goal[i][1]] = goal[i][2]
  for i in range(len(wall)):
    U_prime[wall[i][0]][wall[i][1]] = 0

  #initialize optimal policy table
  Pi = np.random.randint(4, size=(N,M))
  ## policy iteration
  print(f'results for policy iteration algorithm:\nplease note that this Pi table is upside down, when the iterations are done I will print out the Pi table corresponding to the map of the grid world')
  while (True):
    print(f'Pi = {Pi}')
    #compute utility value for each state given a certain policy for each state
    #performing some simplified value iteration steps, eq. 17.10 edition 3 of book
    for i in range(N):
      for j in range(M):
        up = probs[0]*U_val(i,j,0) + probs[1]*U_val(i,j,1) + probs[2]*U_val(i,j,3) + probs[3]*U_val(i,j,2)
        left = probs[0]*U_val(i,j,1) + probs[1]*U_val(i,j,0) + probs[2]*U_val(i,j,2) + probs[3]*U_val(i,j,3)
        down = probs[0]*U_val(i,j,2) + probs[1]*U_val(i,j,1) + probs[2]*U_val(i,j,3) + probs[3]*U_val(i,j,0)
        right = probs[0]*U_val(i,j,3) + probs[1]*U_val(i,j,0) + probs[2]*U_val(i,j,2) + probs[3]*U_val(i,j,1)
        if (Pi[i][j] == 0):
          U_prime[i][j] = reward + gama*up
        if (Pi[i][j] == 1):
          U_prime[i][j] = reward + gama*left
        if (Pi[i][j] == 2):
          U_prime[i][j] = reward + gama*down
        if (Pi[i][j] == 3):
          U_prime[i][j] = reward + gama*right

        #again change the utility values of goal cells and wall cells to their initila values
        for n in range(len(goal)):
          U_prime[goal[n][0]][goal[n][1]] = goal[n][2]
        for m in range(len(wall)):
          U_prime[wall[m][0]][wall[m][1]] = 0

    unchanged = True
    #find optimal policy for each state using the algorithm
    for i in range(N):
      for j in range(M):
        up = probs[0]*U_val(i,j,0) + probs[1]*U_val(i,j,1) + probs[2]*U_val(i,j,3) + probs[3]*U_val(i,j,2)
        left = probs[0]*U_val(i,j,1) + probs[1]*U_val(i,j,0) + probs[2]*U_val(i,j,2) + probs[3]*U_val(i,j,3)
        down = probs[0]*U_val(i,j,2) + probs[1]*U_val(i,j,1) + probs[2]*U_val(i,j,3) + probs[3]*U_val(i,j,0)
        right = probs[0]*U_val(i,j,3) + probs[1]*U_val(i,j,0) + probs[2]*U_val(i,j,2) + probs[3]*U_val(i,j,1)
        if Pi[i][j] == 0:
          SIG = up
        if Pi[i][j] == 1:
          SIG = left
        if Pi[i][j] == 2:
          SIG = down
        if Pi[i][j] == 3:
          SIG = right
        if max(up, left, down, right) > SIG:
          Pi[i][j] = np.argmax([up, left, down, right])
          unchanged = False

    if unchanged:
      break

  #this part just transforms actions from number form to letter form
  action_list = ['U', 'L', 'D', 'R']
  action_dict = dict((i,c) for i, c in enumerate(action_list))
  Pi = np.ndarray.tolist(Pi)
  for i in range(N):
    for j in range(M):
      Pi[i][j] = action_dict[Pi[i][j]]
  for n in range(len(goal)):
    Pi[goal[n][0]][goal[n][1]] = goal[n][2]
  for m in range(len(wall)):
    Pi[wall[m][0]][wall[m][1]] = 'w'

  #print optimal policy on the map of the grid world
  print(f'optimal policy on the map=')
  for i in range(N-1,-1,-1):
    print(f'{Pi[i]}')

Val_iteration()

Policy_iteration()