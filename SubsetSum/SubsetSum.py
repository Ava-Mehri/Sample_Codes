#Ava Mehri, UIC, amehri2@uic.edu, 2021
#Subset Sum Problem
#A dynamic-programming algorithm for the subset-sum problem
#extracts the number of distinct subsets yielding the target sum,
#the size of the smallest subset yielding the target sum and the number of such subsets,
#and the lexicographically first subset of the smallest size yielding the target sum.


# -*- coding: utf-8 -*-


def main(Target, Input_File):
  #open the input file and store it
  file = open(Input_File, 'r').read()
  #read the file line by line and store lines in a list
  for line in file:
    lines = file.split('\n')
  if (lines[-1] == ''):
    lines = lines[0:-1]

  #N is the length of the array and T is the Target-sum
  N = len(lines)
  T = Target

  S = [[None for i in range(T+1)] for j in range(N)]
  Num_Feasible = [[0 for i in range(T+1)] for j in range(N)]
  Smallest_Size = [[0 for i in range(T+1)] for j in range(N)]
  Num_Smallest_Size = [[0 for i in range(T+1)] for j in range(N)]
  First_Min_Subset_Indx = [[[] for i in range(T+1)] for j in range(N)]


  for i in range(N):
    S[i][0] = True
    Num_Feasible[i][0] = 1
    Num_Smallest_Size[i][0] = 1

  for j in range(1,T+1):
    if (j == int(lines[0].split()[0])):
      S[0][j] = True
      Num_Feasible[0][j] = 1
      Smallest_Size[0][j] = 1
      Num_Smallest_Size[0][j] = 1
      First_Min_Subset_Indx[0][j].append(0)
    else:
      S[0][j] = False

  for i in range(1,N):
    a_i = int(lines[i].split()[0])
    for j in range(1,T+1):
      if (S[i-1][j] or ( a_i <= j and S[i-1][j-a_i] )):
        #populating table S[i][j], S[i][j] is True if there is at least 1 subset in A[0:i] such that sum of its elements
        # is j
        S[i][j] = True
        #populating table Num_Feasible[i][j], Num_Feasible[i][j] shows the number of subsets of the set A[0:i] such that
        # sum of elements of each of those subsets is j
        temp_num = 0
        if (a_i <= j):
          temp_num = Num_Feasible[i-1][j-a_i]        
        Num_Feasible[i][j] = Num_Feasible[i-1][j] + temp_num

        #populating table Smallest_Size[i][j], Smallest_Size[i][j] shows size of the smallest subset of the set A[0:i]
        # that sum of its elements is j
        if (S[i-1][j] and ( a_i <= j and S[i-1][j-a_i] )):
          Smallest_Size[i][j] = min(Smallest_Size[i-1][j], Smallest_Size[i-1][j-a_i]+1)
        elif (S[i-1][j]):
          Smallest_Size[i][j] = Smallest_Size[i-1][j]
        elif (a_i <= j and S[i-1][j-a_i] ):
          Smallest_Size[i][j] = Smallest_Size[i-1][j-a_i]+1

        #populating table Num_Smallest_Size[i][j], Num_Smallest_Size[i][j] shows the number of the smallest subsets of
        # the set A[0:i] which sum of elements of each of those subsets is j
        if (S[i-1][j] and ( a_i <= j and S[i-1][j-a_i] )):
          if (Smallest_Size[i-1][j] == Smallest_Size[i-1][j-a_i]+1):
            Num_Smallest_Size[i][j] = Num_Smallest_Size[i-1][j] + Num_Smallest_Size[i-1][j-a_i]
          elif (Smallest_Size[i-1][j] < Smallest_Size[i-1][j-a_i]+1):
            Num_Smallest_Size[i][j] = Num_Smallest_Size[i-1][j]
          else:
            Num_Smallest_Size[i][j] = Num_Smallest_Size[i-1][j-a_i]
        elif (S[i-1][j]):
          Num_Smallest_Size[i][j] = Num_Smallest_Size[i-1][j]
        elif (a_i <= j and S[i-1][j-a_i] ):
          Num_Smallest_Size[i][j] = Num_Smallest_Size[i-1][j-a_i]

        #populating table First_Min_Subset_Indx[i][j], First_Min_Subset_Indx[i][j] shows lexicographically first subset
        # of the set A[0:i] such that sum of its elements is j
        if (S[i-1][j] and ( a_i <= j and S[i-1][j-a_i] )):
          if (Smallest_Size[i-1][j] == Smallest_Size[i-1][j-a_i]+1):
            temp_First_1 = First_Min_Subset_Indx[i-1][j-a_i] + [i]
            temp_First_2 = First_Min_Subset_Indx[i-1][j]
            if (temp_First_1[0] < temp_First_2[0]):
              First_Min_Subset_Indx [i][j] = temp_First_1
            else:
              First_Min_Subset_Indx [i][j] = temp_First_2
          elif (Smallest_Size[i-1][j] < Smallest_Size[i-1][j-a_i]+1):
            First_Min_Subset_Indx [i][j] = First_Min_Subset_Indx[i-1][j]
          else:
            First_Min_Subset_Indx [i][j] = First_Min_Subset_Indx[i-1][j-a_i] + [i]

        elif (S[i-1][j]):
          First_Min_Subset_Indx [i][j] = First_Min_Subset_Indx[i-1][j]
        elif (a_i <= j and S[i-1][j-a_i] ):
          First_Min_Subset_Indx [i][j] = First_Min_Subset_Indx[i-1][j-a_i] + [i]

        

      else:
        S[i][j] = False

  Lexico_First = []
  for each in First_Min_Subset_Indx[-1][-1]:
    Lexico_First = Lexico_First + [[each, lines[each]]]

  print(f'Num_Elements = {N}')
  print(f'Target = {T}')
  print(f'Num_Feasible = {Num_Feasible[-1][-1]}')
  print(f'Smallest_Size = {Smallest_Size[-1][-1]}')
  print(f'Num_Smallest_Size = {Num_Smallest_Size[-1][-1]}')
  print(f'Lexico_First Subset = ')
  for i in range(len(Lexico_First)):
    print(f'{Lexico_First[i]}')

#main(5,'toy.txt')

#main(269,'electoral.txt')

#main(220,'purple.txt')

#main(121,'purple.txt')
