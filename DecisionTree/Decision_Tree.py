#Ava Mehri, UIC, amehri2@uic.edu, 2021
#Decision Tree Problem
#Decision tree algorithm from scratch, works only for datasets similar to the restaurant.txt toy dataset.


# -*- coding: utf-8 -*-


import sys
import numpy as np
from collections import Counter

#open the input file and store it
file = open('restaurant.txt', 'r').read()
#read the file line by line and store lines in a list
for line in file:
  lines = file.split('\n')
if (lines[-1] == ''):
    lines = lines[0:-1]
#store .cvs file into examples data structure
examples = []
for each in lines:
  temp = each.split(',')
  for i in range(len(temp)):
    temp[i] = temp[i].replace(' ', '')
  examples.append(temp)

Attributes = ['Alt', 'Bar', 'Fri', 'Hun', 'Pat', 'Price', 'Rain', 'Res', 'Type', 'Est']

class Node:
    def __init__(self, attribute, left, right):
        self.attribute = attribute
        self.left = left
        self.right = right

class Leaf:
    def __init__(self, samples):
        self.predictions = count(samples)

class Attribute:
    def __init__(self, Att_idx, value):
        self.Att_idx = Att_idx
        self.value = value

    def match(self, sample):
        # Compare the feature value in a sample to the feature value of the attribute
        val = sample[self.Att_idx]
        return val == self.value

    def __repr__(self):
        condition = "=="
        return "Is %s %s %s?" % (
            Attributes[self.Att_idx], condition, str(self.value))

#this function finds values corresponding to each attribute
def find_values(Attribute):
    return set(word for word in Attribute)

def partition(samples, attribute):
    true, false = [], []
    for sample in samples:
        if attribute.match(sample):
            true.append(sample)
        else:
            false.append(sample)
    return true, false

def entropy(y): #y is a vector of all class labels
  #y = y.tolist()
  hist = np.array([Counter(y)['Yes'], Counter(y)['No']])
  ps = hist/len(y)
  return -np.sum([p * np.log2(p) for p in ps if p>0])

def information_gain(left, right):
    left = np.array(left)
    right = np.array(right)
    parent_entropy = entropy(np.append(right[:,-1], left[:,-1]))
    if len(left) == 0 or len(right) == 0:
      return 0
    # weighted avg child E
    n_l, n_r = len(left), len(right)
    e_l, e_r = entropy(left[:,-1]), entropy(right[:,-1])
    n = n_l + n_r
    child_entropy = (n_l/n) * e_l + (n_r/n) * e_r

    ig = parent_entropy - child_entropy
    return ig

def split(samples):
    best_gain = 0  
    best_att = None  
    n_features = len(samples[0]) - 1  

    for col in range(n_features):
        values = set([sample[col] for sample in samples])
        for val in values: 
            test_att = Attribute(col, val)

            left, right = partition(samples, test_att)
            if len(left) == 0 or len(right) == 0:
                continue
                
            gain = information_gain(left, right)
            if gain >= best_gain:
                best_gain, best_att = gain, test_att

    return best_gain, best_att

def count(samples):
    counts = {}  # a dictionary to count the number of each type of example in a dataset
    for sample in samples:
        label = sample[-1]
        if label not in counts:
            counts[label] = 0
        counts[label] += 1
    return counts

def grow_tree(samples):
    gain, attribute = split(samples)

    # Base case: no further info gain
    # Since we can ask no further questions,
    # we'll return a leaf.
    if gain == 0:
      return Leaf(samples)

    true, false = partition(samples, attribute)

    # recursivley grow the branches
    left = grow_tree(true)
    right = grow_tree(false)

    return Node(attribute, left, right)

def print_tree(node, spacing=""):
    if isinstance(node, Leaf):
        print (spacing + "Predict", node.predictions)
        return

    print (spacing + str(node.attribute))
    print (spacing + '--> True:')
    print_tree(node.left, spacing + "  ")
    print (spacing + '--> False:')
    print_tree(node.right, spacing + "  ")

tree = grow_tree(examples)

print_tree(tree)
