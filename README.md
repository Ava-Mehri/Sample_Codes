# Explanation to my Sample Codes
GitHub: https://github.com/Ava-Mehri/Sample_Codes.git
## General guid on how to run the codes
1. Clone the repository on your local machine
2. Open the terminal and change the directory to the cloned folder on your machine
3. Create a new virtual environment (for example using venv: $ python -m venv testenv)
4. Activate it
    1. Linux: $ source testenv/bin/activate
    2. Windows: $ testenv/Scripts/activate.bat
5. $ pip install ipykernel
6. $ ipython kernel install --user --name=testenv
7. install the required dependecies
    1. $ pip install -r requirements.txt _**OR**_
    2. $ pip install numpy && pip install matplotlib
8. Open jupyter notebook
    1. [A guide on installing jupyter notebook](https://jupyter.org/install), in case it's not installed on your machine 
10. Change the directory in jupyter nb to the cloned folder
11. Choose any of .ipynb files 
12. Change the kernel to testenv
13. Simply run each cell
## Decision_Tree.ipynb
GitHub: https://github.com/Ava-Mehri/Sample_Codes/blob/main/Decision_Tree.ipynb
* Decision tree algorithm implemented from scratch
* The input file to run this code is [restauranr.txt](https://github.com/Ava-Mehri/Sample_Codes/blob/main/restaurant.txt)
* This code only works for the input files similar to restaurant.txt. I assumed all attributes' values are of string type, haven't consider the case in which attributes could have numerical values.
* For a different input file, change the list of Attributes in the code. (It is also mentioned in the code)
## Image_Compressor.ipynb
GitHub: https://github.com/Ava-Mehri/Sample_Codes/blob/main/Image_Compressor.ipynb
* An optimized image compressor from scratch implementing competitive learning algorithm
* The input to run this code is [barbara.png](https://github.com/Ava-Mehri/Sample_Codes/blob/main/barbara.png)
* Works for any input image
* The larger **K** value is, the less compressed the image is
## MDP.ipynb
GitHub: https://github.com/Ava-Mehri/Sample_Codes/blob/main/MDP.ipynb
* Solves a Markov decision process problem for a generic grid world using **Value Iteration Algorithm** and **Policy Iteration Algorithm**
* The input to run this code is [mdp_input.txt](https://github.com/Ava-Mehri/Sample_Codes/blob/main/mdp_input.txt)
* Works for any other grid world
* Input file lines:
    * The sizee of the world
    * Wall coordinates
    * Goal coordinates and their corresponding rewards
    * Reward per each step
    * Transition probabilities
    * Discount rate
    * Epsilon
 * The output is the optimal policy for each algorithm
 ## SubsetSum.ipynb
 GitHub: https://github.com/Ava-Mehri/Sample_Codes/blob/main/SubsetSum.ipynb
 * A dynamic-programming algorithm for the subset-sum problem which extacts:
    * The number of distinct subsets yielding the target sum.
    * The size of the smallest subset yielding the target sum and the number of such subsets.
    * The lexicographically first subset of the smallest size yielding the target sum.
 * The intput files to run this code are: 
    * [toy.txt](https://github.com/Ava-Mehri/Sample_Codes/blob/main/toy.txt)
    * [electoral.txt](https://github.com/Ava-Mehri/Sample_Codes/blob/main/electoral.txt)
    * [purple.txt](https://github.com/Ava-Mehri/Sample_Codes/blob/main/purple.txt)
## cpath.ipynb
GitHub: https://github.com/Ava-Mehri/Sample_Codes/blob/main/cpath.ipynb
* Determines the fastest way to get to a destination within a certain budget in a graph
* The test input to run this code is [g2.g](https://github.com/Ava-Mehri/Sample_Codes/blob/main/g2.g)
* Works for any other graph described in the same way as g2.g
## NoiseCanceller.ipynb
GitHub: https://github.com/Ava-Mehri/Sample_Codes/blob/main/NoiseCanceller.ipynb
* _**Not complete yet, will be updated**_
* .m version of this program works well but I haven't been successful to debug this file, still working on it
* It is supposed to remove a Gaussian noise from an audio signal
## bug_1.py and bug_2.py
* Bug1 GitHub: https://github.com/Ava-Mehri/Sample_Codes/blob/main/bug_1_.py
* Bug2 Github: https://github.com/Ava-Mehri/Sample_Codes/blob/main/bug_2_.py
* These programs need ROS(Robotic Operation System) and Gazebo simulation environment
* I haven't add instructions for running these programs here because they need an entire separate instruction manual and separate repository.
