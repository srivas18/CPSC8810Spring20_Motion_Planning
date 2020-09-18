CPSC 8810 - Motion Planning
ASSIGNMENT 3: Discrete Planning
Submitted by: Aakanksha Smriti, Siddhant Srivastava
Submission date: 02-25-2020

Variable Description:
1. current_node = Parent node for a particular instance
2. parent_node = Parent node to create path
3. parchi_diff = Theta difference between parent and child
4. index = Storing index values of 'action' matrix for every iteration
5. actions = Storing each 'action' matrix values for every iteration
6. theta = Calculating and storing the theta values for the child at every instance
7. neighbors = Generating neighbors of parent node every instance
8. address = Storing the f and g values of the parent node at that particular instance

Output - (astar.py)

1. cost = [1, 1, 1]

['-', '-', '-', '-', '-', '-']
['-', '-', '-', '-', '-', '-']
['*', 'F', 'F', 'L', '-', '-']
['-', '-', '-', 'F', '-', '-']
['-', '-', '-', 'F', '-', '-']

Expanded Nodes
(4, 3, 0)
(3, 3, 0)
(2, 3, 0)
(2, 2, 1)
(2, 1, 1)
(2, 0, 1)

2. cost = [1, 1, 10]

['-', '-', '-', 'R', 'F', 'R']
['-', '-', '-', 'F', '-', 'F']
['*', 'F', 'F', 'F', 'F', 'R']
['-', '-', '-', 'F', '-', '-']
['-', '-', '-', 'F', '-', '-']

Expanded Nodes
(4, 3, 0)
(3, 3, 0)
(2, 3, 0)
(2, 4, 3)
(1, 3, 0)
(2, 5, 3)
(0, 3, 0)
(0, 4, 3)
(0, 5, 3)
(1, 5, 2)
(2, 5, 2)
(2, 4, 1)
(2, 3, 1)
(2, 2, 1)
(2, 1, 1)
(2, 0, 1)