## Breadth First Search for 2D Map

# General

The main file for this project is called "BreadthFirstSearch", the start and goal points are given in line #15&#16, feel free to change them.
"isValid" is the file to detect whether the new point is in the free space - in another word, valid or not.
"isValid2" is the file uses half-plane models. Feel free to change the name of "isValid" to anything else and change the "isValid2" to "isValid" to see the results.
"DoublyLinkedList", "Node", "Queue", "Stack" are the online source of a Queue. As we all know, Matlab has some drawbacks about it which needs some compensation.

# Output
The output is a figure showing searched points one by one in green color and connecting the optimal path in red color in the end. The picture is strat from [145, 13] to [185, 57]. There's something wrong with the saved output file in .fig format, it should be looked like the FinalPath.jpg.

# Other
The license file is for the queue and stack classes, please don't delete it.

# Run
Open "BreadthFirstSearch.m" file and click F5 to run. If you want to change the start and goal points, please make that change in the line #15&#16 and run again.
Enjoy the figure output even it runs slowly because I didn't apply a cost function.