# Rapidly-Exploring_Random_Tree
Rapidly-Exploring Random Tree for path planning

Implementation of Rapidly-Exploring Random Tree (RRT), a path planning algorithm developed by Steven LaValle (see [reference][1]).

## Task1: Simple RRT
The first task involves generating an RRT in a 2D domain given an initial configuration. See an example run below:
![simpleRRT](https://user-images.githubusercontent.com/60046203/190181511-b4fe9eed-2355-42eb-8e30-6166d1c58de9.png)

## Task2: RRT with circular obstacles
The second task involves generating a set of random circular obstacles and using RRT to find a path between a start and goal location. See an example run below: 
![RRT_path_planning_with_obstacle](https://user-images.githubusercontent.com/60046203/190182319-b4712865-0567-47c0-b752-3326c72a8259.png)

## Task3: RRT with arbitrary obstacles
The third task involves reading an arbitrary obstacle from an image and using RRT to find a path between a start and goal location. See an example run below using the Northwestern "N" obstacle:
![RRT_arbitrary_obstacle_gray](https://user-images.githubusercontent.com/60046203/190182778-bfc296f8-f000-495b-b796-5c456d697e9a.png)

[1]: http://msl.cs.illinois.edu/~lavalle/papers/Lav98c.pdf    "RRT paper reference"
