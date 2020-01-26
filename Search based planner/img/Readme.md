# <center>Search-Based Path Finding</center>
## Introduction
This report covers an assignment on implementing A* algorithm on ROS and matlab environment. Different metrics and techniques are used to test the performance of the algorithm.
## Implementation in ROS 

#### Algorithm Pipeline
+ Maintain a openSet to store all the nodes to be expanded
+ the openSet is initialized with the start state Xs
+ Assgin g(Xs)=0, and g(n)=inifite for all other nodes in the graph
+ Loop
  + If openSet is empty, return FALSE; break
  + Remove the node "n" with the lowest f(n)=g(n)+h(n) from openSet
  + Mark node "n" as expanded
  + If the node "n" is the goal state, record the node as terminatePtr; break;
  + For all unexpanded neighbors "m" of node "n"
    + if g(m) = infinite ("m" is new discovered)
      + g(m) = g(n) + Cnm
      + Record the parent node of "m", m->cameFrom = n
      + Push node "m" into openSet
    + if g(m) > g(n) + Cnm
      + g(m) = g(n) + Cnm
      + m->cameFrom = n
      + m->f(n) = g(n) + h(m)
  + end
+ End Loop
+ Optimal path can be found by recursively searching the parent node of terminatePtr

####Result in 3D Case
![](/home/lixing/path-planning/Document/img/3d.jpg "3D Case")
  

## Performance Analysis
I explored the influence of heuristic and tie-breaker to the performance of A* algorithm. I ran experiments in a non-obstacle environment for eliminating the bias caused by random maps. Wesee the quadratic of euclidean distance works better than other heuristics. The reason could be that the greediness of the heuristic determines the time consuming of the motion planning process, especially in a non-obstcale environment. To confirm that I used a large random scale as tie-breaker, so that the heu value is far greater than g(n). The result is quite interesting, it took only 1.31ms to find the path

####*Without Tie-Breaker*
![](/home/lixing/path-planning/Document/img/withoutTieBreaker.png)

####*With Tie-Breaker*
![](/home/lixing/path-planning/Document/img/withtie_breaker.png "3D Case")

####*Result*
![](/home/lixing/path-planning/Document/img/Result.png "3D Case")

|Method| Heuristic | Tie breaker | Visited nodes | Running time |
|:----:|:----:|:----:|:----:|:----:|
| A* | Manhattan | True  | 7341 | 2921ms |
| A* | Manhattan | False  |  7568 | 2941ms |
| A* | Euclidean | True | 5139 | 1540ms |
| A* | Euclidean | False | 6641 | 2441ms |
| A* | Euclidean | True and with large scale | 55 | 1.325ms |
| A* | Euclidean*2 | True  | 2540 | 320ms |
| A* | Euclidean*2 | False | 2639 | 384ms |
| A* | Diagonal | True  | 5660 | 1939ms |
| A* | Diagonal | False | 5984 | 1958ms |

## Implementation in MATLAB

![](/home/lixing/path-planning/Document/img/1.PNG "3D Case")
![](/home/lixing/path-planning/Document/img/2.PNG "3D Case")
![](/home/lixing/path-planning/Document/img/3.PNG "3D Case")

