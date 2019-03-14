# Micromouse
For full installation instructions visit: [eniacluo/Micromouse](https://github.com/eniacluo/Micromouse/tree/master/framework)

## Micromouse Rendezous
 - The action function takes a goal and returns a sequence of best actions from the current position
  - This function eliminates sorts return paths based on its priority
 - The priority can be any hueristic, but for rendezvous it is defined as the distance from the goal.
 
 ## References
 - Source: http://wiki.ros.org/rrt_exploration
