# Micromouse
For full installation instructions visit: [eniacluo/Micromouse](https://github.com/eniacluo/Micromouse/tree/master/framework)

## Micromouse Rendezous
 - The action function takes a goal and returns a sequence of best actions from the current position
  - This function eliminates sorts return paths based on its priority
 - The priority can be any hueristic, but for rendezvous it is defined as the distance from the goal.
 - The action stategy initializes the gaol at the near neighbor, once this goal is attained the robots decide a group head 
and follower, wherby the objectives become: go to furthest neighbor, and follow head respectively.
- Can reach rendezvous in less than 5 seconds. 
- Simulation often sticks due to packet loss 
 
 ## References
 - Source: http://wiki.ros.org/rrt_exploration
