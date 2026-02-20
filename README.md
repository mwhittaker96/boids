# Boids in Rust with EFrame and EGui
A quick implementation of the boids algorithm I wrote up over a weekend. I was most interested in learning the concepts of the algorithm itself and not as focused on performance or perfect simulation. 

https://github.com/user-attachments/assets/1e736085-674b-4086-b4b3-83027cd2e90d

## The Algorithm
Each Boid's movement is influence by the following
- Separation: How far apart it would like to be from nearby boids
- Alignment: How much the boid wants to move in the same direction as its neighbors
- Cohesion: How much the boid wants to move in closer to its neighbors
- Avoidance: The boid wants to avoid the 'predator' - shown in the simulation as a red circle on the cursor

For each of these parameters, we apply weights 

## The Interface
- The boid is represented as an arrow that points in the direction of the boid's velocity
- The boid's color indicates the current dominating affect on the boid's velocity (Cohesion - blue, Separation - yellow, Alignment - green, Avoidance - red)
- On the sidebar I provided myself with sliders to tweak parameters of the simulation in real time

## Future Improvements
- I think the boids should have collision so that they can't end up stacked on top of each other (see video below)
- Performance: there are definitely more distance calculations here than there ought to be - reducing square roots and duplicate calculations would probably be a big boost
- A lot of issues with spawning/despawning the boids which probably needs a closer look
- Would probably look at adding vision cones to boids to better simulate perception (right now they can see behind them/in 360 degrees)

https://github.com/user-attachments/assets/406a5f82-7ed4-4d4f-8ad1-52091c4e6736
