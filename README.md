# Collision Detection Project

The goal of the project is to implement a performant collision detection algorithm which composes of a Broad phase, a Narrow phase, and a collision response.


## Features states

**Broad phase** : Completed, I implemented the Sweep and Prune algorithm because it is the most performant algorithm for 2D collision detection and is very easy to implement. I also for experiment purposes improve the Brut force algorithm. All broad phase algorithm inherits from the Broad phase interface and can be founded in the Broad Phase folder.

**Narrow phase** : Completed, I implemented my own version of the GJK and EPA algorithms. 

**Debug tools** : I also created an optimized AABB creator and drawer to create/modify AABB only when needed to save performance.
In addition to the optimized AABB creator/drawer, i implemented new inputs to help visualize collision elements:
-F8 to open the draw debug element menu + visualization.
-Numpad 1,2,3 to toggle some elements which should be drawn or not.
those toggled elements are :
-AABB colliding state (blue == not colliding, green == colliding).
-minkowski shape and minkowski creation.
-GJK last simplex and collision hit result (the purple line).

The base color for colliding shape is the same as the AABB (blue == not colliding, green == colliding). 

## Clips
**Broad phase**
![001](./SCREENS/Gifs/001.gif)
![002](./SCREENS/Gifs/002.gif)
![003](./SCREENS/Gifs/003.gif)
![004](./SCREENS/Gifs/004.gif)
![005](./SCREENS/Gifs/005.gif)

**Narrow phase**
![001](./SCREENS/Gifs/006.gif)
![002](./SCREENS/Gifs/007.gif)
![003](./SCREENS/Gifs/008.gif)
![004](./SCREENS/Gifs/009.gif)
![005](./SCREENS/Gifs/010.gif)

> Note: All gifs/clips were taken with gyazo, as so the framerate is low to save on file size.
