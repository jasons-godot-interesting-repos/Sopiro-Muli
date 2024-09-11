# THIS REPO IS A GODOT 4.3 CSHARP PORT OF SOPIRO'S AABB_TREE

**See the `AabbTree.cs` in the root folder.   everything else is Sopiro's Multi repo, where the original cpp code is from.**

## why did I port this to csharp?

a c# implementation of this guy's Bounding Volume Hierarchy (BVH) https://github.com/Sopiro/DynamicBVH 

BVH has been around for a pretty long time, but over the last decade or so, it seems to have come out on top, choice wise, for game spatial partitions.  Probably because there's more info/analysis/comparison of different options over time, and this one's benefits make it well suited to general-purpose game spatial partitions  (better than sphere trees, quad trees, grids generally speaking).   Godot actually uses BVH for it's spatial partitioning already: https://github.com/godotengine/godot/blob/master/core/math/bvh.h  and it might have ben better for me to port that to csharp instead, but as it's used in a "real game engine" it has a lot of complexity added to it that I didn't want to sift through on my first try.

speaking of first tries, I originally ported this Unity BVH to godot, thinking that porting a csharp would save me time.   https://github.com/rossborchers/UnityBoundingVolumeHeirachy  alas, it seemed to be quite a waste: I got it "working" but in my first tests it fails miserably, with each node on the tree having the same bounding box as the root node.   Maybe i could have spent a few hours to figure it out, but a number of factors (suspiciously complex code mostly) lead me to try porting a cpp one.   The unity port took about half day, the cpp port took a full day.

for why I "need" this:  I don't use scenes, so need a way to track constructs, and my (spaceship building/combat) game is going to need a lot of raycasting, and the godot raycasting solution is actually very weak:  https://sampruden.github.io/posts/godot-is-not-the-new-unity/  so having a functional csharp spatial partition should allow me much greater performance and future optimization potential (like parallel queries) than the built-in system.

if you want to know the basics of a BVH, see https://en.wikipedia.org/wiki/Bounding_volume_hierarchy  and here's an interactive demo by the guy I ended up porting:  https://sopiro.github.io/DynamicBVH/

## screenshot of it with some visualizations care of [DD3D](https://dd3d.dmitriysalnikov.ru/docs/1.4.1/index.html)

![image](https://github.com/user-attachments/assets/8e7dfcfe-8dfb-4b36-81a2-95de5fbafbf1)





# Sopiro's original readme below

![logo](.github/logo.gif)

# Muli

[![Build library](https://github.com/Sopiro/Muli/actions/workflows/cmake-multi-platform.yml/badge.svg)](https://github.com/Sopiro/Muli/actions/workflows/cmake-multi-platform.yml)

2D Rigidbody physics engine

## Features  

### Collision  
  - Continuous collision detection (Bilateral advancement by Erin Catto of box2d)
  - Shapes: circle, capsule and convex polygon
  - Support for rounded polygons
  - Multiple shapes attached to a single body
  - Dynamic, static and kinematic bodies
  - Collision filtering
  - Dynamic AABB tree broadphase
  - Dynamic tree accelerated raycast, shapecast and world query
  - Easy-to-use collision detection and distance funtions
  
### Physics Simulation
  - Continuous physics simulation (Time of impact solver and sub-stepping)  
  - Efficient and persistent contact management from box2d
  - Constraint islanding and sleeping
  - Stable stacking with 2-contact LCP solver (Block solver)
  - Decoupled position correction iteration
  - Contact callbacks: begin, touching, end, pre-solve, post-solve and destroy event
  - Physics material: friction, restitution and surface speed
  - Various joints: angle, distance, grab, line, motor, prismatic, pulley, revolute and weld
  
### ETC
  - 50+ Demos
  - OpenGL based demo framework
  - Cross platform library
  - Intuitive and straightforward API
  - Utilizes specialized memory allocators
  
## Example

``` c++
#include "muli/muli.h"

using namespace muli;

int main()
{
    WorldSettings settings;
    World world(settings);
  
    RigidBody* box = world.CreateBox(1.0f);
    box->SetPosition(0.0f, 5.0f);
  
    // Run simulation for one second
    float dt = 1.0f / 60.0f;
    for(int i = 0; i < 60; ++i)
    {
        world.Step(dt);
    }
  
    return 0;
}
```

## Building and running
- Install [CMake](https://cmake.org/install/)
- Ensure CMake is in the system `PATH`
- Clone the repository `git clone --recursive https://github.com/Sopiro/Muli`
- Run CMake build script depend on your system
  - Visual Studio: Run `build.bat`
  - Otherwise: Run `build.sh`
- You can find the executable demo in the `build/bin`

## Installation

You can install the library using this commands

``` bat
mkdir build
cd build
cmake -DMULI_BUILD_DEMO=OFF ..
cmake --build . --config Release
cmake --install . --prefix "installation-path"
```

Assuming you've added "installation-path" to your system `PATH`, you can now integrate the library into your project

``` cmake
find_package(muli REQUIRED)

target_link_libraries(your-project PRIVATE muli::muli)
```

## Todo
- Implement position solve step for joints  
- Multithreading

## References
Here are some great resources to learn how to build a physics engine!
- https://www.toptal.com/game/video-game-physics-part-i-an-introduction-to-rigid-body-dynamics
- https://allenchou.net/game-physics-series/
- https://box2d.org/publications/
- https://www.cs.cmu.edu/~baraff/sigcourse/index.html
- https://dyn4j.org/blog/
