# SdfTaskTemplate
Template for task 1 (Signed Distance Fields). Part of Forward and Inverse Rendering (FAIR) course, CMC MSU, Spring 2025.

## Build
Clone this repo with its submodules:

    git clone https://github.com/SammaelA/SdfTaskTemplate.git
    cd SdfTaskTemplate
    git submodule update --init

Install SDF (Ubuntu 22)

    sudo apt-get install libsdl2-2.0-0

- For installations on other platforms see https://wiki.libsdl.org/

Build the executable:

    cmake -B build && cmake --build build

## Execute

    ./render

Template visualizes one layer of an SDF grid (example_grid.bin, mode of a bunny)  
use W and S keys to swich between layers.

## Contents

This repository contains several things useful for working on the task.

- Small maths library with vertices, matrices and a set of useful functions (LiteMath/LiteMath.h)
- Functions for saving and loading images (LiteMath/Image2d.h)
- SimpleMesh, data structure to store triangle mesh, and functions to load it from .obj files (mesh.h)
- Data structures for SDF grid and octree, functions to save and load them (main.cpp)
- A template for your application: creating window with SDF, handling keyboard input, rendering to the window (main.cpp)
- A set of test meshes (cube.obj, as1-oc-214.obj, MotorcycleCylinderHead.obj, spot.obj, stanford-bunny.obj)
  All these models are watertight and can be converted to SDF without issues
- A set of test SDFs (example_grid.grid, example_grid_large.grid, example_octree_large.octree)