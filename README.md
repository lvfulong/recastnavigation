
Recast & Detour
===============

[![Build](https://github.com/recastnavigation/recastnavigation/actions/workflows/Build.yaml/badge.svg)](https://github.com/recastnavigation/recastnavigation/actions/workflows/Build.yaml)
[![Tests](https://github.com/recastnavigation/recastnavigation/actions/workflows/Tests.yaml/badge.svg)](https://github.com/recastnavigation/recastnavigation/actions/workflows/Tests.yaml)

![screenshot of a navmesh baked with the sample program](/Docs/Images/screenshot.png)

## Recast

Recast is state of the art navigation mesh construction toolset for games.

Recast is...
* 🤖 **Automatic** - throw any level geometry at it and you will get a robust navmesh out
* 🏎️ **Fast** - swift turnaround times for level designers
* 🧘 **Flexible** - easily customize the navmesh generation and runtime navigation systems to suit your specific game's needs.

Recast constructs a navmesh through a multi-step rasterization process:

1. First Recast voxelizes the input triangle mesh by rasterizing the triangles into a multi-layer heightfield. 
2. Voxels in areas where the character would not be able to move are removed by applying simple voxel data filters.
3. The walkable areas described by the voxel grid are then divided into sets of 2D polygonal regions.
4. The navigation polygons are generated by triangulating and stiching together the generated 2d polygonal regions.

## Detour

Recast is accompanied by Detour, a path-finding and spatial reasoning toolkit. You can use any navigation mesh with Detour, but of course the data generated with Recast fits perfectly.

Detour offers a simple static navmesh data representation which is suitable for many simple cases.  It also provides a tiled navigation mesh representation, which allows you to stream of navigation data in and out as the player progresses through the world and regenerate sections of the navmesh data as the world changes.

## RecastDemo

You can find a comprehensive demo project in the `RecastDemo` folder. It's a kitchen sink demo showcasing all the functionality of the library. If you are new to Recast & Detour, check out [Sample_SoloMesh.cpp](/RecastDemo/Source/Sample_SoloMesh.cpp) to get started with building navmeshes and [NavMeshTesterTool.cpp](/RecastDemo/Source/NavMeshTesterTool.cpp) to see how Detour can be used to find paths.

### Building RecastDemo

RecastDemo uses [premake5](http://premake.github.io/) to build platform specific projects. Download it and make sure it's available on your path, or specify the path to it.

#### Linux

- Install SDL2 and its dependencies according to your distro's guidelines.
- Navigate to the `RecastDemo` folder and run `premake5 gmake2`
- Navigate to `RecastDemo/Build/gmake2` and run `make`
- Navigate to `RecastDemo/Bin` and run `./RecastDemo`

#### macOS

- Grab the latest SDL2 development library dmg from [here](https://github.com/libsdl-org/SDL) and place `SDL2.framework` in `RecastDemo/Bin`
- Navigate to the `RecastDemo` folder and run `premake5 xcode4`
- Open `Build/xcode4/recastnavigation.xcworkspace`
- Set the RecastDemo project as the target and build.

#### Windows

- Grab the latest SDL2 development library release from [here](https://github.com/libsdl-org/SDL) and unzip it `RecastDemo\Contrib`.  Rename the SDL folder such that the path `RecastDemo\Contrib\SDL\lib\x86` is valid.
- Navigate to the `RecastDemo` folder and run `premake5 vs2022`
- Open `Build/vs2022/recastnavigation.sln`.
- Set `RecastDemo` as the startup project, build, and run.

### Running Unit tests

- Follow the instructions to build RecastDemo above.  Premake should generate another build target called "Tests".
- Build the "Tests" project.  This will generate an executable named "Tests" in `RecastDemo/Bin/`
- Run the "Tests" executable.  It will execute all the unit tests, indicate those that failed, and display a count of those that succeeded.

## Integrating with your game or engine

See [Integration.md](Integration.md)

## Contribute

Check out the [Project Roadmap](Roadmap.md) to see what features and functionality you might be able to help with, and the [Contributing doc](CONTRIBUTING.md) for guidance on making contributions.

## Discuss

- Discuss Recast & Detour: http://groups.google.com/group/recastnavigation
- Development blog: http://digestingduck.blogspot.com/

## License

Recast & Detour is licensed under ZLib license, see License.txt for more information.
