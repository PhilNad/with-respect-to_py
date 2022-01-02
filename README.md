# With-Respect-To (+ Expressed-In)
Simple library that manages databases of 3D transformations with explicit accessors.

## Usage
```
usage: WRT [-h] --In world_name (--Get frame_name | --Set frame_name) --Wrt ref_frame_name --Ei in_frame_name [--As pose] [-v]

Manages a database of reference frames.

options:
  -h, --help            show this help message and exit
  --In world_name       The world name the frame lives in.
  --Get frame_name      Name of the frame to get.
  --Set frame_name      Name of the frame to set.
  --Wrt ref_frame_name  Name of the reference frame the frame is described with respect to.
  --Ei in_frame_name    Name of the reference frame the frame is expressed in.
  --As pose             If setting a frame, a string representation of the array defining the pose with rotation R and translation t: [[R00,R01,R02,t0],[R10,R11,R12,t1],[R20,R21,R22,t2],[0,0,0,1]]
  -v, --version         show program's version number and exit
```

## Goals
- Simple. A single 3D convention is used and is explicitly defined.
- Fast. The user should not worry about the overhead of calling accessors.
- Accessible. Information is accessible from a variety of interfaces on Linux (Python, Julia, Bash).
- Minimalist. The API contains as few methods as possible.

## Performances
Currently, a GET operation performed from within Python takes about 0.004 seconds to execute while a SET operation from within Python takes about 0.04 seconds -- 10x slower than GET.
When using the program from Bash, the Python first import some libraries, which takes a considerable amount of time. In particular, matplotlib and spatialmath are especially slow. In this scenario, a single GET/SET operation takes about 1.85 seconds to execute, which is prohibitively slow.

## Design
- Uses [Peter Corke's SpatialMath library](https://github.com/petercorke/spatialmath-python)
- Produces and consumes 4x4 transformation Numpy arrays AND/OR SpatialMath's SE(3) objects
- Store data in a SQLITE database using [sqlite3](https://docs.python.org/3/library/sqlite3.html)
- The scene is described by a tree
  - Re-setting a parent node, also changes the children nodes (i.e. assumes a rigid connection between parent and children)
  - If setting a transform would create a loop, the node is reassigned to a new parent
 
### Python Interface
- Connect: `SE3DB = WithRespectTo().In('my-world')`
- Getter: `SE3DB.Get('end-effector').Wrt('table').Ei('world')`
- Setter: `SE3DB.Set('end-effector').Wrt('table').Ei('world').As(spatialmath.SE3(...))`

### General Interface
Can be used by calling the python program with a single string as the argument. We could also support multiple arguments (--get, --set, --in, --wrt, --ei, --as)
- Get: `"In(my-world).Get(end-effector).Wrt(table).Ei(world)"`
- Set: `"In(my-world).Set(end-effector).Wrt(table).Ei(world).As([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])"`
