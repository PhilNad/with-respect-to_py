# With-Respect-To (+ Expressed-In)
Simple library that manages databases of 3D transformations with explicit accessors.

## Goals
- Simple. A single 3D convention is used and is explicitly defined.
- Fast. The user should not worry about the overhead of calling accessors.
- Accessible. Information is accessible from a variety of interfaces on Linux (Python, Julia, Bash).
- Minimalist. The API contains as few methods as possible.

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
