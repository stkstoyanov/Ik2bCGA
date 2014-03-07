# Maya CGA IK Solver
This is a two-bone inverse kinematics solver based on a conformal geometric
 algebra algorithm.

## Installation
Copy the root folder (Ik2bCGA) to the Maya plug-ins directory.
Create a file Ik2bCGA.py in the same directory with the following code in it: 
```python
from Ik2bCGA import *
```

## Setting up the python plug-in
```python
import maya.cmds as cmds
import maya.mel as mel

if not cmds.objExists('ik2bCGAsolver'):
    cmds.createNode('ik2bCGAsolver', n='ik2bCGAsolver')

mel.eval('ikUpdateSolverUI')
```

## Compiling the plugin
The plug-in depends on the Gaalop C++ precompiler, which can be downloaded from
here: http://www.gaalop.de/download/

Once you configure your CMake build, just type the following in your console:
```
make gaalop_solver
```
