# Maya Conformal Geometric Algebra Inverse Kinematics Solver

## Installation
Copy the root folder (Ik2bCGA) to the Maya plug-ins directory.
Create a file Ik2bCGA.py with the following code in it: 
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
