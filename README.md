Setting up the python plug-in:
------------------------------
```python
import maya.cmds as cmds
import maya.mel as mel

if not cmds.objExists('ik2bCGAsolver'):
    cmds.createNode('ik2bCGAsolver', n='ik2bCGAsolver')

mel.eval('ikUpdateSolverUI')
```
