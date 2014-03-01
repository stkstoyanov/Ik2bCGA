import sys
from math import degrees
import maya.OpenMaya as OpenMaya
import maya.OpenMayaAnim as OpenMayaAnim
import maya.OpenMayaMPx as OpenMayaMPx
from SolverPx import SolverPx

class Ik2bCGA(OpenMayaMPx.MPxIkSolverNode):

    NAME = 'ik2bCGAsolver'
    ID = OpenMaya.MTypeId(0x80100)
    TYPE = OpenMayaMPx.MPxNode.kIkSolverNode

    _SOLVER = SolverPx()

    def __init__(self):
        OpenMayaMPx.MPxIkSolverNode.__init__(self)


    @staticmethod
    def create():
        return OpenMayaMPx.asMPxPtr(Ik2bCGA())


    @staticmethod
    def initialize():
        pass


    @classmethod
    def register(cls, plugin):
        try:
            plugin.registerNode(
                                cls.NAME,
                                cls.ID,
                                cls.create,
                                cls.initialize,
                                cls.TYPE
                               )
        except:
            sys.stderr.write("Failed to register node: " + cls.NAME)
            raise


    @classmethod
    def deregister(cls, plugin):
        try:
            plugin.deregisterNode(cls.ID)
        except:
            sys.stderr.write("Failed to unregister node: " + cls.NAME)
            raise


    def solverTypeName(self):
        return self.NAME


    def doSolve(self):
        self._cgaSolve()


    def _cgaSolve(self):
        # IK Handle
        handleGroup = self.handleGroup()
        handle = handleGroup.handle(0)
        handlePath = OpenMaya.MDagPath.getAPathTo(handle)
        fnIkHandle = OpenMayaAnim.MFnIkHandle(handlePath)

        # Goal Effector
        effector = OpenMaya.MDagPath()
        fnIkHandle.getEffector(effector)
        fnEffector = OpenMayaAnim.MFnIkEffector(effector)

        # Shoulder Joint
        startJoint = OpenMaya.MDagPath()
        fnIkHandle.getStartJoint(startJoint)
        fnShoulderJnt = OpenMayaAnim.MFnIkJoint(startJoint)

        # Mid Joint
        effector.pop()
        fnElbowJnt = OpenMayaAnim.MFnIkJoint(effector)

        # Shoulder Joint Inverse Matrix
        shMatrix = fnShoulderJnt.transformation().asMatrixInverse()

        shoulder = fnShoulderJnt.rotatePivot(OpenMaya.MSpace.kWorld) * shMatrix
        elbow = fnElbowJnt.rotatePivot(OpenMaya.MSpace.kWorld) * shMatrix
        wrist = fnEffector.rotatePivot(OpenMaya.MSpace.kWorld) * shMatrix
        goal = fnIkHandle.rotatePivot(OpenMaya.MSpace.kWorld) * shMatrix
        pole = OpenMaya.MPoint(
                               self._getPlugValue(fnIkHandle, 'pvx'),
                               self._getPlugValue(fnIkHandle, 'pvy'),
                               self._getPlugValue(fnIkHandle, 'pvz')
                              ) * shMatrix

        rotations = self._SOLVER.solveIK(shoulder, elbow, wrist, goal, pole)
        #fnShoulderJnt.rotateBy(rotations[0], OpenMaya.MSpace.kTransform)
        #fnElbowJnt.rotateBy(rotations[1], OpenMaya.MSpace.kTransform)

        '''
        # Debugging Information
        self._printPoint('Shoulder', shoulder)
        self._printPoint('Elbow', elbow)
        self._printPoint('Wrist', wrist)
        self._printPoint('Goal', goal)
        self._printRotation('Shoulder ', rotations[0])
        self._printRotation('Elbow ', rotations[1])
        '''


    def _getPlugValue(self, dagNode, attribute):
        plug = dagNode.findPlug(attribute)
        return plug.asDouble()


    def _printPoint(self, signature, point):
        print signature, point.x, point.y, point.z


    def _printRotation(self, signature, quaternion):
        util = OpenMaya.MScriptUtil()
        util.createFromDouble(0.0)
        anglePtr = util.asDoublePtr()
        axis = OpenMaya.MVector()
        quaternion.getAxisAngle(axis, anglePtr)
        angle = OpenMaya.MScriptUtil.getDouble(anglePtr)

        print
        signature,
        'Axis: ',
        axis.x,
        axis.y,
        axis.z,
        'Angle: ',
        degrees(angle)
