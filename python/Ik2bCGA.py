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
        self._preferredElbows = {}


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

        # Set preferred angles
        shAngle = OpenMaya.MScriptUtil()
        shAngle.createFromDouble(0.0, 0.0, 0.0);
        shAnglePtr = shAngle.asDoublePtr()
        elbAngle = OpenMaya.MScriptUtil()
        elbAngle.createFromDouble(0.0, 0.0, 0.0);
        elbAnglePtr = elbAngle.asDoublePtr()

        fnShoulderJnt.getPreferedAngle(shAnglePtr)
        fnElbowJnt.getPreferedAngle(elbAnglePtr)
        fnShoulderJnt.setRotation(shAnglePtr, fnShoulderJnt.rotationOrder())
        fnElbowJnt.setRotation(elbAnglePtr, fnElbowJnt.rotationOrder())

        shoulder = fnShoulderJnt.rotatePivot(OpenMaya.MSpace.kWorld)
        elbow = fnElbowJnt.rotatePivot(OpenMaya.MSpace.kWorld)
        wrist = fnEffector.rotatePivot(OpenMaya.MSpace.kWorld)
        goal = fnIkHandle.rotatePivot(OpenMaya.MSpace.kWorld)
        pole = OpenMaya.MPoint(
                               self._getPlugValue(fnIkHandle, 'pvx'),
                               self._getPlugValue(fnIkHandle, 'pvy'),
                               self._getPlugValue(fnIkHandle, 'pvz')
                              )
        twistAngle = self._getPlugValue(fnIkHandle, 'twist')


        psign = self._getPreferredElbow(handlePath)
        if psign is None:
            psign = self._SOLVER.preferredElbow(shoulder,
                                                elbow,
                                                wrist,
                                                pole)
            self._addPreferredElbow(handlePath, psign)

        rotations = self._SOLVER.solveIK(shoulder,
                                         elbow,
                                         wrist,
                                         goal,
                                         pole,
                                         twistAngle,
                                         psign)
        fnElbowJnt.rotateBy(rotations[0], OpenMaya.MSpace.kWorld)
        fnShoulderJnt.rotateBy(rotations[1], OpenMaya.MSpace.kWorld)

        # Debugging Information
        '''
        self._printPoint('Shoulder', shoulder)
        self._printPoint('Elbow', elbow)
        self._printPoint('Wrist', wrist)
        self._printPoint('Goal', goal)
        '''


    def _getPreferredElbow(self, mDagPath):
        mObj = mDagPath.node()
        key = OpenMaya.MObjectHandle(mObj).hashCode()
        return  self._preferredElbows.get(key)


    def _addPreferredElbow(self, mDagPath, value):
        mObj = mDagPath.node()
        key = OpenMaya.MObjectHandle(mObj).hashCode()
        self._preferredElbows.update({key:value})


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
        angle = degrees(OpenMaya.MScriptUtil.getDouble(anglePtr))

        print signature, 'Axis: ', axis.x, axis.y, axis.z, 'Angle: ', angle
