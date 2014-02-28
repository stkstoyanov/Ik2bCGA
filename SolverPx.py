import ctypes
import os
import inspect
import maya.OpenMaya as OpenMaya

fileName = inspect.getfile(inspect.currentframe())
currDir = os.path.dirname(os.path.abspath(fileName))
libFile = '/lib/libIk2bCGA.so'
lib = ctypes.CDLL(currDir + libFile)

class VectorPx(ctypes.Structure):
    _fields_ = [
                ('x', ctypes.c_double),
                ('y', ctypes.c_double),
                ('z', ctypes.c_double)
               ]

    def toMVector(self):
        return OpenMaya.MVector(self.x, self.y, self.z)


class QuaternionPx(ctypes.Structure):
    _fields_ = [
                ('x', ctypes.c_double),
                ('y', ctypes.c_double),
                ('z', ctypes.c_double),
                ('w', ctypes.c_double)
               ]

    def toMQuaternion(self):
        '''
        return OpenMaya.MQuaternion(
                                    self.w,
                                    OpenMaya.MVector(self.x, self.y, self.z)
                                   )
        '''
        return OpenMaya.MQuaternion(self.x, self.y, self.z, self.w)


class SolverPx(object):

    def __init__(self):
        pass


    def solveIK(self, shoulder, elbow, wrist, goal, pole):
        sh = VectorPx(shoulder.x, shoulder.y, shoulder.z)
        elb = VectorPx(elbow.x, elbow.y, elbow.z)
        wr = VectorPx(wrist.x, wrist.y, wrist.z)
        g = VectorPx(goal.x, goal.y, goal.z)
        p = VectorPx(pole.x, pole.y, pole.z)
        shRot = QuaternionPx()
        elbRot = QuaternionPx()

        lib.solveIK(
                    ctypes.byref(sh),
                    ctypes.byref(elb),
                    ctypes.byref(wr),
                    ctypes.byref(g),
                    ctypes.byref(p),
                    ctypes.byref(shRot),
                    ctypes.byref(elbRot)
                   )

        return (shRot.toMQuaternion(), elbRot.toMQuaternion())