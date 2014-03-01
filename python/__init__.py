import maya.OpenMayaMPx as OpenMayaMPx
from Ik2bCGA import *

AUTHOR='Stoyan Stoyanov'
VERSION='1.0'

def initializePlugin(mObject):
    mPlugin = OpenMayaMPx.MFnPlugin(mObject, AUTHOR, VERSION)
    Ik2bCGA.register(mPlugin)

def uninitializePlugin(mObject):
    mPlugin = OpenMayaMPx.MFnPlugin(mObject)
    Ik2bCGA.deregister(mPlugin)
