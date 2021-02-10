# -*- coding: utf-8 -*-
import os
from unittest.mock import MagicMock

def LAYOUT():
    return MagicMock()

def SYSTEM():
    return VRMSystem()

def LOG(obj):
    return

def CLEARLOG():
    return

def ImGui():
    return MagicMock()

class VRMSystem(object):
    def __init__(self):
        pass

    def GetLayoutPath(self):
        return os.path.abspath('./foo.bar')