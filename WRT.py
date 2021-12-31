import numpy as np
import sqlite3
import sympy
from spatialmath import SE3
from anytree import NodeMixin, RenderTree


class PoseNode(SE3, NodeMixin):
    '''Describe a node in the pose graph.'''
    def __init__(self, frame_name: str, relative_pose: SE3, parent=None):
        super(SE3, self).__init__()
        self.name           = frame_name
        self.relative_pose  = relative_pose
        self.parent         = parent

class SetAs:
    def __init__(self, frame_name:str, ref_frame_name:str, in_frame_name:str) -> None:
        self.__frame_name     = frame_name
        self.__ref_frame_name = ref_frame_name
        self.__in_frame_name  = in_frame_name
    def As(self, transfo_matrix):
        assert(type(transfo_matrix) == np.ndarray or type(transfo_matrix) == SE3)
        if type(transfo_matrix) is np.array:
            assert(transfo_matrix.shape == (4,4))
        #1) Make sure that the __ref_frame_name exists in the DB
        #2) Remove from DB any frame with  __frame_name
        #2) Add new frame in the DB

class GetExpressedIn:
    def __init__(self, frame_name:str, ref_frame_name:str) -> None:
        self.__frame_name     = frame_name
        self.__ref_frame_name = ref_frame_name
    def Ei(self, in_frame_name: str) -> SE3:
        #Using Drake's monogram notation (https://drake.mit.edu/doxygen_cxx/group__multibody__notation__basics.html)

        #1) Make sure __frame_name, __ref_frame_name and in_frame_name exist in the DB
        #2) Get __frame_name     WRT world EI world
        X_WF_W = SE3()
        #3) Get __ref_frame_name WRT world EI world
        X_WR_W = SE3()
        X_RW_W = SE3(-X_WR_W.t[0], -X_WR_W.t[1], -X_WR_W.t[2])
        #4) Get in_frame_name    WRT world EI world
        X_WI_W = SE3()
        X_IW_I = X_WI_W.inv()
        #5) Compute the __frame_name WRT __ref_frame_name EI world
        X_RF_W = X_RW_W @ X_WF_W
        #6) Change the "expressed in"
        mat = np.eye(4)
        mat[0:3, 0:3] = X_IW_I @ X_RF_W
        mat[0:3, 3]   = X_IW_I @ X_RF_W.t
        X_RF_I = SE3(mat)
        return X_RF_I

class SetExpressedIn:
    def __init__(self, frame_name:str, ref_frame_name:str) -> None:
        self.__frame_name     = frame_name
        self.__ref_frame_name = ref_frame_name
    def Ei(self, in_frame_name: str) -> SetAs:
        return SetAs(self.__frame_name, self.__ref_frame_name, in_frame_name)

class Setter:
    def __init__(self, frame_name:str) -> None:
        self.__frame_name = frame_name
    def Wrt(self, ref_frame_name: str) -> SetExpressedIn:
        return SetExpressedIn(self.__frame_name, ref_frame_name)

class Getter:
    def __init__(self, frame_name:str) -> None:
        self.__frame_name = frame_name
    def Wrt(self, ref_frame_name: str) -> GetExpressedIn:
        return GetExpressedIn(self.__frame_name, ref_frame_name)

class GetSet:
    def Get(self, frame_name: str) -> Getter:
        return Getter(frame_name)
    def Set(self, frame_name: str) -> Setter:
        return Setter(frame_name)

class DbConnector:
    '''Connect to Sqlite database and create new world if needed.'''
    def __init__(self):
        return
    def In(self, worldName: str) -> GetSet:
        return GetSet()

db = DbConnector()
db.In('my-world').Get('table').Wrt('world').Ei('world')