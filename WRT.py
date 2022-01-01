import numpy as np
import sqlite3 as sql
import re
import sympy #spatialmath depends on sympy
import matplotlib.pyplot as plt
from spatialmath import SE3
from spatialmath.base import *
from anytree import NodeMixin, RenderTree
from pathlib import Path


class PoseNode(SE3, NodeMixin):
    '''Describe a node in the pose graph.'''
    def __init__(self, frame_name: str, relative_pose: SE3, parent=None):
        super(SE3, self).__init__()
        self.name           = frame_name
        self.relative_pose  = relative_pose
        self.parent         = parent

class Visualize():
    '''Visualize all frames in the database using Matplotlib.'''
    def __init__(self, worldName:str) -> None:
        con = sql.connect(worldName+".db")
        with con:
            frame_names = [row[0] for row in con.execute("SELECT name FROM frames")]
            db = DbConnector()
            for fn in frame_names:
                X_WF_W = db.In(worldName).Get(fn).Wrt('world').Ei('world')
                trplot( X_WF_W.A, frame=fn)
            plt.show()

class SetAs:
    def __init__(self, sqlite_connection: sql.Connection, frame_name:str, ref_frame_name:str, in_frame_name:str) -> None:
        self.__verifyInput(frame_name)
        self.__verifyInput(ref_frame_name)
        self.__verifyInput(in_frame_name)
        self.__frame_name     = frame_name
        self.__ref_frame_name = ref_frame_name
        self.__in_frame_name  = in_frame_name
        self.__sql_connection = sqlite_connection
    def __verifyInput(self, input_string:str):
        if not re.match("^[0-9a-z\-]+$", input_string):
            raise ValueError("Only [a-z], [0-9] and dash (-) is allowed in the frame name: {}".format(input_string))
    def As(self, transfo_matrix):
        assert(type(transfo_matrix) == np.ndarray or type(transfo_matrix) == SE3)
        if type(transfo_matrix) is np.ndarray:
            if transfo_matrix.shape != (4,4):
                raise ValueError("The shape of the transformation matrix should be (4,4) but we got {}".format(transfo_matrix.shape))
            else:
                transfo_matrix = SE3(transfo_matrix)
        
        with self.__sql_connection as con:
            #1) Make sure that the __ref_frame_name exists in the DB
            rows = [row for row in con.execute("SELECT * FROM frames WHERE name IS ?",(self.__ref_frame_name,))]
            if len(rows) == 0:
                raise ValueError("The reference frame '{}' does not exist in this world.".format(self.__ref_frame_name))
            #The name field is a UNIQUE field in the database, consequently not more than one row should be returned.
            assert(len(rows) == 1)
            #2) Remove from DB any frame with  __frame_name
            con.execute("DELETE FROM frames WHERE name IS ?",(self.__frame_name,))
            #3) Take into account the fact that the transformation can be expressed in a frame different from the reference frame
            #       Like: SET object WRT table EI world
            getter = GetExpressedIn(self.__sql_connection, self.__in_frame_name, self.__ref_frame_name)
            X_RI_R = getter.Ei(self.__ref_frame_name)
            R = X_RI_R.R @ transfo_matrix.R
            t = X_RI_R.R @ transfo_matrix.t
            #4) Add new frame in the DB
            con.execute("INSERT INTO frames VALUES (?, ?, ?,?,?, ?,?,?, ?,?,?, ?,?,?)", 
                (self.__frame_name, 
                self.__ref_frame_name, 
                R[0,0],R[0,1],R[0,2], 
                R[1,0],R[1,1],R[1,2], 
                R[2,0],R[2,1],R[2,2], 
                t[0],t[1],t[2]))

class GetExpressedIn:
    def __init__(self, sqlite_connection: sql.Connection, frame_name:str, ref_frame_name:str) -> None:
        self.__verifyInput(frame_name)
        self.__verifyInput(ref_frame_name)
        self.__frame_name     = frame_name
        self.__ref_frame_name = ref_frame_name
        self.__sql_connection = sqlite_connection
    def __verifyInput(self, input_string:str):
        if not re.match("^[0-9a-z\-]+$", input_string):
            raise ValueError("Only [a-z], [0-9] and dash (-) is allowed in the frame name: {}".format(input_string))
    def __getParentFrame(self, frame_name:str) -> tuple:
        '''Return the parent frame of frame_name if one exists, otherwise return None.'''
        with self.__sql_connection as con:
            rows = [row for row in con.execute("SELECT * FROM frames WHERE name IS ?",(frame_name,))]
            if len(rows) == 0:
                raise ValueError("The reference frame '{}' does not exist in this world.".format(frame_name))
            #The name field is a UNIQUE field in the database, consequently not more than one row should be returned.
            assert(len(rows) == 1)
            
            #Load data
            row = rows[0]
            name   = row[0]
            parent = row[1]

            R00    = row[2]
            R01    = row[3]
            R02    = row[4]
            
            R10    = row[5]
            R11    = row[6]
            R12    = row[7]
            
            R20    = row[8]
            R21    = row[9]
            R22    = row[10]

            t0     = row[11]
            t1     = row[12]
            t2     = row[13]

            #Build a transformation matrix
            mat = np.eye(4)
            mat[0,0:3] = [R00,R01,R02]
            mat[1,0:3] = [R10,R11,R12]
            mat[2,0:3] = [R20,R21,R22]
            mat[0:3,3] = [t0,t1,t2]
            transfo = SE3(mat)

            return (name, parent, transfo)

    def __poseWrtWorld(self, frame_name:str) -> SE3:
        '''Return the pose of frame_name relative to world reference frame, expressed in world.'''
        (name, parent, new_transfo) = self.__getParentFrame(frame_name)
        while parent is not None:
            (name, parent, transfo) = self.__getParentFrame(parent)
            new_transfo = transfo @ new_transfo
        return new_transfo

    def Ei(self, in_frame_name: str) -> SE3:
        self.__verifyInput(in_frame_name)
        #Using Drake's monogram notation (https://drake.mit.edu/doxygen_cxx/group__multibody__notation__basics.html)

        #1) Make sure __frame_name, __ref_frame_name and in_frame_name exist in the DB
        #2) Get __frame_name     WRT world EI world
        X_WF_W = self.__poseWrtWorld(self.__frame_name)
        #3) Get __ref_frame_name WRT world EI world
        X_WR_W = self.__poseWrtWorld(self.__ref_frame_name)
        X_RW_W = SE3(-X_WR_W.t[0], -X_WR_W.t[1], -X_WR_W.t[2])
        #4) Get in_frame_name    WRT world EI world
        X_WI_W = self.__poseWrtWorld(in_frame_name)
        X_IW_I = X_WI_W.inv()
        #5) Compute the __frame_name WRT __ref_frame_name EI world
        X_RF_W = X_RW_W @ X_WF_W
        #6) Change the "expressed in"
        mat = np.eye(4)
        mat[0:3, 0:3] = X_IW_I.R @ X_RF_W.R
        mat[0:3, 3]   = X_IW_I.R @ X_RF_W.t
        X_RF_I = SE3(mat)
        return X_RF_I

class SetExpressedIn:
    def __init__(self, sqlite_connection: sql.Connection, frame_name:str, ref_frame_name:str) -> None:
        self.__frame_name     = frame_name
        self.__ref_frame_name = ref_frame_name
        self.__sql_connection = sqlite_connection
    def Ei(self, in_frame_name: str) -> SetAs:
        return SetAs(self.__sql_connection, self.__frame_name, self.__ref_frame_name, in_frame_name)

class Setter:
    def __init__(self, sqlite_connection: sql.Connection, frame_name:str) -> None:
        self.__frame_name = frame_name
        self.__sql_connection = sqlite_connection
    def Wrt(self, ref_frame_name: str) -> SetExpressedIn:
        return SetExpressedIn(self.__sql_connection, self.__frame_name, ref_frame_name)

class Getter:
    def __init__(self, sqlite_connection: sql.Connection, frame_name:str) -> None:
        self.__frame_name = frame_name
        self.__sql_connection = sqlite_connection
    def Wrt(self, ref_frame_name: str) -> GetExpressedIn:
        return GetExpressedIn(self.__sql_connection, self.__frame_name, ref_frame_name)

class GetSet:
    def __init__(self, sqlite_connection: sql.Connection) -> None:
        self.__sql_connection = sqlite_connection
    def Get(self, frame_name: str) -> Getter:
        return Getter(self.__sql_connection, frame_name)
    def Set(self, frame_name: str) -> Setter:
        if frame_name == "world":
            raise ValueError("Cannot change the 'world' reference frame as it's assumed to be an inertial/immobile frame.")
        return Setter(self.__sql_connection, frame_name)

class DbConnector:
    '''Connect to Sqlite database and create new world if needed. 
    To limit concurrent actions, each world is in its one database/file.
    When a new world is mentioned, a new database is created.'''
    def __init__(self):
        self.__opened_connection = None
        self.__opened_world_name = None
        return
    def __del__(self):
        if self.__opened_connection is not None:
            self.__opened_connection.close()
    def In(self, worldName: str) -> GetSet:
        #If the connection is already open, simply return it
        if self.__opened_connection is not None and self.__opened_world_name == worldName:
            return GetSet(self.__opened_connection)
        #Otherwise, connect to the database
        else:
            if not re.match("^[0-9a-z\-]+$", worldName):
                raise ValueError("Only [a-z], [0-9] and dash (-) is allowed in the world name.")
            #Get a list of existing databases
            current_dir = Path('.')
            db_list = [x for x in current_dir.iterdir() if x.is_file() and x.suffix == '.db']
            db_name_list = [x.stem for x in db_list]
            #Connect to the database
            con = sql.connect(worldName+".db")
            #If the world name is not in the list, create a new database
            if worldName not in db_name_list:
                cur = con.cursor()
                with con:
                    #Each row describes a single frame with
                    # - name : Unique string
                    # - parent: Name of parent frame (reference this frame is defined from)
                    # - R00,R01,R02: First row of the rotation matrix in the transformation
                    # - R10,R11,R12: Second row of the rotation matrix in the transformation
                    # - R20,R21,R22: Third row of the rotation matrix in the transformation
                    # - t0,t1,t2: Translation vector in the transformation
                    #The 'world' frame is always the inertial/immobile reference frame, it's parent is set to NULL/None.
                    #All other frames must have a non-NULL parent, creating a tree with a single root.
                    cur.executescript("""
                        CREATE TABLE IF NOT EXISTS frames(
                            name TEXT PRIMARY KEY,
                            parent TEXT,
                            R00 REAL,
                            R01 REAL,
                            R02 REAL,
                            R10 REAL,
                            R11 REAL,
                            R12 REAL,
                            R20 REAL,
                            R21 REAL,
                            R22 REAL,
                            t0 REAL,
                            t1 REAL,
                            t2 REAL
                        );
                        """)
                    cur.execute("INSERT INTO frames VALUES ('world', NULL, 1,0,0, 0,1,0, 0,0,1, 0,0,0)")
            
            self.__opened_connection = con
            self.__opened_world_name = worldName
            return GetSet(con)

db = DbConnector()
mat = np.eye(4)
mat[0:3, 3] = [1,1,1]
db.In('my-world').Set('table').Wrt('world').Ei('world').As(mat)
pose = SE3.Rx(30, "deg", t=[0.5,0.5,0])
db.In('my-world').Set('object').Wrt('table').Ei('table').As(pose)

db.In('my-world').Get('object').Wrt('world').Ei('world')

Visualize('my-world')