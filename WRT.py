import numpy as np
import sqlite3 as sql
import re
import sympy #spatialmath depends on sympy
from spatialmath import SE3
from anytree import NodeMixin, RenderTree
from pathlib import Path


class PoseNode(SE3, NodeMixin):
    '''Describe a node in the pose graph.'''
    def __init__(self, frame_name: str, relative_pose: SE3, parent=None):
        super(SE3, self).__init__()
        self.name           = frame_name
        self.relative_pose  = relative_pose
        self.parent         = parent

class SetAs:
    def __init__(self, sqlite_connection: sql.Connection, frame_name:str, ref_frame_name:str, in_frame_name:str) -> None:
        self.verifyInput(frame_name)
        self.verifyInput(ref_frame_name)
        self.verifyInput(in_frame_name)
        self.__frame_name     = frame_name
        self.__ref_frame_name = ref_frame_name
        self.__in_frame_name  = in_frame_name
        self.__sql_connection = sqlite_connection
    def verifyInput(self, input_string:str):
        if not re.match("^[0-9a-z\-]+$", input_string):
            raise ValueError("Only [a-z], [0-9] and dash (-) is allowed in the frame name: {}".format(input_string))
    def As(self, transfo_matrix):
        assert(type(transfo_matrix) == np.ndarray or type(transfo_matrix) == SE3)
        if type(transfo_matrix) is np.array:
            assert(transfo_matrix.shape == (4,4))
        
        with self.__sql_connection as con:
            cur = con.cursor()
            #1) Make sure that the __ref_frame_name exists in the DB
            #2) Remove from DB any frame with  __frame_name
            cur.execute("DELETE FROM frames WHERE name IS ?",(self.__frame_name))
            #2) Add new frame in the DB
            #TODO: Complete this.
            cur.execute("INSERT INTO frames VALUES (?, ?, 1,0,0, 0,1,0, 0,0,1, 0,0,0)", (self.__frame_name, self.__ref_frame_name))

class GetExpressedIn:
    def __init__(self, sqlite_connection: sql.Connection, frame_name:str, ref_frame_name:str) -> None:
        self.verifyInput(frame_name)
        self.verifyInput(ref_frame_name)
        self.__frame_name     = frame_name
        self.__ref_frame_name = ref_frame_name
        self.__sql_connection = sqlite_connection
    def verifyInput(self, input_string:str):
        if not re.match("^[0-9a-z\-]+$", input_string):
            raise ValueError("Only [a-z], [0-9] and dash (-) is allowed in the frame name: {}".format(input_string))
    def Ei(self, in_frame_name: str) -> SE3:
        self.verifyInput(in_frame_name)
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
            con = sql.connect(worldName)
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
db.In('my-world').Get('table').Wrt('world').Ei('world')