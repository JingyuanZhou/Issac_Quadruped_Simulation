from MPC_Controller.FSM_states.DataReader import DataReader
from MPC_Controller.common.LegController import LegControllerData
import math

class DataReadCtrl:
    def __init__(self,data_reader,_dt):
        self._data_reader = data_reader
        self.dt = _dt
        self._key_pt_step = math.ceil(self.dt*1000)
        self._Kp=[0 for _ in range(12)]
        self._Kd=[0 for _ in range(12)]
        self._des_jpos=[0 for _ in range(12)]
        self._des_jvel=[0 for _ in range(12)]
        self._jtorque=[0 for _ in range(12)]
        self._Kp_joint=[0 for _ in range(3)]
        self._Kd_joint=[0 for _ in range(3)]
        self._ctrl_start_time = 0
        self._q_knee_max = 2.0
        self._qdot_knee_max = 2.0
        self._state_machine_time = None
        self.current_iteration = None
        self.pre_mode_count = None
        self._end_time = 5.5
        self._b_Preparation = False
        self._b_set_height_target = None
        self._dim_contact = None

    def FirstVisit(self,_curr_time):
        self._ctrl_start_time = _curr_time
        self.current_iteration = 0
        self.pre_mode_count = 0

    def LastVisit(self):
        pass

    def EndOfPhase(self,data):
        if self._state_machine_time > self._end_time - 2*self.dt:
            return True
        for leg in range(4):
            if self._state_machine_time>2.7 and data[leg].q[1] > self._q_knee_max and data[leg].qd[1] > self._qdot_knee_max:
                print("Contact detected at leg ["+ str(leg) +"] => Switch to the landing phase !!!")
                print("state_machine_time:"+str(self._state_machine_time))
                print("Q-Knee:"+str(data[leg].q[1]))
                print("Qdot-Knee:"+str(data[leg].qd[1]))
                return True
        return False

    def SetParameter(self):
        for i in range(12):
            self._Kp[i] = 1000
            self._Kd[i] = 5.

        self._Kp_joint=[10.0, 10.0, 10.0]
        self._Kd_joint=[1.0, 1.0, 1.0]