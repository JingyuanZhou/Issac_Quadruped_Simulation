from math import floor
import numpy as np
from MPC_Controller.FSM_states.ControlFSMData import ControlFSMData
from MPC_Controller.FSM_states.FSM_State import FSM_State, FSM_StateName
from MPC_Controller.Parameters import Parameters
from MPC_Controller.utils import DTYPE
from MPC_Controller.FSM_states.DataReader import DataReader
from MPC_Controller.FSM_states.FrontJumpCtrl import FrontJumpCtrl


class FSM_State_FrontJump(FSM_State):
    def __init__(self, _controlFSMData: ControlFSMData):
        super().__init__(_controlFSMData, FSM_StateName.STAND_UP, "STAND_UP")

        self.Preparation = 0
        self.Flip = 1
        self.Landing = 2

        self.test_initialized=False
        self.iter = 0
        self._state_iter = 0
        self._motion_start_iter = 0
        self._flag = self.Preparation

        self._b_running = True
        self._b_first_visit = True
        self._count = 0
        self._waiting_count = 6
        self._curr_time = 0
        self.initial_jpos = [0,0,0,0]

        # Set the pre controls safety checks
        self.checkSafeOrientation = False
        # Post control safety checks
        self.checkPDesFoot = False
        self.checkForceFeedForward = False

        self.zero_vec3 = np.zeros((3,1), dtype=DTYPE)
        
        self.controller_dt = 0.002
        self._data_reader=DataReader(_controlFSMData._quadruped._robotType,FSM_StateName.FRONTJUMP)
        self.front_jump_ctrl_=FrontJumpCtrl(self._data_reader,self.controller_dt)
        self.front_jump_ctrl_.SetParameter()
        

    def onEnter(self):
        # Default is to not transition
        self.nextStateName = self.stateName

        # Reset the transition data
        self.transitionDone = False
        self.transitionData = 0

        # Reset iteration counter
        self.iter = 0
        self._state_iter = 0
        self._curr_time = 0
        self._count = 0
        self._motion_start_iter = 0
        self._b_first_visit = True

        # initial configuration, position
        for leg in range(4):
            self.initial_jpos[leg] = self._data._legController.datas[leg].q

        self.front_jump_ctrl_.SetParameter()

    def run(self):
        """
         * Calls the functions to be executed on each control loop iteration.
        """
        if self._b_running:
            if not self._Initialization():
                self.ComputeCommand()
        else:
            self._SafeCommand()

        self._count=self._count+1
        self._curr_time = self._curr_time+self.controller_dt#self._count+self.controller_dt

    def _Initialization(self):
        if self.test_initialized is not True:
            self.test_initialized = True
            print("[Cheetah Test] Test initialization is done")
        if self._count<self._waiting_count:
            for leg in range(4):
                self._data._legController.commands[leg].qDes = self.initial_jpos[leg]
                for jidx in range(3):
                    self._data._legController.commands[leg].tauFeedForward[jidx] = 0
                    self._data._legController.commands[leg].qdDes[jidx] = 0
                    self._data._legController.commands[leg].kpJoint[jidx,jidx]=20
                    self._data._legController.commands[leg].kdJoint[jidx,jidx]=2
            return True

        return False

    def ComputeCommand(self):
        if self._b_first_visit:
            self.front_jump_ctrl_.FirstVisit(self._curr_time)
            self._b_first_visit=False

        self.front_jump_ctrl_.OneStep(self._curr_time,False,self._data._legController.commands)

        if self.front_jump_ctrl_.EndOfPhase(self._data._legController.datas):
            self.front_jump_ctrl_.LastVisit()
            Parameters.control_mode = FSM_StateName.RECOVERY_STAND

    def _SafeCommand(self):
        for leg in range(4):
            for jidx in range(3):
                self._data._legController.commands[leg].tauFeedForward[jidx] = 0
                self._data._legController.commands[leg].qDes[jidx] = self._data._legController.datas[leg].q[jidx]
                self._data._legController.commands[leg].qdDes[jidx] = 0

    def onExit(self):
        """
         * Cleans up the state information on exiting the state.
        """
        # Nothing to clean up when exiting
        pass

    def checkTransition(self):
        """
        * Manages which states can be transitioned into either by the user
        * commands or state event triggers.
        *
        * @return the enumerated FSM state name to transition into
        """
        self.nextStateName = self.stateName
        self.iter += 1

        # Switch FSM control mode
        if Parameters.control_mode is FSM_StateName.FRONTJUMP:
            pass
        elif Parameters.control_mode is FSM_StateName.RECOVERY_STAND:
            self.nextStateName = FSM_StateName.RECOVERY_STAND

        elif Parameters.control_mode is FSM_StateName.LOCOMOTION:
            self.nextStateName = FSM_StateName.LOCOMOTION

        elif Parameters.control_mode is FSM_StateName.PASSIVE:
            self.nextStateName = FSM_StateName.PASSIVE
        elif Parameters.control_mode is FSM_StateName.BACKFLIP:
            self.nextStateName = FSM_StateName.BACKFLIP
            
        else:
            print("[CONTROL FSM] Bad Request: Cannot transition from "
                + self.stateName.name
                + " to "
                + Parameters.control_mode.name)
        return self.nextStateName

    def transition(self):
        """
        * Handles the actual transition for the robot between states.
        * Returns true when the transition is completed.
        *
        * @return true if transition is complete
        """
        # Finish Transition

        if self.nextStateName==FSM_StateName.PASSIVE:
            self.transitionDone = True

        elif self.nextStateName == FSM_StateName.RECOVERY_STAND:
            self.transitionDone = True

        elif self.nextStateName == FSM_StateName.LOCOMOTION:
            self.transitionDone = True
        elif self.nextStateName == FSM_StateName.BACKFLIP:
            self.transitionDone = True
        else:
            print("[CONTROL FSM] Something went wrong in transition")

        # Return the transition data to the FSM
        return self.transitionDone

    def _SetJPosInterPts(self, 
                         curr_iter:int, max_iter:int, leg:int, 
                         ini:np.ndarray, fin:np.ndarray):
        a = 0.0
        b = 1.0

        # if we're done interpolating
        if curr_iter <= max_iter:
            b = float(curr_iter) / max_iter
            a = 1.0 - b
        
        # compute setpoints
        inter_pos = a * ini + b * fin
        # do control
        self.jointPDControl(leg, inter_pos, self.zero_vec3)