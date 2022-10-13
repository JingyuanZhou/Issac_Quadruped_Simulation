from MPC_Controller.FSM_states.ControlFSMData import ControlFSMData
from MPC_Controller.FSM_states.FSM_State import FSM_State, FSM_StateName
from MPC_Controller.Parameters import Parameters
import numpy as np
from MPC_Controller.utils import DTYPE
from MPC_Controller.common.Quadruped import RobotType

class FSM_State_StandUp(FSM_State):
    def __init__(self, _controlFSMData: ControlFSMData):
        super().__init__(_controlFSMData, FSM_StateName.STAND_UP, "STAND_UP")

        self.checkSafeOrientation = False
        self.checkPDesFoot = False
        self.checkForceFeedForward = False
        self._ini_foot_pos = np.zeros((3,4), dtype=DTYPE)
        self.controller_dt = 0.002
        self.iter = 0

    def onEnter(self):
        self.nextStateName = self.stateName
        self.transitionDone = None
        self.iter = 0
        for leg in range(4):
            self._ini_foot_pos[:,leg]=self._data._legController.datas[leg].p
        
    def run(self):
        if self._data._quadruped._robotType == RobotType.MINI_CHEETAH:
            hMax = 0.25
            progress = 2*self.iter*self.controller_dt
            if progress > 1:
                progress = 1
            for i in range(4):
                self._data._legController.commands[i].kpCartesian = np.diag([500,500,500])
                self._data._legController.commands[i].kdCartesian = np.diag([8,8,8])
                self._data._legController.commands[i].pDes = self._ini_foot_pos[:,i]
                self._data._legController.commands[i].pDes[2] = progress*(-hMax)+(1.-progress)*self._ini_foot_pos[2,i]

        return 
    
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
        if Parameters.control_mode is FSM_StateName.STAND_UP:
            pass
        elif Parameters.control_mode is FSM_StateName.BACKFLIP:
            self.nextStateName = FSM_StateName.BACKFLIP
        elif Parameters.control_mode is FSM_StateName.RECOVERY_STAND:
            self.nextStateName = FSM_StateName.RECOVERY_STAND
        elif Parameters.control_mode is FSM_StateName.LOCOMOTION:
            self.nextStateName = FSM_StateName.LOCOMOTION
        elif Parameters.control_mode is FSM_StateName.PASSIVE:
            self.nextStateName = FSM_StateName.PASSIVE
        elif Parameters.control_mode is FSM_StateName.FRONTJUMP:
            self.nextStateName = FSM_StateName.FRONTJUMP
            
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
        elif self.nextStateName == FSM_StateName.FRONTJUMP:
            self.transitionDone = True
        elif self.nextStateName == FSM_StateName.BACKFLIP:
            self.transitionDone = True
        else:
            print("[CONTROL FSM] Something went wrong in transition")

        # Return the transition data to the FSM
        return self.transitionDone

    def onEnter(self):
        pass