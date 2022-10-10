from MPC_Controller.FSM_states.DataReader import DataReader
from MPC_Controller.FSM_states.DataReaderCtrl import DataReadCtrl
from MPC_Controller.common.LegController import LegControllerCommand
import struct

class FrontJumpCtrl(DataReadCtrl):
    def __init__(self,data_reader,_dt):
        super(FrontJumpCtrl,self).__init__(data_reader,_dt)
        self.q0_offset = 0
        self.qd0_offset = 7
        self.tau_offset = 14#14
        self.force_offset = 18

    def OneStep(self,_curr_time,b_preparation,command):
        self._state_machine_time = _curr_time - self._ctrl_start_time
        self._b_Preparation = b_preparation
        self._update_joint_command()

        for leg in range(4):
            for jidx in range(3):
                command[leg].tauFeedForward[jidx] = self._jtorque[3*leg+jidx]
                command[leg].qDes[jidx] = self._des_jpos[3*leg+jidx]+0*_curr_time
                command[leg].qdDes[jidx] = self._des_jpos[3*leg+jidx]
                command[leg].kpJoint[jidx, jidx] = self._Kp_joint[jidx]
                command[leg].kdJoint[jidx,jidx] = self._Kd_joint[jidx]

    def Bytes2Float32Slice(self,feature):
        x = []
        FEATURE_SIZE = 22
        for i in range(FEATURE_SIZE):
            data = feature[i * 4: (i * 4) + 4]
            a = struct.unpack('f', data)
            x.append(float(a[0]))
        return x

    def _update_joint_command(self):
        pre_mode_duration = 700
        leg_clearance_iteration_front = 240
        leg_clearance_iteration = 600
        leg_ramp_iteration = 610
        tuck_iteration = 610
        ramp_end_iteration = 700

        self._Kp_joint = [10.0, 10.0, 10.0]
        self._Kd_joint = [1.0, 1.0, 1.0]

        if self.pre_mode_count < pre_mode_duration or self._b_Preparation:
            if self.pre_mode_count == 0:
                print("plan_timesteps:"+str(self._data_reader.plan_timesteps))
            
            self.pre_mode_count = self.pre_mode_count + self._key_pt_step
            self.current_iteration = 0
            tau_mult = 0
        else:
            tau_mult = 1.2

        if self.current_iteration>self._data_reader.plan_timesteps - 1:
            self.current_step = int(self._data_reader.plan_timesteps - 1)
        
        current_step = self._data_reader.get_plan_at_time(self.current_iteration)
        current_step = self.Bytes2Float32Slice(current_step)

        tau_offset_list=[self.tau_offset for i in range(22)]
        tau = [current_step[i]+tau_offset_list[i] for i in range(min(len(current_step),len(tau_offset_list)))]

        q_des_front = [0.0,current_step[3],current_step[4]]
        q_des_rear = [0.0,current_step[5],current_step[6]]
        qd_des_front = [0.0,current_step[10],current_step[11]]
        qd_des_rear = [0.0,current_step[12],current_step[13]]

        tau_front = [0.0,tau_mult*tau[0]/2.0,tau_mult*tau[1]/2.0]
        tau_rear = [0.0,tau_mult*tau[2]/2.0,tau_mult*tau[3]/2.0]

        s=0.0

        if self.current_iteration >= leg_clearance_iteration_front and self.current_iteration <= leg_clearance_iteration:
            q_des_front = [0.0,current_step[3],current_step[4]]

            current_step = self._data_reader.get_plan_at_time(leg_clearance_iteration_front)
            q_des_front = [0.0,-2.3,2.5]
            qd_des_front = [0.0,0.0,0.0]
            tau_front = [0.0,0.0,0.0]

        if self.current_iteration >= leg_clearance_iteration and self.current_iteration<tuck_iteration:
            qd_des_front = [0.0,0.0,0.0]
            qd_des_rear = [0.0,0.0,0.0]
            tau_front = [0.0,0.0,0.0]
            tau_rear = [0.0,0.0,0.0]

            s = (self.current_iteration - leg_clearance_iteration)/(leg_ramp_iteration-leg_clearance_iteration)

            if s>1:
                s=1

            current_step = self._data_reader.get_plan_at_time(tuck_iteration)
            current_step = self.Bytes2Float32Slice(current_step)
            q_des_front_0 = [0.0,-2.3,2.5]
            q_des_rear_0 = [0.0,-1.25,2.5]

            # SET THE DESIRED JOINT STATES FOR LEG_CLEARANCE_ITERATION
            current_step = self._data_reader.get_plan_at_time(0)
            current_step = self.Bytes2Float32Slice(current_step)
            q_des_front_f = [0.0,-2.3,2.5]
            q_des_rear_f = [0.0,-1.25,2.5]

            transition_q_des_front_0=[i * (1-s) for i in q_des_front_0]
            transition_q_des_front_f=[i * (1-s) for i in q_des_front_f]
            transition_q_des_rear_0=[i * (1-s) for i in q_des_rear_0]
            transition_q_des_rear_f=[i * (1-s) for i in q_des_rear_f]

            q_des_front = [transition_q_des_front_0[i]+transition_q_des_front_f[i] for i in range(min(len(transition_q_des_front_0),len(transition_q_des_front_f)))]
            q_des_rear = [transition_q_des_rear_0[i]+transition_q_des_rear_f[i] for i in range(min(len(transition_q_des_rear_0),len(transition_q_des_rear_f)))]

        elif self.current_iteration >= tuck_iteration: # ramp to landing configuration
            qd_des_front = [0.0,0.0,0.0]
            qd_des_rear = [0.0,0.0,0.0]
            tau_front = [0.0,0.0,0.0]
            tau_rear = [0.0,0.0,0.0]

            s = (self.current_iteration-tuck_iteration)/(ramp_end_iteration-tuck_iteration)

            if s>1:
                s=1

            current_step = self._data_reader.get_plan_at_time(tuck_iteration)
            q_des_front_0 = [0.0,-2.3,2.5]
            q_des_rear_0 = [0.0,-1.25,2.5]

            current_step = self._data_reader.get_plan_at_time(0)
            q_des_front_f = [0.0,-0.45,1.3]
            q_des_rear_f = [0.0,-0.45,1.3]

            transition_q_des_front_0=[i * (1-s) for i in q_des_front_0]
            transition_q_des_front_f=[i * (1-s) for i in q_des_front_f]
            transition_q_des_rear_0=[i * (1-s) for i in q_des_rear_0]
            transition_q_des_rear_f=[i * (1-s) for i in q_des_rear_f]

            q_des_front = [transition_q_des_front_0[i]+transition_q_des_front_f[i] for i in range(min(len(transition_q_des_front_0),len(transition_q_des_front_f)))]
            q_des_rear = [transition_q_des_rear_0[i]+transition_q_des_rear_f[i] for i in range(min(len(transition_q_des_rear_0),len(transition_q_des_rear_f)))]

            self._Kp_joint = [20.0, 20.0, 20.0]
            self._Kd_joint = [2.0, 2.0, 2.0]
        
        # Abduction
        for i in range(0,12,3):
            self._des_jpos[i] = 0.0
            self._des_jvel[i] = 0.0
            self._jtorque[i] = 0.0

        self._des_jpos[0] = s*(-0.2)
        self._des_jpos[3] = s*0.2
        self._des_jpos[6] = s*(-0.2)
        self._des_jpos[9] = s*0.2

        if self.current_iteration >= tuck_iteration:
            self._des_jpos[0] = -0.2
            self._des_jpos[3] = 0.2
            self._des_jpos[6] = -0.2
            self._des_jpos[9] = 0.2

        # Front Hip
        for i in range(1,6,3):
            self._des_jpos[i] = q_des_front[1]
            self._des_jvel[i] = qd_des_front[1]
            self._jtorque[i] = tau_front[1]

        #Front Knee
        for i in range(2,6,3):
            self._des_jpos[i] = q_des_rear[2]
            self._des_jvel[i] = qd_des_front[2]
            self._jtorque[i] = tau_front[2]

        #Hind Hip
        for i in range(7,12,3):
            self._des_jpos[i] = q_des_rear[1]
            self._des_jvel[i] = qd_des_rear[1]
            self._jtorque[i] = tau_rear[1]

        #Hind Knee
        for i in range(8,12,3):
            self._des_jpos[i] = q_des_rear[2]
            self._des_jvel[i] = qd_des_rear[2]
            self._jtorque[i] = tau_rear[2]

        self.current_iteration = self.current_iteration+self._key_pt_step
