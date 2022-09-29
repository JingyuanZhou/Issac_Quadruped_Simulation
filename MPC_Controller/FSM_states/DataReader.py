from io import SEEK_END, SEEK_SET
import MPC_Controller.FSM_states.FSM_State

class DataReader:

    def __init__(self,_type,stateNameIn):
        self.plan_cols = 22
        self.plan_loaded = False
        self.plan_buffer = None
        print("[Backflip DataReader] Setup for cheetah 3")
        self.load_control_plan("/home/jyzhou53/rl-mpc-locomotion/MPC_Controller/FSM_states/backflip.dat")
        print("[Backflip DataReader] Constructed.")
    
    def load_control_plan(self,filename):
        f=open(filename,"rb")
        f.seek(0,SEEK_END)
        filesize = f.tell()
        f.seek(0,SEEK_SET)
        print("allocating "+str(filesize)+" bytes  for control plan")
        self.plan_buffer = f.read()
        f.close()
        
        self.plan_loaded = True
        self.plan_timesteps = filesize / (4*self.plan_cols)
        print("[Backflip DataReader] Done loading plan for "+str(self.plan_timesteps)+" timesteps")

    def get_initial_configuration(self):
        pass

    def get_plan_at_time(self,timestep):
        if self.plan_loaded:

            if timestep<0 or timestep>=self.plan_timesteps:
                print("[Backflip DataReader] Error: get_plan_at_time called for timestep error")
                timestep = int(self.plan_timesteps - 1)

            return self.plan_buffer[4*self.plan_cols*timestep:4*self.plan_cols*(timestep+1)]

        print("[Backflip DataReader] Error: get_initial_configuration called without a plan!")
        return None

    def unload_control_plan(self):
        self.plan_timesteps = -1
        self.plan_loaded = False
        print("[Backflip DataReader] Unloaded plan.")