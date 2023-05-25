from dynamics.vehicle_dynamics_stown import vehicle_dynamics_st
from helpers.load_model import get_dotdict
from scipy.integrate import odeint
import numpy as np

model_name = "NUC4_pacejka"
model, tiretype = model_name.split("_")

model = get_dotdict(model_name)

class Simulator:
  def func_ST(self, x, t, u, p):
      f = vehicle_dynamics_st(x, u, p, tiretype)
      return f

  def generate_lookup(self):
    start_steer = 0.0
    steer_fine_end = 0.1
    end_steer = 0.4
    n_steps_steer = 30 # used for fine region and then again for coarse region
    start_vel = 0.5
    end_vel = 7.0
    n_steps_vel = 65
    
    fine_steers = np.linspace(start_steer, steer_fine_end, n_steps_steer)
    steers = np.linspace(steer_fine_end, end_steer, n_steps_steer)
    steers = np.concatenate((fine_steers, steers))
    vels = np.linspace(start_vel, end_vel, n_steps_vel)

    n_steps_steer = len(steers)
    self.lookup_table = np.empty([n_steps_steer + 1, n_steps_vel +1])    

    self.lookup_table[0, 1:] = vels
    self.lookup_table[1:, 0] = steers

    for steer_idx, steer in enumerate(steers):
      for vel_idx, vel in enumerate(vels):
        initialState = [0, 0, 0, vel, 0, 0]
        u = [steer, 0]
        
        # print("Steering Angle :" + str(steer) + " Velocity :" + str(vel))
        dt = 0.01
        duration = 2.0
        t = np.arange(0, duration, dt)

        self.sol = odeint(self.func_ST, initialState, t, args=(u, model))  

        if abs(self.sol[-1, 5] - self.sol[-10, 5]) > 0.05:
          #print("failed to converge, limit")
          self.lookup_table[steer_idx+1, vel_idx+1:] = None
          break

        else:
          a_lat = self.sol[-1, 5] * vel
          a_lat_all = self.sol[:, 5] * vel
          self.lookup_table[steer_idx+1, vel_idx+1] = a_lat

  def save_lookup(self):
    model, tires = model_name.split("_")
    file_path = "./models/" + model + "/" + model_name + "_lookup_table.csv"
    np.savetxt(file_path, self.lookup_table, delimiter=",")

if __name__ == "__main__":
  sim = Simulator()
  sim.generate_lookup()
  sim.save_lookup()
  print("Done")