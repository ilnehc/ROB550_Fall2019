import numpy as np 
import time

"""
TODO: build a trajectory generator and waypoint planner 
        so it allows your state machine to iterate through
        the plan at the desired command update rate
"""

class TrajectoryPlanner():
    def __init__(self, rexarm):
        self.idle = True
        self.rexarm = rexarm
        self.num_joints = rexarm.num_joints
        self.initial_wp = [0.0]*self.num_joints
        self.final_wp = [0.0]*self.num_joints
        self.dt = 0.05 # command rate
    
    def set_initial_wp(self):
        self.initial_wp = self.rexarm.get_positions()
        #print(self.initial_wp)

    def set_final_wp(self, waypoint):
        self.final_wp = waypoint

    def go(self, max_speed = 2.5):
        self.rexarm.set_speeds([max_speed,max_speed,max_speed,max_speed,max_speed,max_speed])
        #self.rexarm.set_positions = self.final_wp
        return max_speed

    def stop(self):
        pass

    def calc_time_from_waypoints(self, initial_wp, final_wp, max_speed):
        #print(np.amax(np.absolute(final_wp - initial_wp)))
        t = 2 * np.amax(np.absolute(final_wp - initial_wp)) / max_speed  #?????
        return t

    def generate_cubic_spline(self, initial_wp, final_wp, T):
        a_coe = np.zeros((4,self.num_joints))
        dtarray = np.arange(0,T+self.dt,self.dt)
        innerpoints = np.zeros((len(dtarray),self.num_joints))
        innerp_vel = np.zeros((len(dtarray),self.num_joints))
        tmatrix = np.array([
                                [1, 0, 0, 0], 
                                [0, 1, 0, 0], 
                                [1, T, T**2, T**3], 
                                [0, 1, 2*T, 3*T**2]
                            ])
        for i in np.arange(self.num_joints):
            tmp = np.matmul(np.linalg.inv(tmatrix), [[initial_wp[i]], [0], [final_wp[i]], [0]])
            a_coe[0][i] = tmp[0]
            a_coe[1][i] = tmp[1]
            a_coe[2][i] = tmp[2]
            a_coe[3][i] = tmp[3]
            for j in np.arange(len(dtarray)):
                innerpoints[j][i] = np.matmul(np.transpose(a_coe[:,i]), [[1], [dtarray[j]], [dtarray[j]**2], [dtarray[j]**3]])
                innerp_vel[j][i] = np.matmul(np.transpose(a_coe[:,i]), [[0], [1], [2*dtarray[j]], [3*dtarray[j]**2]])
        return innerpoints, innerp_vel

    #for list of points execution
    def execute_plan(self, plan, max_v, look_ahead=8):
        self.rexarm.set_positions(np.zeros(6))
        self.rexarm.pause(3.0)
        for waypoint in plan:
            print("next waypoint")
            self.set_initial_wp()
            self.set_final_wp(waypoint)
            print(self.initial_wp)
            print(self.final_wp)
            T = self.calc_time_from_waypoints(self.initial_wp,self.final_wp,self.go(max_v))
            #print(self.go())
            #print(T)
            innerpoints, innerp_vel = self.generate_cubic_spline(self.initial_wp,self.final_wp,T)
            #print(innerpoints)
            if (len(innerpoints)<=look_ahead):
                self.rexarm.set_speeds(innerp_vel[-2])
                self.rexarm.set_positions(waypoint)
                self.rexarm.pause(5*T)
                print("less than 8 innerpoints!")
                #continue
            else:
                i = look_ahead
                for innerpoint in innerpoints[look_ahead:]:
                    self.rexarm.set_positions(innerpoint)
                    print(innerpoint)
                    self.rexarm.set_speeds(innerp_vel[i])
                    self.rexarm.pause(self.dt)
                    print("done!")
                    i += 1
            print(waypoint)
            self.rexarm.pause(2*T)
            #if (np.amax(np.absolute(self.rexarm.get_positions()-waypoint))<1e-5):
            #    continue
            #else:
            #    self.rexarm.pause(5*T)
    
    #for one point execution
    def execute_point(self, point, max_v, look_ahead=8):
        print("next waypoint")
        self.set_initial_wp()
        self.set_final_wp(point)
        print(self.initial_wp)
        print(self.final_wp)
        T = self.calc_time_from_waypoints(self.initial_wp,self.final_wp,self.go(max_v))
        #print(self.go())
        #print(T)
        innerpoints, innerp_vel = self.generate_cubic_spline(self.initial_wp,self.final_wp,T)
        #print(innerpoints)
        if (len(innerpoints)<=look_ahead):
            self.rexarm.set_speeds(innerp_vel[-2])
            self.rexarm.set_positions(point)
            self.rexarm.pause(5*T)
            print("less than 8 innerpoints!")
            #continue
        else:
            i = look_ahead
            for innerpoint in innerpoints[look_ahead:]:
                self.rexarm.set_positions(innerpoint)
                print(innerpoint)
                self.rexarm.set_speeds(innerp_vel[i])
                self.rexarm.pause(self.dt)
                print("done!")
                i += 1
        print(point)
        self.rexarm.pause(2*T)
