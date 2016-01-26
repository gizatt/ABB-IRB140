"""
Author: alexc89@mit.edu
IRB140DRCLCM_Convertor
This is a script to convert LCM channels from IRB140LCM_Monitor (IRB140STATE) to DRC Channels: robot_state_t.

"""
import lcm
import time
import drc
import abblcm
import math
import drake
from collections import deque

class abbIRB140DRCLCMConvertor:
    last_states = deque([]);
    last_states_t = deque([]);
    last_velocities = deque([]);
    last_velocities_t = deque([]); 
    velocity_avg_window = 3;
    accel_avg_window = 3;

    def __init__(self):
        self.lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
        self.lc.subscribe("IRB140STATE", self.convertNSend)
        self.lc.subscribe("COMMITTED_ROBOT_PLAN",self.joint_cmd_handler)
        self.lc.subscribe("IRB140FTSENSOR",self.ft_handler)
        

    def joint_plan_handler(self,channel,data):
        msgIn = drc.robot_plan_t.decode(data)
        msgOut = abblcm.abb_irb140joint_plan()

        msgOut.utime = msgIn.utime
        msgOut.n_cmd_times = msgIn.num_states
        msgOut.joint_cmd = [abblcm.abb_irb140joints() for i in range(msgOut.n_cmd_times)]
        for i in range(msgOut.n_cmd_times):
            msgOut.joint_cmd[i].pos = [msgIn.plan[i].joint_position[j]/math.pi*180.0 for j in range(msgIn.plan[i].num_joints)]
            msgOut.joint_cmd[i].vel = [msgIn.plan[i].joint_velocity[j]/math.pi*180.0 for j in range(msgIn.plan[i].num_joints)]
            msgOut.joint_cmd[i].utime = msgIn.plan[i].utime
            #msgOut.joint_cmd[i].pos[0] = -msgOut.joint_cmd[i].pos[0]
            #msgOut.joint_cmd[i].pos[2] = -msgOut.joint_cmd[i].pos[2]
            #msgOut.joint_cmd[i].vel[0] = -msgOut.joint_cmd[i].vel[0]
            #msgOut.joint_cmd[i].vel[2] = -msgOut.joint_cmd[i].vel[2]
        self.lc.publish("IRB140JOINTPLAN",msgOut.encode())    
    
    def joint_cmd_handler(self,channel,data):
        msgIn = drc.robot_plan_t.decode(data)
        msgOut = abblcm.abb_irb140joints()

        msgOut.utime = msgIn.utime
        msgOut.pos = [msgIn.plan[-1].joint_position[j]/math.pi*180.0 for j in range(msgIn.plan[-1].num_joints)]
        msgOut.vel = [msgIn.plan[-1].joint_velocity[j]/math.pi*180.0 for j in range(msgIn.plan[-1].num_joints)]
        #msgOut.pos[0] = -msgOut.pos[0]
        #msgOut.pos[2] = -msgOut.pos[2]
        #msgOut.vel[0] = -msgOut.vel[0]
        #msgOut.vel[2] = -msgOut.vel[2]
        self.lc.publish("IRB140JOINTCMD",msgOut.encode())


    def ft_handler(self,channel,data):
        msgIn = abblcm.abb_irb140ftsensor.decode(data)
        msgOut = drake.lcmt_force_torque()

        msgOut.timestamp = msgIn.utime
        msgOut.fx = msgIn.hand_force[0]
        msgOut.fy = msgIn.hand_force[1]
        msgOut.fz = msgIn.hand_force[2]
        msgOut.tx = msgIn.hand_torque[0]
        msgOut.ty = msgIn.hand_torque[1]
        msgOut.tz = msgIn.hand_torque[2]
        self.lc.publish("FORCE_TORQUE", msgOut.encode())



    def convertNSend(self,channel,data):
        msgIn = abblcm.abb_irb140state.decode(data)
        msgOut = drc.robot_state_t()

        # Extract position
        curr_pos = [joint_pos/180.0*math.pi for joint_pos in msgIn.joints.pos];
        # Keep track of them over time
        self.last_states.append(curr_pos);
        self.last_states_t.append(time.time());
        if (len(self.last_states) > self.velocity_avg_window):
            self.last_states.popleft()
            self.last_states_t.popleft()

        # Compute velocities
        curr_vel = [0]*len(curr_pos);
        for i in range(0, len(curr_pos)):
            for j in range(1, len(self.last_states)):
                curr_vel[i] += (self.last_states[j][i] - self.last_states[j-1][i])/(self.last_states_t[j]-self.last_states_t[j-1]);
            if (len(self.last_states) > 1):
                curr_vel[i] /= (len(self.last_states)-1);

        
        self.last_velocities.append(curr_vel);
        self.last_velocities_t.append(time.time());
        if (len(self.last_velocities) > self.accel_avg_window):
            self.last_velocities.popleft()
            self.last_velocities_t.popleft()

        # Similarly, accelerations
        curr_acc = [0]*len(curr_pos);
        for i in range(0, len(curr_pos)):
            for j in range(1, len(self.last_velocities)):
                curr_acc[i] += (self.last_velocities[j][i] - self.last_velocities[j-1][i])/(self.last_velocities_t[j]-self.last_velocities_t[j-1]);
            if (len(self.last_velocities) > 1):
                curr_acc[i] /= (len(self.last_velocities)-1);

        print curr_pos, curr_vel, curr_acc

        ### Msg Conversion
        msgOut.utime = msgIn.utime
        msgOut.pose = drc.position_3d_t()
        msgOut.pose.translation = drc.vector_3d_t()
        msgOut.pose.translation.x = 0.0
        msgOut.pose.translation.y = 0.0
        msgOut.pose.translation.z = 0.0
        msgOut.pose.rotation = drc.quaternion_t()
        # rotate by x axis by -90 degrees
        msgOut.pose.rotation.w = 1.0 
        msgOut.pose.rotation.x = 0.0
        msgOut.pose.rotation.y = 0.0
        msgOut.pose.rotation.z = 0.0
        msgOut.twist = drc.twist_t()
        msgOut.twist.linear_velocity = drc.vector_3d_t()
        msgOut.twist.linear_velocity.x = 0.0
        msgOut.twist.linear_velocity.y = 0.0
        msgOut.twist.linear_velocity.z = 0.0
        msgOut.twist.angular_velocity = drc.vector_3d_t()
        msgOut.twist.angular_velocity.x = 0.0
        msgOut.twist.angular_velocity.y = 0.0
        msgOut.twist.angular_velocity.z = 0.0

        msgOut.num_joints = len(msgIn.joints.pos)
        msgOut.joint_name = ["joint" + str(i+1) for i in range(msgOut.num_joints)]
        msgOut.joint_position = [joint_pos/180.0*math.pi for joint_pos in msgIn.joints.pos]
        #msgOut.joint_position[0] = -msgOut.joint_position[0]
	#msgOut.joint_position[2] = -msgOut.joint_position[2]
        msgOut.joint_velocity = curr_vel #[joint_vel/180.0*math.pi for joint_vel in msgIn.joints.vel]
        msgOut.joint_effort = [0.0 for i in range(msgOut.num_joints)]
        
        msgOut.force_torque = drc.force_torque_t()
        msgOut.force_torque.l_foot_force_z = 0
        msgOut.force_torque.l_foot_torque_x = 0
        msgOut.force_torque.l_foot_torque_y = 0
        msgOut.force_torque.r_foot_force_z = 0
        msgOut.force_torque.r_foot_torque_x = 0
        msgOut.force_torque.r_foot_torque_y = 0
        msgOut.force_torque.l_hand_force = [0,0,0]
        msgOut.force_torque.l_hand_torque = [0,0,0]
        msgOut.force_torque.r_hand_force = [0,0,0]
        msgOut.force_torque.r_hand_torque = [0,0,0]
 
        #Msg Publish
        self.lc.publish("EST_ROBOT_STATE", msgOut.encode())

if __name__ == "__main__":
    convertor = abbIRB140DRCLCMConvertor()
    print "IRB140DRCLCMConvertor Setup Success.  Running..."
    try:
        while True:
            convertor.lc.handle()
    except KeyboardInterrupt:
        print "KeyboardInterrupt detected.  Convertor Terminated"    
