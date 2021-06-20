function CloseGripper(clientID,vrep,handles,velocity,GripperNum)
    if GripperNum == 1
        res=vrep.simxSetJointForce(clientID,handles.gripper_1,150,vrep.simx_opmode_oneshot);
        res=vrep.simxSetJointTargetVelocity(clientID,handles.gripper_1,-velocity,vrep.simx_opmode_oneshot);
        pause(2);
        
    elseif GripperNum == 2
        res=vrep.simxSetJointForce(clientID,handles.gripper_2,150,vrep.simx_opmode_oneshot);
        res=vrep.simxSetJointTargetVelocity(clientID,handles.gripper_2,-velocity,vrep.simx_opmode_oneshot);
        pause(2);
    end
end