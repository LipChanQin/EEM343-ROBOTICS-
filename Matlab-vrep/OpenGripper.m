function OpenGripper(clientID,vrep,handles,velocity,GripperNum)
    if GripperNum ==1
        % Open the gripper
        res=vrep.simxSetJointForce(clientID,handles.gripper_1,150,vrep.simx_opmode_oneshot);
        res=vrep.simxSetJointTargetVelocity(clientID,handles.gripper_1,velocity,vrep.simx_opmode_oneshot);
        pause(2.5);

        % Stop  
        res=vrep.simxSetJointTargetVelocity(clientID,handles.gripper_1,0,vrep.simx_opmode_oneshot);
    
    elseif GripperNum ==2
        % Open the gripper
        res=vrep.simxSetJointForce(clientID,handles.gripper_2,150,vrep.simx_opmode_oneshot);
        res=vrep.simxSetJointTargetVelocity(clientID,handles.gripper_2,velocity,vrep.simx_opmode_oneshot);
        pause(2.5);

        % Stop  
    	res=vrep.simxSetJointTargetVelocity(clientID,handles.gripper_2,0,vrep.simx_opmode_oneshot);
    end
end