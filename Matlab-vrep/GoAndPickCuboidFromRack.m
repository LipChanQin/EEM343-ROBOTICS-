function GoAndPickCuboidFromRack(clientID,vrep,handles,ItemNum)
% Get the Target 
    relativeToRef_1 = handles.base_1;
    [res,TargPos]=vrep.simxGetObjectPosition(clientID,handles.item(ItemNum),...
                relativeToRef_1,vrep.simx_opmode_buffer);
    
    [res,TargTheta]=vrep.simxGetObjectOrientation(clientID,handles.item(ItemNum),...
                relativeToRef_1,vrep.simx_opmode_buffer);
    
    RG2GripperOffset= [0.22 0 0.035];
    TotOffset=RG2GripperOffset;
  
%Open the gripper
        OpenGripper(clientID,vrep,handles,0.02,1);

%% Approaching to the cuboid
    %Calculate And make offset
        M=EulerZYX(TargTheta)*ROT('Y',-pi/2);
        M(1:3, 4)= TargPos + TotOffset;
        M=double(M);
        
    %Get the joint angle (in radian)
        q = handles.UR10robot.ikunc(M);
    
    %Move frame 1
        MoveFrame(clientID, vrep, M, handles.target, relativeToRef_1);
        pause(0.5);
        
    %Move the arm of UR10
        MoveUR10joints(clientID,vrep,handles,q,1);
      
%% When reached, pick the cuboid
    CloseGripper(clientID,vrep,handles,0.02,1);
    pause(0.5);
 
%% Set the arm to its original configuration
    for i = 1:6
        vrep.simxSetJointTargetPosition(clientID, handles.UR10joints(i),...
                  handles.startingJoints(i),vrep.simx_opmode_oneshot);
    end
        
    pause(1);   
end