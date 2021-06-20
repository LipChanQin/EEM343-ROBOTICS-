function GoAndLeaveCuboidOnTable(clientID,vrep,handles)
% Get the Target 
    relativeToRef_2 = handles.base_2;
    [res,TargPos]=vrep.simxGetObjectPosition(clientID,handles.table,...
                relativeToRef_2,vrep.simx_opmode_buffer);
    
    [res,TargTheta]=vrep.simxGetObjectOrientation(clientID,handles.table,...
                relativeToRef_2,vrep.simx_opmode_buffer);
    
    RG2GripperOffset= [0 -0.2 0.32];
    TotOffset=RG2GripperOffset;
   
%% Approaching the table
    %Calculate And make offset
        M=EulerZYX(TargTheta)*ROT('Z',pi/2)*ROT('X',pi);
        M(1:3, 4)= TargPos + TotOffset;
        M=double(M);
        
    %Get the joint angle (in radian)
        q = handles.UR10robot.ikunc(M);
    
    %Move frame 1
        MoveFrame(clientID, vrep, M, handles.target, relativeToRef_2);
        pause(0.5);
    
    %Move the arm of UR10
        MoveUR10joints(clientID,vrep,handles,q,2);

%% When reached, drop the cuboid
    OpenGripper(clientID,vrep,handles,0.02,2);
    pause(0.5);
    
%% Set the arm to its original configuration
    for i = 1:6
        vrep.simxSetJointTargetPosition(clientID, handles.ur10joints(i),...
                  handles.startingJoints(i),vrep.simx_opmode_oneshot);
    end
    pause(1);
end