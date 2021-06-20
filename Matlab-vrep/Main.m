%% EEM343 Robotics Mini Project (Group 3)
% Theme: Assisting the Nation during Pandemic
% Title: Drive-through goods picking robot in grocery sector
% Name: Leong Kong Yue (141560)
%       Lip Chan Qin   (143989)
%       Lu Hao Ran     (144921)
%       Teh Tong Yu    (144274)

vrep=remApi('remoteApi');       
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID>-1)
        disp('Connected to remote API server');
end
 
%% Get Object Handle
% For two UR10 joints
 
UR10joints = {'UR10_joint1','UR10_joint2','UR10_joint3','UR10_joint4',...
     'UR10_joint5','UR10_joint6'};      % First UR10 arm
     
ur10joints = {'ur10_joint1','ur10_joint2','ur10_joint3','ur10_joint4',...
     'ur10_joint5','ur10_joint6'};      % Second UR10 arm
 
handles.UR10joints = -ones(1,6);
handles.ur10joints = -ones(1,6);

for i = 1:6
    [res,handles.UR10joints(i)]=vrep.simxGetObjectHandle(clientID,...
                UR10joints{i},vrep.simx_opmode_blocking);
            
    [res,handles.ur10joints(i)]=vrep.simxGetObjectHandle(clientID,...
                ur10joints{i},vrep.simx_opmode_blocking);
end
 
% For item1 and item2
ItemNum = {'Item1','Item2'};
handles.item= -ones(1,2);

for i = 1:2
    [res,handles.item(i)]=vrep.simxGetObjectHandle(clientID,...
                    ItemNum{i},vrep.simx_opmode_blocking);
end
 
% For frame 0,frame 1 and frame2
handles.base_1 = -1;
handles.base_2 = -1;
handles.target = -1;
[res,handles.base_1]=vrep.simxGetObjectHandle(clientID,...
            'Frame0',vrep.simx_opmode_blocking);

[res,handles.base_2]=vrep.simxGetObjectHandle(clientID,...
            'Frame2',vrep.simx_opmode_blocking);
 
[res,handles.target]=vrep.simxGetObjectHandle(clientID,...
            'Frame1',vrep.simx_opmode_blocking);
        
% For conveyor 
handles.conveyor = -1;
[res,handles.conveyor]=vrep.simxGetObjectHandle(clientID,...
            'customizableConveyor',vrep.simx_opmode_blocking);

% For table
handles.table = -1;
[res,handles.table]=vrep.simxGetObjectHandle(clientID,...
            'customizableTable',vrep.simx_opmode_blocking);

% For proximity sensor
handles.proximitysensor = -1;
[res,handles.proximitysensor]=vrep.simxGetObjectHandle(clientID,...
            'Proximity_sensor',vrep.simx_opmode_blocking);
        
% For gripper1 and gripper2
handles.gripper_1 = -1;
handles.gripper_2 = -1;
[res,handles.gripper_1]=vrep.simxGetObjectHandle(clientID,...
            'RG2_openCloseJoint',vrep.simx_opmode_blocking);

[res,handles.gripper_2]=vrep.simxGetObjectHandle(clientID,...
            'rg2_opencloseJoint',vrep.simx_opmode_blocking);
 
%% Stream (Get position/orientation of object and joint)
relativetoRef_1 = handles.base_1;
relativetoRef_2 = handles.base_2;
 
% For item1 and item2
for i = 1:2
    res=vrep.simxGetObjectPosition(clientID,handles.item(i),relativetoRef_1,...
            vrep.simx_opmode_streaming);
    
    res=vrep.simxGetObjectOrientation(clientID,handles.item(i),relativetoRef_1,...
            vrep.simx_opmode_streaming);
    
    res=vrep.simxGetObjectPosition(clientID,handles.item(i),relativetoRef_2,...
            vrep.simx_opmode_streaming);
    
    res=vrep.simxGetObjectOrientation(clientID,handles.item(i),relativetoRef_2,...
            vrep.simx_opmode_streaming);
end

% For conveyor
res=vrep.simxGetObjectPosition(clientID,handles.conveyor,relativetoRef_1,...
        vrep.simx_opmode_streaming);

res=vrep.simxGetObjectOrientation(clientID,handles.conveyor,relativetoRef_1,...
        vrep.simx_opmode_streaming);
 
% For table
res=vrep.simxGetObjectPosition(clientID,handles.table,relativetoRef_2,...
        vrep.simx_opmode_streaming);

res=vrep.simxGetObjectOrientation(clientID,handles.table,relativetoRef_2,...
        vrep.simx_opmode_streaming);

% For proximity sensor
res=vrep.simxReadProximitySensor(clientID,handles.proximitysensor,vrep.simx_opmode_streaming);

% For gripper 1 and gripper 2
res=vrep.simxGetJointPosition(clientID,handles.gripper_1,...
        vrep.simx_opmode_streaming);

res=vrep.simxGetJointPosition(clientID,handles.gripper_2,...
        vrep.simx_opmode_streaming);
 
% For joints of two arms 
for i = 1:6
    res=vrep.simxGetJointPosition(clientID,handles.UR10joints(i),...
        vrep.simx_opmode_streaming);
    
    res=vrep.simxGetJointPosition(clientID,handles.ur10joints(i),...
        vrep.simx_opmode_streaming);
end

%% Simulation

% Set the threshold to check if the end effector has reached its destination
handles.threshold = 0.01;

%Set The Arm Parameters Using Peter Corke robotics toolbox
handles.UR10robot = DHP('UR10');
pause(0.5);
%Rest Joint for 1st time
handles.startingJoints = [0,0,0,0,0,0];
res=vrep.simxPauseCommunication(clientID,true);

for i = 1:6
    vrep.simxSetJointTargetPosition(clientID,handles.UR10joints(i),...
        handles.startingJoints(i),vrep.simx_opmode_oneshot);
    
     vrep.simxSetJointTargetPosition(clientID,handles.ur10joints(i),...
        handles.startingJoints(i),vrep.simx_opmode_oneshot);
end

res = vrep.simxPauseCommunication(clientID, false);
pause(1);

%Here begins
while true
     str = input('Which item do u want?\n[num:Item(num)]\n[N:No]\n','s');
     if str == '1'
        ItemNum = 1;
        GoAndPickCuboidFromRack(clientID,vrep,handles,ItemNum);
        GoAndLeaveCuboidOnConveyor(clientID,vrep,handles);

        %read proximity sensor 
        [res,detectionState,detectedPoint,~,~]=vrep.simxReadProximitySensor(clientID,...
                handles.proximitysensor,vrep.simx_opmode_buffer);
        distance = round(norm(detectedPoint),2);

        while distance > 0 
            [res,detectionState,detectedPoint,~,~]=vrep.simxReadProximitySensor(clientID,...
                    handles.proximitysensor,vrep.simx_opmode_buffer);
            distance = round(norm(detectedPoint),2);
    
            if distance < 1
                GoAndPickCuboidFromConveyor(clientID,vrep,handles,ItemNum);
                GoAndLeaveCuboidOnTable(clientID,vrep,handles)
                pause(3);
                m= eye(4,4);
                m(1:3,4) = [-0.8777 0.0250 1.0610];
                MoveFrame(clientID,vrep,m,handles.item(1),handles.base_1);
                break;
            end
        end
        
     elseif str == '2'
        ItemNum = 2;
        GoAndPickCuboidFromRack(clientID,vrep,handles,ItemNum);
        GoAndLeaveCuboidOnConveyor(clientID,vrep,handles);

        %read proximity sensor 
        [res,detectionState,detectedPoint,~,~]=vrep.simxReadProximitySensor(clientID,...
                handles.proximitysensor,vrep.simx_opmode_buffer);
        distance = round(norm(detectedPoint),2);

        while distance > 0 
            [res,detectionState,detectedPoint,~,~]=vrep.simxReadProximitySensor(clientID,...
                        handles.proximitysensor,vrep.simx_opmode_buffer);
            distance = round(norm(detectedPoint),2);
    
            if distance < 1
                GoAndPickCuboidFromConveyor(clientID,vrep,handles,ItemNum);
                GoAndLeaveCuboidOnTable(clientID,vrep,handles);
                pause(3);
                m= eye(4,4);
                m(1:3,4) = [-0.8779 0.3250 1.0610];
                MoveFrame(clientID,vrep,m,handles.item(2),handles.base_1);
                break;
            end
        end
     elseif str == 'N'
         break;
    
     else
         disp('No specified item available!')
     end
end
disp('Thank you for your service!')
vrep.delete;  
clear;
