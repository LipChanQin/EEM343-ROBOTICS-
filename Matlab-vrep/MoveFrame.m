function MoveFrame(clientID,vrep,m,h,frel )
%Get the rotational matrix 
    rot = m(1:3, 1:3);
%Get the position 
    pos = m(1:3,4);
%Get theta
    theta=EulerZYX_inv(rot);
%Set the Orientation &position
    res=vrep.simxSetObjectPosition(clientID,h,frel,pos,vrep.simx_opmode_oneshot);
    res=vrep.simxSetObjectOrientation(clientID,h,frel,theta,vrep.simx_opmode_oneshot);
end