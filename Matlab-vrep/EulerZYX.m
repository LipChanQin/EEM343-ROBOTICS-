function [ R ] = EulerZYX( Theta )
    R = ROT('X',Theta(1)) * ROT('Y',Theta(2)) * ROT('Z',Theta(3));
end