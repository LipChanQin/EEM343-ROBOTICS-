function MoveUR10joints(clientID, vrep, handles, Theta,UR10Num )
    if UR10Num ==1 
        % Make theta within the range
            targetTheta=Theta+handles.startingJoints; %joint angle needs to rotate
    
        for i= 1:6
            if targetTheta(i) > pi
                targetTheta(i) = targetTheta(i) - 2*pi;
            elseif targetTheta(i) < -pi
                targetTheta(i) = targetTheta(i) + 2*pi;
            end
        end 
    
        % Adjust jiont0 angle
        targetTheta(1)=targetTheta(1)+(pi/2);
        A = targetTheta;
    
        % Act
        for i = 1:6
            res=vrep.simxSetJointTargetPosition(clientID,handles.UR10joints(i),...
            targetTheta(i),vrep.simx_opmode_oneshot);
        end
        %Check action 
            currentTheta = zeros(1,6);    

        while true                        
        % Get current joint angles for each joint
            for i = 1:6
                [res,currentTheta(i)] = vrep.simxGetJointPosition(clientID, handles.UR10joints(i),...
                                         vrep.simx_opmode_oneshot_wait);
            end  
            % Check joints 
            diffJoints = currentTheta - targetTheta;
            if max(abs(diffJoints)) < handles.threshold
               break; 
            end
        end 
        
    elseif UR10Num == 2
        % Make theta within the range
            targetTheta=Theta+handles.startingJoints; %joint angle needs to rotate
    
        for i= 1:6
            if targetTheta(i) > pi
                targetTheta(i) = targetTheta(i) - 2*pi;
             elseif targetTheta(i) < -pi
                targetTheta(i) = targetTheta(i) + 2*pi;
            end
        end 
    
        % Adjust jiont0 angle
            targetTheta(1)=targetTheta(1)+(pi/2);
            B = targetTheta;
        % Act
        for i = 1:6
            res=vrep.simxSetJointTargetPosition(clientID,handles.ur10joints(i),...
                targetTheta(i),vrep.simx_opmode_oneshot);
        end
        %Check action 
            currentTheta = zeros(1,6);    

        while true                        
        % Get current joint angles for each joint
            for i = 1:6
                [res,currentTheta(i)] = vrep.simxGetJointPosition(clientID, handles.ur10joints(i),...
                                         vrep.simx_opmode_oneshot_wait);
            end  
            % Check joints 
            diffJoints = currentTheta - targetTheta;
            if max(abs(diffJoints)) < handles.threshold
               break; 
            end
        end 
        
    end
end