function [ui,persistentMemory] = controller(missionData,persistentMemory,currentTime)
% Decentralized controller for multi-robot search and rescue mission
% 
% Inputs:
%   +missionData struct containining the following fields:
%       -Submission     : current submission ('init', 'room', 'return', 'refuel') 
%       -SensorData     : 8-by-1 vector of distance to objects in sensor range in the 0,45,...,315 degree directions
%       -AgentState     : 2-by-1 position of current agent
%       -AgentID        : current agent id
%       -NeighborState  : 2-by-|Ni| matrix of visible neighbor states
%       -NeighborIDs    : 1-by-|Ni| vector of neighbor id's
%       -MissionInfo    : Struct containing mission info (e.g., target)
%       -MinSafetyDist  : Minimum safety separation
%       -MaxSensCommRng : Maximum sensing/comm distance
%   +persistentMemory   : 1-by-|Ni| vector of auxiliary flags
%   +currentTime        : current mission time
% Outputs:
%   +ui                 : 2-by-1 velocity reference for current agent
%   +persistentMemory   : 1-by-|Ni| vector of persistent memory scalars per neighbor

% Extracting variable info
xi = missionData.AgentState;
xj = missionData.NeighborStates;
ID = missionData.AgentID;
nIDs = missionData.NeighborIDs;
currentMission = missionData.Submission;
Delta = missionData.MaxSensCommRng;
delta = missionData.MinSafetyDist;
A_formation = missionData.FormationSpec;
sensorData = missionData.SensorData;
% Get sensor directions associated with sensor range data
deltaSensor = delta*0.85/1.25;
numSensors = length(sensorData);
sensorTheta = 0:2*pi/numSensors:2*pi*(1-1/numSensors);
sensorDir = [cos(sensorTheta);sin(sensorTheta)];
% If leader agent, extract target waypoint info
%leaderAgent = isfield(missionData.MissionInfo,'target');
%if leaderAgent
    % Target waypoint
%    xt = missionData.MissionInfo.target;
%end
% Initialize control
ui = zeros(2,1);
p1 = zeros(1,numSensors);

%%%%%%%%%%%%%%%%%%%%%%%%%%%% Modify this block %%%%%%%%%%%%%%%%%%%%%%%%%%%%
switch currentMission
    case 'init'
        for jj = 1:size(xj,2)           
            if nIDs(jj) > ID
                nIDs(jj) = nIDs(jj) -1;
            end
            if jj ~= nIDs(jj)
                nIDs(jj) = jj;
            end
            %% Collision Avaidance
            for kk = 1:numSensors
                
                if sensorData(kk) ~= Inf
                    dij = norm(sensorData(kk)*sensorDir(:,kk));
                    if (p1(kk) == 0) && (dij <= 1.8*delta)
                        
                        p1(kk) = 1;
                    end
                    wij = 1e-3*p1(kk)*(dij^2 - Delta*delta)/...
                        ((Delta - dij) ^ 3 *(dij - delta) ^ 3 );
                    ui = ui + min(0.25, wij)*(sensorData(kk)*sensorDir(:,kk));
                else
                    ui = ui + [0;0];
                end
                
            end 
            %% Formation Control            
            dij = A_formation(1,nIDs(jj));
            nij = norm(xj(:,nIDs(jj)) - xi);
            if abs(nij - dij) < 0.001
                wij = 0;
            else
                if (persistentMemory(nIDs(jj)) == 0) && ((nij < 0.9*Delta) && (nij > 1.6*delta))
                    persistentMemory(nIDs(jj)) = 1;
                    
                end
                 wij = 1e-1*persistentMemory(nIDs(jj))*(nij - dij)*(nij*((Delta - dij) - (dij - delta))...
                     + dij*(Delta + delta) - 2*delta*Delta)/(nij * (Delta - nij)^2*(nij - delta)^2);
            end
            ui = ui + min(0.25, wij)*(xj(:,jj) - xi);
            
        end
        %% Leader consensus with the target
        if ID == 6
            leaderAgent = isfield(missionData.MissionInfo,'target');
            
            if leaderAgent
                % Target waypoint
                xt = missionData.MissionInfo.target;
                ui = ui + min(0.25, norm(xt - xi))*(xt - xi)/norm(xt - xi);
            end
        end
        
    case 'return'
        for jj = 1:size(xj,2)           
            if nIDs(jj) > ID
                nIDs(jj) = nIDs(jj) -1;
            end
            if jj ~= nIDs(jj)
                nIDs(jj) = jj;
            end
            %% Collision Avaidance
            for kk = 1:numSensors
                
                if sensorData(kk) ~= Inf
                    dij = norm(sensorData(kk)*sensorDir(:,kk));
                    if (p1(kk) == 0) && (dij <= 1.6*delta)
                        
                        p1(kk) = 1;
                    end
                    wij = 1e-3*p1(kk)*(dij^2 - Delta*delta)/...
                        ((Delta - dij) ^ 3 *(dij - delta) ^ 3 );
                    ui = ui + min(0.15,wij)*(sensorData(kk)*sensorDir(:,kk));
                else
                    ui = ui + [0;0];
                end
                
            end 
            %% Formation Control  
            
            dij = A_formation(1,nIDs(jj));
            nij = norm(xj(:,nIDs(jj)) - xi);
            if abs(nij - dij) < 0.001
                wij = 0;
            else
                if (persistentMemory(nIDs(jj)) == 0) && (nij < Delta) && (nij > 1.8*delta)
                    persistentMemory(nIDs(jj)) = 1;
                end
                 wij = 1e-1*persistentMemory(nIDs(jj))*(nij - dij)*(nij*((Delta - dij) - (dij - delta))...
                     + dij*(Delta + delta) - 2*delta*Delta)/(nij * (Delta - nij)^2*(nij - delta)^2);
            end
            ui = ui + min(0.15,wij)*(xj(:,jj) - xi);
            
        end
        %% Leader consensus with the target
        
        if ID == 6
            leaderAgent = isfield(missionData.MissionInfo,'target');
            
            if leaderAgent
                % Target waypoint
                xt = missionData.MissionInfo.target;
                ui = ui + min(0.20, norm(xt - xi))*(xt - xi)/norm(xt - xi);
            end
        end
        % works for 0.25 and 1e-1
        % works for 0.20 leader and 1e-1
    case 'room'       
            
        for jj = 1:size(xj,2)           
            if nIDs(jj) > ID
                nIDs(jj) = nIDs(jj) -1;
            end
            if jj ~= nIDs(jj)
                nIDs(jj) = jj;
            end
            %% Collision Avaidance
            for kk = 1:numSensors

                if sensorData(kk) ~= Inf
                    dij = norm(sensorData(kk)*sensorDir(:,kk));
                    if (p1(kk) == 0) && (dij <= 1.15*delta)

                        p1(kk) = 1;
                    end
                    wij = 1e-3*p1(kk)*(dij^2 - Delta*delta)/...
                        ((Delta - dij) ^ 3 *(dij - delta) ^ 3 );
                    ui = ui + min(0.15,wij)*(sensorData(kk)*sensorDir(:,kk));
                else
                    ui = ui + [0;0];
                end

            end 
            %% Formation Control            
            dij = A_formation(1,nIDs(jj));
            nij = norm(xj(:,nIDs(jj)) - xi);
            if abs(nij - dij) < 0.001
                wij = 0;
            else
                if (persistentMemory(nIDs(jj)) == 0) && (nij < 0.9*Delta) && (nij > 1.3*delta)
                    persistentMemory(nIDs(jj)) = 1;
                end
                 wij = 1e-1*persistentMemory(nIDs(jj))*(nij - dij)*(nij*((Delta - dij) - (dij - delta))...
                     + dij*(Delta + delta) - 2*delta*Delta)/(nij * (Delta - nij)^2*(nij - delta)^2);
            end
            ui = ui + min(0.15,wij)*(xj(:,jj) - xi);

        end
        
        %% Leader consensus with the target
        if ID == size(A_formation,2) + 1
            %leaderAgent = isfield(missionData.MissionInfo,'target');

            %if leaderAgent
                % Target waypoint
            if currentTime < 34
                xt = missionData.MissionInfo.domain(:,3);
            elseif currentTime > 35 && currentTime < 68
                xt = missionData.MissionInfo.domain(:,4);
            else
                xt = missionData.MissionInfo.domain(:,1);
            end
            ui = ui + min(0.5, norm(xt - xi))*(xt - xi)/norm(xt - xi);
           
        end
        
    case 'refuel'
        xt = missionData.MissionInfo.target;
        for jj = 1:size(xj,2)           
            if nIDs(jj) > ID
                nIDs(jj) = nIDs(jj) -1; 
            end
            if jj ~= nIDs(jj)
                nIDs(jj) = jj;
            end
            
            %% Collision Avaidance
            for kk = 1:numSensors
                
                if sensorData(kk) ~= Inf
                    dij = norm(sensorData(kk)*sensorDir(:,kk));
                    if (p1(kk) == 0) && (dij <= 1.9*delta)
                        
                        p1(kk) = 1;
                    end
                    wij = 1e-3*p1(kk)*(dij^2 - Delta*delta)/...
                        ((Delta - dij) ^ 3 *(dij - delta) ^ 3 );
                    ui = ui + min(0.15,wij)*(sensorData(kk)*sensorDir(:,kk));
                else
                    ui = ui + [0;0];
                end
                
            end 
            %% Formation Control
            
            dij = A_formation(nIDs(jj));
            nij = norm(xj(:,nIDs(jj)) - xi);
            if abs(nij - dij) < 0.001
                wij = 0;
            else
                if (persistentMemory(nIDs(jj)) == 0) && (nij < 0.9*Delta) && (nij > 1.7*delta)
                    persistentMemory(nIDs(jj)) = 1;
                end
                 wij = 1e-1*persistentMemory(nIDs(jj))*(nij - dij)*(nij*((Delta - dij) - (dij - delta))...
                     + dij*(Delta + delta) - 2*delta*Delta)/(nij * (Delta - nij)^2*(nij - delta)^2);
            end
            ui = ui + min(0.15,wij)*(xj(:,jj) - xi) + min(0.15, norm(xt(:,ID) - xi))*(xt(:,ID) - xi)/norm(xt(:,ID) - xi);
            
        end
        
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%% Modify this block %%%%%%%%%%%%%%%%%%%%%%%%%%%%
end