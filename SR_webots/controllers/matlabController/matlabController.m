clc
clear all
close all
warning off

%% INITIALIZATION
% include needed folders/files
addpath([pwd '/extras']);

% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:
% desktop;
% keyboard;

% do we run a WEBOTS or MATLAB simulation? 
if exist('wb_robot_step', 'file') 
    IS_WEBOTS_SIMULATION=true; 
else
    IS_WEBOTS_SIMULATION=false; 
end

% simulation parameters
CONTROLLER_TIME_STEP = 20;      % in ms
dt=CONTROLLER_TIME_STEP/1000;

% get and enable devices:
if IS_WEBOTS_SIMULATION; 
    robot=initializeSRLink(CONTROLLER_TIME_STEP); 
end;

% initialize time
k=1;
t=0;

% initialize q
q=zeros(14,1);

%% SWIMMING PARAMETERS 
% initialize parameters <- FREE TO CHANGE
f=1;
R=0.3;
phi_total=2*pi;

%% GRID SEARCH INITIALIZATION (can overwrite the parameters above)
% Simulation length - only for Matlab simulation and GRID SEARCH
Trun = 30;   

% the flag indicating whether we use grid search or not
GRID_SEARCH=false;
if GRID_SEARCH && IS_WEBOTS_SIMULATION
    % specify name of the .mat file which will contain grid search results
    % every time simulation starts with a new parameters, variables in the
    % workspace are lost. Therefore, storing them into .mat and saving is
    % the only way to preserve data between grid search iterations
    filename='gridSearch1';
    
    % define search parameters as vectors of equaly spaced values. MODIFY AS NEEDED
    phi_total_vector=linspace(pi/4, 4*pi, 4);
    R_vector=linspace(0.01, 1, 4);
    
    % In every iteration, this function returs a pair of search parameters MODIFY AS NEEDED
    [phi_total, R, GS]=initializeGridSearch(filename, phi_total_vector, R_vector);    
end
    
%% MAIN CONTROLLER LOOP
while true  
    % check if we run WEBOTS or MATLAB simulation
    if IS_WEBOTS_SIMULATION
        if wb_robot_step(CONTROLLER_TIME_STEP)== -1
            break;
        end
    else
        if t(k)>Trun
            break
        end
    end
    

    % RUN THE CONTROLLER
    % IMPLEMENT double chain of oscillators inside this function:
    [theta, r]=runCPGNetwork(f, phi_total, R, dt);                                                                  % <- IMPLEMENT INSIDE
    
    % IMPLEMENT relation to get joint angle reference from the CPG output:
    qs=spineController(theta, r);                                                                                   % <- IMPLEMENT INSIDE

    
    
    % leg angles set to swimming position
    qlimb=[-pi/2,-pi/2,-pi/2,-pi/2]';    
    % collect all joint angles for the robot
    q=[qs;qlimb];
    
    
    % interface to the robot
    if IS_WEBOTS_SIMULATION 
        % send angles to the Webots simulator
        sendCommands2Robot(robot, q); 
        
        % read robot's GPS
        gps(1:3, k)=wb_gps_get_values(robot.gps);
        
        % read motor angles and torques from Webots
        for jj=1:14
            motor_torque(jj, k) = wb_motor_get_torque_feedback(robot.motor(jj));
            motor_position(jj, k) = wb_position_sensor_get_value(robot.positionSensor(jj));
        end
        if GRID_SEARCH && t(k)>Trun
            % When maximum simulation time is achieved, save search results
            % IMPLEMENT your search metrics (speed, energy etc.) inside this function. Modify inputs if needed
            if saveGridSearch(filename, GS, gps, motor_position, motor_torque, t);                                 % <- IMPLEMENT INSIDE
                % if grid search is done, pause simulation and stop the control loop 
                wb_console_print('GRID SEARCH FINISHED', WB_STDOUT);
                break;
            end
        end
    else
        % visualize robot's kinematics in matlab
        p=SR_forwardKinematics(q);
        
    end
    q_log(1:14,k)=q;
    % track time
    t(k+1,1)=t(k,1)+dt;
    k=k+1;
end
%% save logged variables after simulation is finished (RELOAD button must be pressed)
if IS_WEBOTS_SIMULATION
    t(end)=[];    %remove last element from time vector to match length of other stored variables
    dlmwrite('./logs/time_log.txt', t, '\t');
    dlmwrite('./logs/gps_log.txt', gps', '\t');
    dlmwrite('./logs/angles_reference_log.txt', q_log', '\t');
    dlmwrite('./logs/angles_feedback_log.txt', motor_position', '\t');
    dlmwrite('./logs/torque_log.txt', motor_torque', '\t');
end
disp('DONE');
