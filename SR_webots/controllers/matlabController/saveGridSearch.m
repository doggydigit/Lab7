
function do_we_stop=saveGridSearch(filename, GS, gps, motor_position, motor_torque, t)
%%
%     input:
%         filename - name of the .mat file where results will be stored
%         GS - the structure containing parameters and metrics
%         gps - GPS values from the current run (X, Y, Z coordinates)
%         motor_position - positions of all motors during the simulation
%         motor_torque - torques of all motors during the simulation
%         t - time vector of the current simulation

%     output:
%         do_we_stop - flag to indicate if grid search is finished



    % An example how to formulate metrics. Use GS.iteration to index
    % correct vector element in GS function
    GS.distance(GS.iteration) = norm(gps([1 3], end) - gps([1 3], 1));
    
    % IMPLEMENT your speed and energy metrics
%     GS.speed(GS.iteration) = ...
%     GS.energy(GS.iteration) = ...





    % Saving current GS into .mat file to preserve data between different
    % iterations
    save(filename, 'GS');

    if GS.iteration >= GS.maxIter
%         wb_supervisor_simulation_set_mode(WB_SUPERVISOR_SIMULATION_MODE_PAUSE);
        do_we_stop=1;
    else 
        wb_supervisor_simulation_revert;
        do_we_stop=0;
    end


end