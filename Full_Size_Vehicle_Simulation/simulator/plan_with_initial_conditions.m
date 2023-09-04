max_planning_time = 0.2;
initial_condition_index = 0;

dir_name = 'initial_conditions/initial_conditions_' + string(max_planning_time) + '/';
filename = dir_name + string(initial_condition_index) + '.mat';
while exist(filename, 'file') ~= 0
    load(filename);
    k_mex = TEST_MEX(agent_state_mex, x_des_mex, dyn_obs_mex);
    initial_condition_index = initial_condition_index + 1;
    filename = dir_name + string(initial_condition_index) + '.mat';
end
display('Complete');