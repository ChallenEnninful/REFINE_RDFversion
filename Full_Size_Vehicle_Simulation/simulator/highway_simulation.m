% Main function for REFINE simulation

% load FRS
frs_filename = 'car_frs.mat';
if ~exist('frs','var')
    disp('Loading frs')
    frs = load(frs_filename) ;
else
    disp('table already loaded') ;
end

visualize = 0; %need to turn off to stop visualization
visualize = 0; %need to turn off to stop visualization
plot_sim_flag = visualize; %if visualize is on plot_sim_flag should also be on
plot_AH_flag = 1;
save_result = 1; % make it true if you want to save the simulation data
save_video = false; %make it true if you want to save videos of the trials


%% set up required objects for simulation
lanewidth = 3.7;
bounds = [0, 2000, -(lanewidth / 2) - 1, ((lanewidth/2) * 5) + 1];
goal_radius = 12;
world_buffer = 1 ;
t_move = 3;
t_plan = 3;
t_failsafe_move = 3;
verbose_level = 0;
num_ego_vehicles = 1;
% num_moving_cars = 15;
% num_static_cars = 3;
num_moving_cars = 30;
num_static_cars = 0;
num_moving_cars = 30;
num_static_cars = 0;
num_total_cars = num_ego_vehicles + num_moving_cars + num_static_cars;
hlp_lookahead = 90;
lane_changeFRS_log = {};
SIM_MAX_DISTANCE = 400; %maximum distance along highway to spawn a vehicle. Should not be >900 since highway is only 1000m
min_obs_start_dist = 50; %min distance away from ego vehicle that the obstacle vehicles can be spawned
min_obs_start_dist = 50; %min distance away from ego vehicle that the obstacle vehicles can be spawned
car_safe_dist = 1; %min allowable distance between obstacle vehicles
num_trials=100;

%% Error catching for simulation setup
if save_video && ~visualize
    error("Error. Visualize flag needs to enabled on if save_video flag is enabled")
end

%%
max_planning_time = 0.25;
save_dir_prefix = "RecedingHorizonPlanning/" + string(num_trials) + "Trials_" + string(num_moving_cars) + "Obs_" + string(SIM_MAX_DISTANCE) + "obsLength_" + string(max_planning_time) + "MaxTime";
for j = 1:num_trials
    % RESET simulation environment
    World = dynamic_car_world( 'bounds', bounds, ...
        'buffer', world_buffer, 'goal', [1010;3.7], ...
        'verbose', verbose_level, 'goal_radius', goal_radius, ...
        'num_cars', num_total_cars, 'num_moving_cars', num_moving_cars, ...
        't_move_and_failsafe', t_move+t_failsafe_move, ...
        'SIM_MAX_DISTANCE',SIM_MAX_DISTANCE,'min_obs_start_dist',min_obs_start_dist, ...
         'car_safe_dist',car_safe_dist) ;

    Agent = highway_cruising_10_state_agent; % takes care of vehicle states and dynamics
    Agent.desired_initial_condition = [10;0; 0; 20;0;0;20;0;0;0];
    Agent.integrator_type= 'ode45';
    HLP = simple_highway_HLP; % high-level planner
    HLP.lookahead = hlp_lookahead; 
    AgentHelper = highwayAgentHelper_timed(Agent,frs,HLP,'t_plan',t_plan,'t_move',t_move,'t_failsafe_move',t_failsafe_move,...
        'verbose',verbose_level); % takes care of online planning
    AgentHelper.FRS_u0_p_maps = load("u0_p_maps.mat");
    Simulator = rlsimulator(AgentHelper,World,'plot_sim_flag',plot_sim_flag,'plot_AH_flag',plot_AH_flag,'save_result',save_result,...
        'save_video',save_video,'epscur',j,'visualize',visualize,'save_dir_prefix',save_dir_prefix);

    AgentHelper.S = Simulator;
    Simulator.eval = 1; %turn on evaluation so summary will be saved

   

    rng(j+1);
    IsDone4 = 0;
    Simulator.epscur = j;
    Simulator.reset();
    jk=1;
    for i = 1:4000
        
        AgentHelper.planned_path = [linspace(0,1000);repmat([0;0],1,100)];
        [~,~,IsDone,LoggedSignal]=Simulator.step([rand*2-1;rand*2-1]);
%         if jk>1
%             timediff = AgentHelper.A.time(end) - lane_changeFRS_log{jk-1}.global_time;
%         end
% 
%         if i>1 && timediff >6 && strcmp(AgentHelper.SDF_trial_FRSdata.man_type ,'lan')
%             lane_changeFRS_log{jk} = AgentHelper.SDF_trial_FRSdata;
%             jk=jk+1;
%         elseif jk==1
%             lane_changeFRS_log{jk} = AgentHelper.SDF_trial_FRSdata;
%             jk=jk+1;
%         elseif i>1 && ~strcmp(AgentHelper.SDF_trial_FRSdata.man_type ,'lan')
%             lane_changeFRS_log{jk} = AgentHelper.SDF_trial_FRSdata;
%             jk=jk+1;
%         end
            


%         lane_changeFRS_log{i} = AgentHelper.SDF_trial_FRSdata;  
%         if i == 1 && noskip
%             lane_changeFRS_log{j} = AgentHelper.SDF_trial_FRSdata;
%         end

        

        if IsDone == 1 || IsDone == 3 || IsDone == 4 || IsDone == 5
            %crash
            %      crash with safety layer on
            %                      safely stopped but stuck
            %                                           reached goal!
            break
        end
    end
    pause(1)
end



done = 'Simulation Complete'
