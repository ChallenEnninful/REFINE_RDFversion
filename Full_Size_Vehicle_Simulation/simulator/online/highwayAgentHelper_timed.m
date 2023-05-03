classdef highwayAgentHelper_timed < agentHelper
    %% properties
    properties
        HLP;
        
        % partitions on u0
        u0_array        
        
        % reference data for plot
        ref_Z = [];
        proposed_ref_Z = [];
        t_real_start = [];
        t_proposed_start = [];
        
        prev_action = -1;
        cur_t0_idx = 1;
        saved_K = [];
        t_plan;    
        S 

        FRS_helper_handle = struct;
        truncating_factor;

        
        waypt_hist = [];
        K_hist = [];
        FRS_hist = {};
        mirror_hist = [];
        state_hist = [];
        type_manu_hist = [];
        time_hist = [];
        solve_time_hist = [];

        %Challen added for Jon
        SDF_trial_FRSdata = struct;

        FRS_plot_struct = struct;
        FRS_u0_p_maps = struct;
        FRS_spd = struct;
        FRS_dir = struct;
        FRS_lan = struct;

    end
    %% methods
    methods
        function AH = highwayAgentHelper_timed(A,FRS_obj,HLP,varargin)
            AH@agentHelper(A,FRS_obj,varargin{:});
            AH.HLP = HLP;
            info_file_dir = load('dir_change_Ay_info.mat');
%             info_file_lan = load('lane_change_Ay_info.mat');
            AH.u0_array = info_file_dir.u0_vec; 
            AH.truncating_factor = 1;
        end
        
        
        
        function [K,tout] = gen_parameter_standalone(AH, world_info, agent_state,waypts)
            % main online planning function
            x_des = waypts(:,1);
            
            if (AH.cur_t0_idx > 1 && AH.prev_action == 2) || (AH.cur_t0_idx > 2 && AH.prev_action == 3)|| AH.prev_action  == 1
                AH.prev_action = -1;
                AH.cur_t0_idx = 1;
            end

            if AH.prev_action ~= -1 
                K = [AH.saved_K(1); AH.saved_K(2); AH.cur_t0_idx ;AH.prev_action];
                AH.cur_t0_idx = AH.cur_t0_idx + 1;
                return
            end

            x_des_mex = x_des;
            dyn_obs_mex = get_obs_mex(world_info.dyn_obstacles, world_info.bounds);
            agent_state_mex = agent_state(1:6);

            %FOR JON, do local frame transformation of obstacles in order
            %to output vertices for Qingyi
            dyn_obs_local = dyn_obs_mex;
            world_to_local(agent_state_mex(1:3), dyn_obs_mex(1:3,:))
            dyn_obs_local(1:3,:) = world_to_local(agent_state_mex(1:3), dyn_obs_mex(1:3,:));

            %% ####################### NEW GENERATOR METHOD FOR QINGYI

                %loop over FRS's to output the corresponding obstacle
                %generators in local frame

            obs_zono = zeros(length(dyn_obs_local),4,2);
            obs_zono_mirr = zeros(length(dyn_obs_local),4,2);
            v = 25; %max obstacle velocity  


                for jj = 1:length(dyn_obs_local)
                    cx = dyn_obs_local(1,jj);
                    cy = dyn_obs_local(2,jj);
                    h = dyn_obs_local(3,jj);
%                     v = dyn_obs_local(4,jj); %using actual velocity
                      
                    l = dyn_obs_local(5,jj);
                    w = dyn_obs_local(6,jj);
                    
                    rot_h = [cos(h) -sin(h);
                             sin(h) cos(h)];
                    rot_h_mirror = [cos(-h) -sin(-h);
                                    sin(-h) cos(-h)];
                    
                    %mirrored centers
                    cent_mirr = [cx;-cy];

                    %generate rotated generators 
                    gens = rot_h*[l/2 0;0 w/2];
                    vgen = rot_h*[v/2;0];

                    %generate rotated mirrored generators 
                    gens_mirr = rot_h_mirror*[l/2 0;0 w/2];
                    vgen_mirr = rot_h_mirror*[v/2;0];
                    
                    obs_zono_jj = [cx,cy;
                                gens';
                                vgen'];
                    obs_zono_mirr_jj  = [cent_mirr';
                                gens_mirr';
                                vgen_mirr'];
                    
                    obs_zono(jj,:,:) = obs_zono_jj;
                    obs_zono_mirr(jj,:,:) = obs_zono_mirr_jj;
                    
                end

               %% ######################## Timing of constraints and gradients evaluation

                u0_FRS_ranges = AH.FRS_u0_p_maps.u0map;
                urange_diff = abs(u0_FRS_ranges - agent_state(4));
    %             [~,idxu0_minpot] = min(urange_diff);
                [~,idxmin]  = min(sum(urange_diff,2));
                idxu0 = min(idxmin);
    
    %             idxu0_z = [];
    %             for kk = 1:length(u0_FRS_ranges)
    %                 if agent_state(4) >= u0_FRS_ranges(kk,1) && agent_state(4) <= u0_FRS_ranges(kk,2)
    %                     idxu0_z = [idxu0_z,kk];
    %                 end
    %             end
    %             idxu0 = min(idxu0_minpot);
    
    
                spd_FRS_ranges = AH.FRS_u0_p_maps.pmap(idxu0).spd;
                lan_FRS_ranges = AH.FRS_u0_p_maps.pmap(idxu0).lan;
                dir_FRS_ranges = AH.FRS_u0_p_maps.pmap(idxu0).dir;
    
                min_spd_au = 0; %min speed that the vehicle can change speed to in each maneuver
                max_spd_au = agent_state_mex(4) + 4; %max speed that the vehicle can speed up to in each maneuver
                bin_indices_i = [];
                bin_indices_j = [];
                FRS_types = [];
                obstacles_are_mirrored = [];
                way_point = [];
                spd_FRS_range_feas = [];
    
                for ii = 1:3
                    if ii ==1
    
                        for i = 1:length(spd_FRS_ranges)
                            if spd_FRS_ranges(i,1) >= min_spd_au && spd_FRS_ranges(i,2) <= max_spd_au
                                bin_indices_i = [bin_indices_i,idxu0-1];
                                bin_indices_j = [bin_indices_j,i-1];
                                FRS_types = [FRS_types,ii-1];
                                obstacles_are_mirrored = [obstacles_are_mirrored,0];
                                spd_FRS_range_feas = [spd_FRS_range_feas; spd_FRS_ranges(i,:)];
                            
                            end
    
                        end
    
                    elseif ii == 2
    
                        %Don't look at direction changes if they are outside the linear regime (<= 7.0m/s)
                        if u0_FRS_ranges(idxu0,1) > 7 
                            for i = 1:length(dir_FRS_ranges)
                                %add double the number of indices to account
                                %for mirrored FRSes
                                bin_indices_i = [bin_indices_i,idxu0-1,idxu0-1];
                                bin_indices_j = [bin_indices_j,i-1,i-1];
                                FRS_types = [FRS_types,ii-1,ii-1];
                                obstacles_are_mirrored = [obstacles_are_mirrored,0,1]; 
                                
                            end
    
                        end
                     
                    elseif ii == 3
    
                        %Don't look at lane changes if they are outside the linear regime (<= 7.0m/s)
                        if u0_FRS_ranges(idxu0,1) > 7 
                            for i = 1:length(lan_FRS_ranges)
                                %add double the number of indices to account
                                %for mirrored FRSes
                                bin_indices_i = [bin_indices_i,idxu0-1,idxu0-1];
                                bin_indices_j = [bin_indices_j,i-1,i-1];
                                FRS_types = [FRS_types,ii-1,ii-1];
                                obstacles_are_mirrored = [obstacles_are_mirrored,0,1]; 
                                
                            end
    
                        end
                    end
    
                end
    
    
                %% Call New function to run constraint evaluation time test
                %comment out during normal operation

%                 num_k = 10; %number of k values you want to evaluate
%                 tic
%                 times = time_cons_eval(AH,agent_state_mex, x_des_mex, dyn_obs_mex,...
%                         spd_FRS_ranges,lan_FRS_ranges,dir_FRS_ranges,...
%                         FRS_types,bin_indices_j,obstacles_are_mirrored,num_k);
% 
%                 toc








%% ###########Continue optimization
            %continue with original code and run C++ optimization
            tic
            k_mex = TEST_MEX(agent_state_mex, x_des_mex, dyn_obs_mex);
            
            t_temp = toc;
            AH.solve_time_hist = [AH.solve_time_hist, t_temp];
            indices = k_mex(5:6)+1;
            k_mex = k_mex(1:4);
            K = k_mex;
            if K(end) == -1
                K = [];
            else % visualize FRS and update for the agent
                type_manu = K(4);
                multiplier = 1;
                mirror_flag = 0;

                type_manu_all = ["Au","dir","lan"];
                type_text = type_manu_all(type_manu);
                [~,idxu0] = min(abs(AH.u0_array - agent_state(4)));
                M = AH.zono_full.M_mega{idxu0};

                %% test block to compute max time interval in FRS set
%                 dt_max  = 0;
%                 for idx = 1: length(AH.zono_full.M_mega)
%                     M = AH.zono_full.M_mega{idx};
%                     for i = 1:3
%                         type_text = type_manu_all(i);
%                         FRS = M(type_text);
%                         for jj = 1:2
%                             try 
%                                 FRS = FRS{jj,1};
%                                 if isempty(FRS)
%                                     continue
%                                 end
%                             catch
%                                 continue
%                             end
%                             dstack = zeros(length(FRS.vehRS_save),1);
%                             dstack2 = zeros(length(FRS.vehRS_save),1);
%                             parfor ii=1:length(FRS.vehRS_save)
%                                 Zi = interval(FRS.vehRS_save{1,ii}) %generate interval representation of zonotope
%                                 dstack(ii,1) = infimum(Zi(20)); %20th index corresponds to the interval representation for the time center and generators
%                                 dstack2(ii,1) = supremum(Zi(20));
%                             end
%             
%                             interval_size = dstack2 - dstack;
%                             dt = max(interval_size);
%                             if dt > dt_max
%                                 dt_max = dt;
%                             end
%                         end
% 
%                     end
% 
%                 end

                %%
                if type_manu == 1
                    k = K(1);
                    FRS = M(type_text);
                    AH.prev_action = -1; 
                    AH.cur_t0_idx = 1;
                else
                    k = K(2);
                    if k<0
                        multiplier = -1;
                        mirror_flag = 1;
                    end
                    FRS = M(type_text); 
                    AH.prev_action = type_manu;
                    AH.cur_t0_idx = 2;
                    AH.saved_K = K;
                end
                if size(FRS,1) == 1
                    FRS = FRS';
                end

                FRS = FRS{indices(1),indices(2)};

                %get waypoint data in local frame to provide for SDF cost
                %function computation

                waypoint = world_to_local(agent_state_mex(1:3), x_des_mex);
                
                % Find the maximum time interval for the FRS
                dstack = zeros(length(FRS.vehRS_save),1);
                dstack2 = zeros(length(FRS.vehRS_save),1);
                parfor i=1:length(FRS.vehRS_save)
                    Zi = interval(FRS.vehRS_save{1,i}) %generate interval representation of zonotope
                    dstack(i,1) = infimum(Zi(20)); %20th index corresponds to the interval representation for the time center and generators
                    dstack2(i,1) = supremum(Zi(20));
                end

                interval_size = dstack2 - dstack;
                dt = max(interval_size);
%% ######################## OLD VERTICES METHOD FOR QINGYI
%                 %loop over FRS's to output the corresponding obstacle
%                 %vertices in local frame over each time step of the FRS
%                 obs_vertices = cell(2,length(dyn_obs_local));
%                 for jj = 1:length(dyn_obs_local)
%                     cx = dyn_obs_local(1,jj);
%                     cy = dyn_obs_local(2,jj);
%                     h = dyn_obs_local(3,jj);
% %                     v = dyn_obs_local(4,jj); %using actual velocity
%                     v = 25; %max obstacle velocity    
%                     l = dyn_obs_local(5,jj);
%                     w = dyn_obs_local(6,jj);
%                     rot_h = [cos(h) -sin(h);
%                              sin(h) cos(h)];
%                     rot_h_mirror = [cos(-h) -sin(-h);
%                                     sin(-h) cos(-h)];
% 
%                     %iterate over FRS to get the vertices for the jj-th obstacle
%                     %at each time for each of the zonotopes in the FRS
%                     parfor i=1:length(FRS.vehRS_save)
%                         ti = FRS.vehRS_save{1,i}.Z(20,1);
%                         cdisp = [cx; cy] + rot_h*[v*ti;0];         
% %                         dt = FRS.vehRS_save{1, i}.Z(20,(FRS.vehRS_save{1, i}.Z(20,:)~=0));
% %                         dt = dt(end);
%                         vertices{1,i} = [cdisp(1),cdisp(2)]+(rot_h*[(l/2+v*dt)*[1;1;-1;-1],(w/2)*[1;-1;-1;1]]')'; 
%                         mirr_vertices{1,i} = [cdisp(1),-cdisp(2)]+(rot_h_mirror*[(l/2+v*dt)*[1;1;-1;-1],(w/2)*[1;-1;-1;1]]')'; %flip heading and y position for mirroring 
%                     end
%                     obs_vertices{1,jj} = vertices;
%                     obs_vertices{2,jj} = mirr_vertices;
%                 end



%%
                AH.plot_selected_parameter_FRS(k,type_manu,FRS,mirror_flag,agent_state,multiplier);

                if isempty(AH.A.time)
                    global_time = 0;
                else
                    global_time = AH.A.time(end);
                end
                
                %log stuff for Jon
%                 AH.SDF_trial_FRSdata.obs_info = dyn_obs_mex;
                AH.SDF_trial_FRSdata.agent_state = agent_state_mex;
                AH.SDF_trial_FRSdata.uidx0 = idxu0;
                AH.SDF_trial_FRSdata.man_type = type_text;
                AH.SDF_trial_FRSdata.p_idxes = indices;
                AH.SDF_trial_FRSdata.obs_zono = obs_zono;
                AH.SDF_trial_FRSdata.obs_zono_mirr = obs_zono_mirr;
                AH.SDF_trial_FRSdata.max_vel = v;
%                 AH.SDF_trial_FRSdata.max_FRStime_interval = dt;
                AH.SDF_trial_FRSdata.waypoint = waypoint;
%                 AH.SDF_trial_FRSdata.global_time = global_time;

%                 AH.SDF_trial_FRSdata.


                AH.waypt_hist = [AH.waypt_hist x_des];
                AH.K_hist = [AH.K_hist k];
                AH.FRS_hist{end+1} = FRS;
                AH.mirror_hist = [AH.mirror_hist mirror_flag];
                AH.type_manu_hist = [AH.type_manu_hist type_manu];
                AH.state_hist = [AH.state_hist agent_state];
                AH.time_hist = [AH.time_hist AH.A.time(end)];
            end
            tout = 0; % place holder
        end
        
        function plot_selected_parameter_FRS(AH,K,type_manu,FRS,mirror_flag,agent_state,multiplier)
            if ~isempty(K)
                %clear data and then plot
                AH.FRS_helper_handle.XData = cell(3,1);
                AH.FRS_helper_handle.YData = cell(3,1);
                if type_manu == 1 % 1: speed change. 2: direction change. 3: lane change
                    AH.plot_zono_collide_sliced(FRS,mirror_flag,agent_state,[K; 0],[0 0 1],2);
                else
                    AH.plot_zono_collide_sliced(FRS,mirror_flag,agent_state,[agent_state(4);K *multiplier],[0 1 0],2);
                end
            end
        end


        
        function [T, U, Z]=gen_ref(AH, K, reference_flag,agent_state, ref_time)
            % generate reference based on parameter and states
            if ~exist('agent_state','var')
                agent_info = AH.get_agent_info();
                agent_state = agent_info.state(:,end);
            end
            if ~exist('ref_time','var')
                ref_time = AH.A.time(end);
            end
            if ~exist('reference_flag','var')
                reference_flag = 1;
            end
            u_cur = agent_state(4) ;
            y_cur = agent_state(2) ;
            x_cur = agent_state(1) ;
            Au = K(1);
            Ay = K(2);
            t0_idx = K(3);
            
            t0 = (t0_idx-1)*AH.t_move;
            type_manu = K(4);
            if type_manu == 3 % 1: speed change. 2: direction change. 3: lane change
                [T, U,Z] = gaussian_T_parameterized_traj_with_brake(t0,Ay,Au,u_cur,[],0,1);
            else
                [T, U,Z] = sin_one_hump_parameterized_traj_with_brake(t0,Ay,Au,u_cur,[],0,1);
            end
            
            if reference_flag
                AH.ref_Z=[AH.ref_Z;x_cur+Z(1,:);y_cur+Z(2,:)];% for plotting
                AH.t_real_start = [AH.t_real_start;ref_time];
            else
                AH.proposed_ref_Z=[AH.proposed_ref_Z;x_cur+Z(1,:);y_cur+Z(2,:)];% for plotting
                AH.t_proposed_start = [AH.t_proposed_start;ref_time];
            end
            
            
        end

        function reset(AH,flags,eps_seed)
            if ~exist('eps_seed','var')
                AH.A.reset();
            else
                rng(eps_seed)
                AH.A.reset();
            end
            AH.flags = flags;
            AH.ref_Z = [];
            AH.proposed_ref_Z = [];
            AH.t_real_start = [];
            AH.t_proposed_start = [];
            AH.K_hist = [];
            AH.waypt_hist = [];
            AH.FRS_hist = {};
            AH.mirror_hist = [];
            AH.state_hist = [];
            AH.type_manu_hist = [];
            AH.time_hist = [];
            if ~isempty(AH.HLP)
                AH.HLP.reset();
                if ~isempty(AH.HLP.plot_data.waypoints)
                    AH.HLP.plot_data.waypoints.XData = [];
                    AH.HLP.plot_data.waypoints.YData = [];
                end
            end
        end
        
        %% plot functions
        function plot(AH)
            hold_check = false ;
            if ~ishold
                hold_check = true ;
                hold on
            end
            if ~isempty(AH.planned_path)
               plot(AH.planned_path(1,:),AH.planned_path(2,:),'k-','LineWidth',1);
            end
            text(-250,15,"u="+num2str(AH.A.state(4,end))+"m/s",'Color','red','FontSize',15)
            
            if hold_check
                hold off ;
            end
            
        end
        function plot_zono_collide_sliced(AH,FRS,mirror_flag,agent_state,K,color,slice_level)
            if K(2) == 0
                slice_dim = 11;
                k_slice = K(1);
            else
                slice_dim = 12;
                k_slice = K(2);
            end

            
            for t_idx = 1:5:length(FRS.vehRS_save) 
                if slice_level == 0
                    zono_one = FRS.vehRS_save{t_idx};
                elseif slice_level == 1
                    zono_one = zonotope_slice(FRS.vehRS_save{t_idx}, [7;8;9], [agent_state(4);agent_state(5);agent_state(6)]);
                elseif slice_level == 2
                    zono_one = zonotope_slice(FRS.vehRS_save{t_idx}, [7;8;9;slice_dim], [agent_state(4);agent_state(5);agent_state(6);k_slice]);
                else
                    error('unknown slice_level in plot selected zono');
                end
                h = plot(zono_one,[1,2],'Color',color);
                if mirror_flag
                    h.YData = - h.YData;
                end
                XY = [h.XData(:) h.YData(:)];                                    
                theta = agent_state(3);
                R=[cos(theta) -sin(theta); sin(theta) cos(theta)];
                rotXY=XY*R'; %MULTIPLY VECTORS BY THE ROT MATRIX
                Xqr = reshape(rotXY(:,1), size(h.XData,1), []);
                Yqr = reshape(rotXY(:,2), size(h.YData,1), []);
                %SHIFTING
                h.XData = Xqr+agent_state(1);
                h.YData = Yqr+agent_state(2);
                
                AH.FRS_helper_handle.XData{slice_level+1} = [h.YData nan AH.FRS_helper_handle.XData{slice_level+1}];
                AH.FRS_helper_handle.YData{slice_level+1} = [h.XData nan AH.FRS_helper_handle.YData{slice_level+1}];
            end
            

        end

    end
end

%% helper function to generate obstacle structure for c++
function obj_mex = get_obs_mex(dyn_obs, bounds)
    all_pts = dyn_obs{1};
    all_vels = dyn_obs{2};
    obj_mex = [];
    n_obs = length(all_vels);
    for dyn_obs_idx = 1:n_obs
        dyn_obs_pts_start_idx = ((dyn_obs_idx-1) * 6) + 1;
        curr_pts = all_pts(:, ...
            dyn_obs_pts_start_idx:dyn_obs_pts_start_idx+3);
        deltas = max(curr_pts,[],2) - min(curr_pts,[],2);
        means = mean(curr_pts, 2);
        dyn_c_x = means(1);
        dyn_c_y = means(2);
        dyn_length = deltas(1);
        dyn_width = deltas(2);
        dyn_velocity = all_vels(dyn_obs_idx);
        dyn_heading_rad = 0;
        obj_mex(:,end+1) = [dyn_c_x; 
                            dyn_c_y; 
                            dyn_heading_rad;
                            dyn_velocity;
                            dyn_length;
                            dyn_width];
    end
    xlo = bounds(1) ; xhi = bounds(2) ;
    ylo = bounds(3) ; yhi = bounds(4) ;
    dx = xhi - xlo;
    dy = yhi - ylo;
    x_c = mean([xlo, xhi]);
    y_c = mean([ylo, yhi]);
    b_thick = 0.01;
    b_half_thick = b_thick / 2.0;

    % Top
    obj_mex(:,end+1) = [x_c; yhi+b_half_thick; 0; 0; dx; b_thick];
    % Bottom
    obj_mex(:,end+1) = [x_c; ylo-b_half_thick; 0; 0; dx; b_thick];
    % Right
    obj_mex(:,end+1) = [xhi+b_half_thick; y_c; 0; 0; b_thick; dy];
    % Left
    obj_mex(:,end+1) = [xlo-b_half_thick; y_c; 0; 0; b_thick; dy];
end
