function k_times = time_cons_eval(AH, agent_state_mex, x_des_mex, dyn_obs_mex,...
                        spd_FRS_ranges,lan_FRS_ranges,dir_FRS_ranges,...
                    FRS_types,bin_indices_j,obstacles_are_mirrored,num_k)

%     num_k = 100;
    num_bins = length(bin_indices_j);
    k_cell = cell(1,num_bins);
    num_parr = 13; %number of bins solved in parallel

    %generate k parameters to evaluate
    for ii = 1:num_bins
        if FRS_types(ii) == 0
            min_val = spd_FRS_ranges(bin_indices_j(ii)+1,1);
            max_val = spd_FRS_ranges(bin_indices_j(ii)+1,2);
            k = linspace(min_val,max_val,num_k);
            k_cell{1,ii} = k;

        elseif FRS_types(ii) == 1
            min_val = dir_FRS_ranges(bin_indices_j(ii)+1,1);
            max_val = dir_FRS_ranges(bin_indices_j(ii)+1,2);
            k = linspace(min_val,max_val,num_k);
            k_cell{1,ii} = k;

        elseif FRS_types(ii) == 2
            min_val = lan_FRS_ranges(bin_indices_j(ii)+1,1);
            max_val = lan_FRS_ranges(bin_indices_j(ii)+1,2);
            k = linspace(min_val,max_val,num_k);
            k_cell{1,ii} = k;

        end


    end
    
    k_times = struct;
    
    %actual k_values plugged in to constraint evaluation
    k_times.k_values = k_cell;

    % Total time needed to evaluate constraints and gradient
    k_times.cons_total_times = zeros(num_k,1);
    k_times.grad_total_times = zeros(num_k,1);

    % Mean times for each set of 13 bins ran in parallel
    k_times.cons_parr_times = cell(num_k,1);
    k_times.grad_parr_times = cell(num_k,1);

    %Cell to save raw timing data from each bin for each k value
    k_times.raw_times = cell(num_k,1);


    for i = 1:num_k
        k_in = zeros(length(bin_indices_j),1);
        parfor jj = 1:length(k_in)
            k_temp = k_cell{1,jj};
            k_in(jj,1) = k_temp(1,i);
        end

        %call REFINE MEX file that returns constraint evaluation times
        %2 x num_bins array. 1st row is contraint eval time, 2nd row is
        %gradient eval time
        times_i = RDF_MEX(agent_state_mex, x_des_mex, dyn_obs_mex,k_in);
        
        %find total number of sequential iterations it take to run
        %optimization over all the bins
        total_runs = ceil(num_bins/num_parr);
        cons_parr_times = zeros(total_runs,1);
        grad_parr_times = zeros(total_runs,1);

        %store mean evaluation times of each set of 13 bins ran in
        %parallel. If there are less than 13 bins take average of all the
        %available bins
        parfor jj = 1:total_runs
            if num_parr*jj > num_bins
                cons_parr_times(jj,1) = mean(times_i(1,num_parr*(jj-1)+1:end));
                grad_parr_times(jj,1) = mean(times_i(2,num_parr*(jj-1)+1:end));
            else
                cons_parr_times(jj,1) = mean(times_i(1,num_parr*(jj-1)+1:num_parr*jj));
                grad_parr_times(jj,1) = mean(times_i(2,num_parr*(jj-1)+1:num_parr*jj));
            end

        end

        % sum times for each collection of 13 bins ran in parallel to get total time
        %e.g. if 37 bins, this is collections of 13,13,11. Sum the mean
        %times of each collection to get overall time needed 

        k_times.cons_total_times(i) = sum(cons_parr_times);
        k_times.grad_total_times(i) = sum(grad_parr_times);
        k_times.cons_parr_times{i} = cons_parr_times;
        k_times.grad_parr_times{i} = grad_parr_times;    
        k_times.raw_times{i} = times_i;

    end



end