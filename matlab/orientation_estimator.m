%% Simulator
% Estimator to replace the current orientation estimator in the MIT 
% controller (which is a pass through of IMU/ARS orientation)

% read in the log file of interest
log = "data/mq_telem_07_06_2022_15-01-45.csv";
T = readtable(log);
headers = T.Properties.VariableNames;
mq_telem = parse_mq_telem_table(T);

% create a time series (in seconds) from the log
sample_freq = 500; % Hz
mq_time = (1:1:height(T))/sample_freq;

% declare matrix to hold onto orientation estimates
orientation = nan(length(mq_time),4);
contact_estimates = nan(length(mq_time),4);

% create a notion of gait progress over the log (a value that MIT control 
% code normally has) so that we know where in a gait pattern we are
progress = populate_progress(mq_telem.contact_state);

% iterate over the entire log, and build up an orientation estimate of the 
% quadruped at each timestep
for t = 1:length(mq_time)
    
    % get the relevant data for timestep t from the parsed log
    data_t = get_mq_data(mq_telem, t);
    
    % get the contact estimate of the feet for timestep t
    contact_states = contact_estimator(data_t, progress(:,t), 10);
    
    % save contact_states data for later use
    contact_estimates(t,:) = contact_states;
    
    % now we run the orientation estimator
    orientation_est = get_orientation_estimate(data_t, contact_states);
    
    % if the result of the estimate was nans then we can assume we didnt
    % have enough legs and that we should trust the IMU/ARS fully
    if (sum(isnan(orientation_est)) > 0)
       orientation(t,:) = data_t.quat; 
    else
       orientation(t,:) = orientation_est;
    end
    
    if ~mod(t, 1000)
        fprintf("\ridx = %d / %d", t, length(mq_time));
    end
end

% plot the results
figure; hold on;

orientation_rpy = quat2eul(orientation);
plot(orientation_rpy(:,1)*180/pi);
plot(orientation_rpy(:,2)*180/pi);
plot(orientation_rpy(:,3)*180/pi);
