% This matlab script can be used to visualize and post-process both data
% gathered from the robot (through our python logger) and data gathered
% from the phase space motion capture system
addpath(genpath(pwd))

%% This section loads & parses the python logger data into a struct
standup_kp400 = "data\mq_telem_04_05_2022_15-01-20.csv";
balancestand_attemp1 = "data\mq_telem_04_05_2022_18-17-45.csv";
balancestand_attemp2 = "data\mq_telem_04_05_2022_18-34-10.csv";
balancestand_attemp3 = "data\mq_telem_04_05_2022_19-54-04.csv"; % leg broke
balancestand_attemp4 = "data\mq_telem_04_05_2022_22-10-42.csv"; % flailing leg, killed
balancestand_attemp5 = "data\mq_telem_04_05_2022_22-25-57.csv"; % flailing, no kill, estop (orientation check fail)
balancestand_attemp9 = "data\mq_telem_05_05_2022_00-16-00.csv";
state_est_debug1 = "data\mq_telem_05_05_2022_01-21-48.csv";
state_est_debug2 = "data\mq_telem_05_05_2022_02-15-42.csv";
pushup_attempt1 = "data\mq_telem_05_05_2022_19-07-41.csv";
pushup_attempt2 = "data\mq_telem_05_05_2022_19-39-58.csv";
pushup_attempt3 = "data\mq_telem_05_05_2022_20-24-59.csv";
pushup_attempt4 = "data\mq_telem_05_05_2022_21-40-46.csv";
pushup_attempt5 = "data\mq_telem_05_05_2022_22-04-05.csv"; % lateral axis link snapped! cartesian kp=500, kd=8
grf_tes1 = "data\mq_telem_06_05_2022_12-25-36.csv";
wbc_debug_1 = "data\/mq_telem_10_05_2022_12-22-38.csv"; % after fixing imu, didn't even get to wbc tho bc oscillation
wbc_debug_2 = "data\mq_telem_10_05_2022_14-39-08.csv"; %mq leg 2 shook itself off. Lots of oscillation in front 2 leg. Kp 400, Kd 3.25 fFF -20
standup_thermal_1 = "data\mq_telem_10_05_2022_16-43-01.csv"; % mq a23 has vented housing, got to stable standing with a little push
standup_thermal_2 = "data\mq_telem_10_05_2022_17-11-30.csv"; % added ff force in z to rear legs
wbc_debug_3 = "data/mq_telem_11_05_2022_15-57-12.csv"; % fan for cooling, poor tuning on standup went stright to wbc
wbc_debug_4 = "data/mq_telem_11_05_2022_16-19-20.csv"; % orientation: kp=10; kd=3 -- destabilized even faster
wbc_debug_5 = "data/mq_telem_11_05_2022_16-35-08.csv"; % orientation: kp=kd=0. Foot broke off, lots of oscillation
wbc_debug_7 = "data/mq_telem_12_05_2022_12-55-20.csv"; % femurs got stuck on hips and fuse blew
wbc_debug_8 = "data/mq_telem_12_05_2022_13-38-04.csv"; % front leg hop, intermittent comm
wbc_debug_10 = "data/mq_telem_12_05_2022_16-17-28.csv"; %demo for talia with gamepad
wbc_debug_11 = "data/mq_telem_13_05_2022_14-49-32.csv"; % tried changing body mass to 2.2kg and some gamepad
trot_attempt_1 = "data/mq_telem_16_05_2022_14-41-33.csv"; % trot attempt -- lots of lateral axis action, tore a leg off
bad_balancestand_2 = "data/mq_telem_16_05_2022_17-09-40.csv"; % robot keeps wanting to sit back in balancestand
bad_balancestand_3 = "data/mq_telem_17_05_2022_12-28-41.csv"; % leg 3 not contacting the ground
torsopos_debug_1 = "data/mq_telem_17_05_2022_12-44-35.csv"; % trying to find issue with leg 3
wbc_debug_12 = "data/mq_telem_17_05_2022_14-50-28.csv"; % after fixing actuator 33 rotor slip
wbc_debug_13 = "data/mq_telem_17_05_2022_16-15-23.csv"; % printing matrices
wbc_debug_14 = "data/mq_telem_17_05_2022_17-46-32.csv"; % removed gravity term from KF
trot_attempt_2 = "data/mq_telem_17_05_2022_18-26-53.csv"; % after KF fix
trot_attempt_3 = "data/mq_telem_17_05_2022_19-11-10.csv"; % constrained the tibial links. First real walk!

wbc_tuning_2 = "data/mq_telem_18_05_2022_13-45-54.csv"; % first run w gamepad after fixing large grf issue. Still oscillates Kp 30 Kd 0.3 
wbc_tuning_3 = "data/mq_telem_18_05_2022_14-15-38.csv"; % reduced Kd on WBC tasks. Less oscillation visually Kp 30 Kd 0.1
wbc_tuning_4 = "data/mq_telem_18_05_2022_14-33-14.csv"; % Kp 50 Kd 0.1
wbc_tuning_5 = "data/mq_telem_18_05_2022_16-28-47.csv"; % Fixed softstop issue in robot server. Kp 50 Kd 0.1
wbc_tuning_6 = "data/mq_telem_18_05_2022_16-36-23.csv"; % Kp 30 Kd 0.1 robot is very oscillatory and destabilizes
wbc_tuning_7 = "data/mq_telem_20_05_2022_15-04-47.csv";
kp_kd_test_1 = "data/mq_telem_20_05_2022_16-07-34.csv"; % log is messed up
kp_kd_test_2 = "data/mq_telem_20_05_2022_17-00-43.csv"; %
kp_kd_test_3 = "data/mq_telem_20_05_2022_17-44-06.csv"; 
kp_kd_test_4 = "data/mq_telem_20_05_2022_18-03-28.csv"; % Finally not useless, the robot stood with SM's damping lol. Kp and Kd for WBC tasks were 0, and no softstops in robot_server
kp_kd_test_5 = "data/mq_telem_23_05_2022_10-40-23.csv"; % Reduced Kp_kin to 0.5 No unstable oscillations
kp_kd_test_6 = "data/mq_telem_23_05_2022_11-40-07.csv"; % increased torso mass from 3.3 to 4.3 overall mass is ~10kg now. better response from KinWBC
kp_kd_test_7 = "data/mq_telem_23_05_2022_11-59-53.csv"; % increased torso mass to 4.6. Robot seemed stiffer with only KinWBC running (that's good)
kp_kd_test_8 = "data/mq_telem_23_05_2022_12-22-50.csv"; % set ini_pos y to 0, and ini_rpy r and p to 0 for ::onEnter in balancestand
kp_kd_test_9 = "data/mq_telem_23_05_2022_12-40-39.csv"; % decreased actuator Kp by half. Less bad oscillations, and robot response actually feels damped.
kp_kd_test_10 = "data/mq_telem_23_05_2022_12-58-07.csv"; % decreased actuator Kp by half again. No oscillation, response felt weaker and "squishy". Might need more accurate model for better feedforwards
trot_attempt_4 = "data/mq_telem_23_05_2022_13-31-08.csv"; % trotted semi-successfully? Wavered around in x-y plane bc ab/ads go medial in swing phase
trot_attempt_5 = "data/mq_telem_23_05_2022_17-14-34.csv"; % same software as before. Replaced spine with 40x20, appeared more stable wrt torsion abt spine
trot_attempt_6 = "data/mq_telem_23_05_2022_17-45-22.csv"; % increased z height for trot walk -- similar visual results (ignore, was overwritten later in code)
trot_attempt_7 = "data/mq_telem_23_05_2022_17-59-13.csv"; % actually increased the height for trot walk
trot_attempt_9 = "data/mq_telem_24_05_2022_14-12-55.csv"; % lower link broke -- trying to see side-to-side drift
trot_attempt_10 = "data/mq_telem_24_05_2022_15-05-59.csv"; % changed trot period in YAML but it didn't do anything. used gamepad to move bot
trot_attempt_11 = "data/mq_telem_25_05_2022_11-43-01" ; % Random Jump happened before we started anything, no clue why!
trot_attempt_12 = "data/mq_telem_25_05_2022_12-07-33" ; % Switched to cmpc gait walking2, also want to check the position values when the robot does not move 
trot_attempt_13 = "data/mq_telem_25_05_2022_12-26-58" ; % Logged data correctly this time hopefully!
trot_attempt_14 = "data/mq_telem_25_05_2022_13-17-11" ; %Checking if logging mpc_data worked!
trot_attempt_15 = "data/mq_telem_25_05_2022_13-31-38" ; % Changed Kp for position task to 130,70,100
trot_attempt_16 = "data/mq_telem_25_05_2022_14-23-13"; %bodypostask kp 0 50 100 kd 0 10 10
trot_attempt_9_5 = "data/mq_telem_24_05_2022_13-53-24" ; % Best run we had!
trot_attempt_17 = "data/mq_telem_25_05_2022_16-41-55" ; % Changed Kp foot to 750 from 500
trot_attempt_18 = "data/mq_telem_25_05_2022_16-52-39"; %Kp foot increased to 5000. No noticeable rolling
trot_attempt_19 = "data/mq_telem_26_05_2022_09-51-42"; %IMU noise to .04 from .02
trot_attempt_20 = "data/mq_telem_26_05_2022_10-15-06"; %IMU noise to .175 and .2
state_est_debug_1 = "data/mq_telem_26_05_2022_10-50-48"; %trust window from 0.2 to 0.1 No noticeable change
trot_12_1_1 = "data/mq_telem_01_06_2022_16-29-43.csv"; % 12:1 actuators on all x3's, rapid walk backwards when going to trot
trot_12_1_2 = "data/mq_telem_01_06_2022_17-09-25.csv"; % same as above, but did torso orientation demo to verify wbc without mpc
trot_12_1_4 = "data/mq_telem_01_06_2022_18-18-03.csv"; % same behavior observed as last test
wbc_oscillation_tuning_1 = "data/mq_telem_02_06_2022_10-38-18.csv"; % WBC standup/standup has lateral oscillations in the legs...
wbc_oscillation_tuning_2 = "data/mq_telem_02_06_2022_11-06-00.csv"; % disable wbc by setting gains to 0, only used kin_wbc
wbc_oscillation_tuning_3 = "data/mq_telem_02_06_2022_11-49-21.csv"; % kp_kin to 1 (was 0.5), kd pos/ori 5 (from 15), kp pos/ori 70 (from 100), locomotion trust window to 0.02 (from 0.0), cg back 7cm -- good results

high_contact_walk_2 = "data/mq_telem_03_06_2022_13-03-18.csv"; % used the walking gait which overlaps stance phase
robot_walk_new_actuator_1 = "data/mq_telem_03_06_2022_14-18-58.csv";  % walked but back legs sagged after a bit
logging_revision_test_1 = "data/mq_telem_06_06_2022_14-44-36.csv";

% dog pose for the first and last 30sec with locomotion in between to 
% check for IMU drift
imu_drift_test_1 = "data/mq_telem_06_06_2022_14-54-08.csv";
% after applying mounting angle changes on robot-server side
% robot freaked out during standup and terminal print for orientation
% safety check failure came up
imu_drift_test_2 = "data/mq_telem_07_06_2022_12-09-00.csv";
% reverted mounting angle change and the robot did not freak out, though
% gradual pitch/yaw drift still an issue
imu_drift_test_3 = "data/mq_telem_07_06_2022_13-06-49.csv";
% Set mounting-roll to 180, mounting-pitch to 90, and applied 180deg 
% rotation about roll in MIT data unpacking. Less drift for robot, still
% pitch back in locomotion state
imu_drift_test_4 = "data/mq_telem_07_06_2022_15-01-45.csv";
% orientation test in balance stand
orientation_test_1 = "data/mq_telem_08_06_2022_15-54-05.csv";
% we ran the robot on a yoga mat to see if having a slightly damped foor
% helps it or not. The torso lacks torsional rigidity still. Still pitched
% back but potentially more slowly?
imu_drift_test_5 = "data/mq_telem_13_06_2022_17-21-25.csv";
% Changed spine from 40x20 to 40x40. Increased mass by 107 grams
% Robot still drifts negative in pitch but perhaps less? a screw came out
% of a link, prematurely ending test
imu_drift_test_6 = "data/mq_telem_17_06_2022_17-31-58.csv";

manual_orientation_check = "data/mq_telem_28_06_2022_15-43-46.csv";

x_type_test_1 = "data/mq_telem_29_06_2022_16-49-05.csv";

sample_freq = 500; % Hz
T = readtable(x_type_test_1);
mq_time = (1:1:height(T))/sample_freq;
headers = T.Properties.VariableNames;
mq_telem = parse_mq_telem_table(T);

%% This section loads & parses the mocap data into a struct in body frame

% For readability, its recommended to use the associated python logger
% filename + "_mocap" so that its easy to know which files go together
test_datafile = "data/muadquad_setup_002.csv";
walking_test_1 = "data/muadquad_setup_006.csv";
high_contact_mocap_1 = "data/muadquad_setup_009.csv";
high_contact_mocap_2 = "data/muadquad_setup_010.csv";
dummy_test_1 = "data/muadquad_setup_013.csv";
dummy_test_2 = "data/muadquad_setup_014.csv";
high_contact_mocap_3 = "data/muadquad_setup_017.csv";
imu_drift_test_4_mocap = "data/muadquad_6_7_22_004.csv";
imu_drift_test_5_mocap = "data/muadquad_6_13_22_001.csv";
orientation_test_1_mocap = "data/muadquad_6_7_22_005.csv";
imu_drift_test_6_mocap = "data/muadquad_6_17_22_002.csv";
manual_orientation_check_mocap = "data/muadquad_6_28_22.csv";

mocap_T = readtable(manual_orientation_check_mocap);
mocap_data = parse_mocap(mocap_T);

%% mocap - plot of 3d cartesian location of robot over time
% green bubble -- start
% red bubble -- end
figure;
hold on;
plot3(mocap_data.RB_pos(1,20:end), mocap_data.RB_pos(2,20:end), mocap_data.RB_pos(3,20:end));
plot3(mocap_data.RB_pos(1,20), mocap_data.RB_pos(2,20), mocap_data.RB_pos(3,20), 'go');
plot3(mocap_data.RB_pos(1,end), mocap_data.RB_pos(2,end), mocap_data.RB_pos(3,end), 'ro');
daspect([1 1 1]);
xlabel("x [mm]")
ylabel("y [mm]")
zlabel("z [mm]")
view([-140 30])
grid on;
hold off;


%% python log - 3d foot pos in hip throughout timeframe set in <xlims>
% note: also plots GRF command and estimate in the same timeframe
xlims = [0, inf];
% xlims = []; % optional -- set limits and comment this line in
time_mask = mq_time > xlims(1) & mq_time < xlims(2);
figure;

subplot(1,2,1); hold on;
plot3(mq_telem.leg0_foot_pos(time_mask,1), mq_telem.leg0_foot_pos(time_mask,2), mq_telem.leg0_foot_pos(time_mask,3));
plot3(mq_telem.leg1_foot_pos(time_mask,1), mq_telem.leg1_foot_pos(time_mask,2), mq_telem.leg1_foot_pos(time_mask,3));
plot3(mq_telem.leg2_foot_pos(time_mask,1), mq_telem.leg2_foot_pos(time_mask,2), mq_telem.leg2_foot_pos(time_mask,3));
plot3(mq_telem.leg3_foot_pos(time_mask,1), mq_telem.leg3_foot_pos(time_mask,2), mq_telem.leg3_foot_pos(time_mask,3));
grid on
daspect([1 1 1])
xlabel("x [m] (fore-aft)")
ylabel("y [m] (latero-medial)")
zlabel("z [m] (vertical)")
view([-140 30])
title("3D foot positions")
hold off;

subplot(1,2,2); hold on;
plot3(mq_telem.leg0_grf_cmd(time_mask,1), mq_telem.leg0_grf_cmd(time_mask,2), mq_telem.leg0_grf_cmd(time_mask,3), 'DisplayName', 'leg0 cmd')
plot3(mq_telem.leg1_grf_cmd(time_mask,1), mq_telem.leg1_grf_cmd(time_mask,2), mq_telem.leg1_grf_cmd(time_mask,3), 'DisplayName', 'leg1 cmd')
plot3(mq_telem.leg2_grf_cmd(time_mask,1), mq_telem.leg2_grf_cmd(time_mask,2), mq_telem.leg2_grf_cmd(time_mask,3), 'DisplayName', 'leg2 cmd')
plot3(mq_telem.leg3_grf_cmd(time_mask,1), mq_telem.leg3_grf_cmd(time_mask,2), mq_telem.leg3_grf_cmd(time_mask,3), 'DisplayName', 'leg3 cmd')
plot3(mq_telem.leg0_grf_est(time_mask,1), mq_telem.leg0_grf_est(time_mask,2), mq_telem.leg0_grf_est(time_mask,3), 'DisplayName', 'leg0 est')
plot3(mq_telem.leg1_grf_est(time_mask,1), mq_telem.leg1_grf_est(time_mask,2), mq_telem.leg1_grf_est(time_mask,3), 'DisplayName', 'leg1 est')
plot3(mq_telem.leg2_grf_est(time_mask,1), mq_telem.leg2_grf_est(time_mask,2), mq_telem.leg2_grf_est(time_mask,3), 'DisplayName', 'leg2 est')
plot3(mq_telem.leg3_grf_est(time_mask,1), mq_telem.leg3_grf_est(time_mask,2), mq_telem.leg3_grf_est(time_mask,3), 'DisplayName', 'leg3 est')
grid on
daspect([1 1 1])
xlabel("x [N] (fore-aft)")
ylabel("y [N] (latero-medial)")
zlabel("z [N] (vertical)")
view([-140 30])
legend()
title("3D foot GRFs")
hold off;


%% python log - GRF in the z direction over timeframe set in <xlims>
xlims = [0, inf];
% xlims = [38, 39]; % optional -- set limits and comment this line in
figure;

subplot(2,1,1)
hold on
plot(mq_time, mq_telem.leg0_grf_cmd(:,3), 'r', 'DisplayName', 'leg0 cmd')
plot(mq_time, mq_telem.leg0_grf_est(:,3), 'r.', 'DisplayName', 'leg0 est')
plot(mq_time, mq_telem.leg1_grf_cmd(:,3), 'b', 'DisplayName', 'leg1 cmd')
plot(mq_time, mq_telem.leg1_grf_est(:,3), 'b.', 'DisplayName', 'leg1 est')
plot(mq_time, mq_telem.leg2_grf_cmd(:,3), 'k', 'DisplayName', 'leg2 cmd')
plot(mq_time, mq_telem.leg2_grf_est(:,3), 'k.', 'DisplayName', 'leg2 est')
plot(mq_time, mq_telem.leg3_grf_cmd(:,3), 'm', 'DisplayName', 'leg3 cmd')
plot(mq_time, mq_telem.leg3_grf_est(:,3), 'm.', 'DisplayName', 'leg3 est')
xlim(xlims)
ylabel("GRF z [N]")
legend()
hold off

subplot(2,1,2)
hold on
plot(mq_time, mq_telem.leg0_grf_cmd(:,3)...
    +mq_telem.leg1_grf_cmd(:,3)...
    +mq_telem.leg2_grf_cmd(:,3)...
    +mq_telem.leg3_grf_cmd(:,3), 'g', 'DisplayName', 'total cmd')
plot(mq_time, mq_telem.leg0_grf_est(:,3)...
    +mq_telem.leg1_grf_est(:,3)...
    +mq_telem.leg2_grf_est(:,3)...
    +mq_telem.leg3_grf_est(:,3), 'g.', 'DisplayName', 'total est')
xlim(xlims)
ylabel("GRF z [N]")
legend()
hold off

%% python log - torso cartesian pos plotting. Choice of x, y, z
figure; hold on;
plot_xyz = [true false false];

if (sum(plot_xyz) == 3)
    % plot x, y, and z
    subplot(1,3,1)
    plot(mq_time, mq_telem.torso_pos(:,1));
    xlabel('Time (s)')
    ylabel('State Estimated X (m)')
    subplot(1,3,2)
    plot(mq_time, mq_telem.torso_pos(:,2));
    xlabel('Time (s)')
    ylabel('State Estimated Y (m)')
    subplot(1,3,3)
    plot(mq_time, mq_telem.torso_pos(:,3));
    xlabel('Time (s)')
    ylabel('State Estimated Z (m)')
elseif (sum(plot_xyz) == 2)
    % plot the two axes selected
    % TODO: not implemented yet
    title("Not implemented yet :(")
elseif (sum(plot_xyz) == 1)
    % plot the selected axis
    plot(mq_time, mq_telem.torso_pos(:,find(plot_xyz)));
    xlabel('Time (s)')
    if(find(plot_xyz) == 1)
        ylabel('State Estimated X (m)')
    elseif(find(plot_xyz) == 2)
        ylabel('State Estimated Y (m)')
    else
        ylabel('State Estimated Z (m)')
    end
end
hold off;


%% python log & mocap optional - torso rpy plotting
plot_mocap = true;
mq_telem_time_mark = 51.008;
mocap_time_mark = 13.5792;
plot_low_pass_filtered = false;
lims = [0, inf];
% lims = [35, 70]; % optional -- set limits and comment this line in

figure; hold on;
mocap_offset_back = mocap_time_mark - mq_telem_time_mark;
[b,a] = butter(2,0.002,'low');

plot(mq_time, (180/pi)*mq_telem.torso_rpy(:,1), 'DisplayName',"IMU roll");
plot(mq_time, (180/pi)*mq_telem.torso_rpy(:,2), 'DisplayName',"IMU pitch");
% plot(mq_time, (180/pi)*mq_telem.torso_rpy(:,3), 'DisplayName',"IMU yaw");
if plot_low_pass_filtered
    plot(mq_time, (180/pi)*filter(b,a,mq_telem.torso_rpy(:,1)), 'DisplayName',"filtered IMU roll");
    plot(mq_time, (180/pi)*filter(b,a,mq_telem.torso_rpy(:,2)), 'DisplayName',"filtered IMU pitch");
    % plot(mq_time, (180/pi)*filter(b,a,mq_telem.torso_rpy(:,3)), 'DisplayName',"filtered IMU yaw");
end
if plot_mocap
    plot(mocap_data.time - mocap_offset_back, (180/pi)*mocap_data.RB_ypr(:,3), 'DisplayName',"MOCAP roll");
    plot(mocap_data.time - mocap_offset_back, (180/pi)*mocap_data.RB_ypr(:,2), 'DisplayName',"MOCAP pitch");
    % plot(mocap_data.time - mocap_offset_back, (180/pi)*mocap_data.RB_ypr(:,1), 'DisplayName',"MOCAP yaw");
    if plot_low_pass_filtered
        plot(mocap_data.time - mocap_offset_back, (180/pi)*filter(b,a,mocap_data.RB_ypr(:,3)), 'DisplayName',"filtered MOCAP roll");
        plot(mocap_data.time - mocap_offset_back, (180/pi)*filter(b,a,mocap_data.RB_ypr(:,2)), 'DisplayName',"filtered MOCAP pitch");
        % plot(mocap_data.time - mocap_offset_back, (180/pi)*filter(b,a,mocap_data.RB_ypr(:,1)), 'DisplayName',"filtered MOCAP yaw");
    end
end

legend()
xlabel("time [s]")
ylabel("angle [deg]")
xlim(lims)
if plot_mocap
    title("State Estimate vs Mocap")
else
    title("State Estimate")
end
hold off;


%% python log - robot torso angular rate
lims = [0, inf];
% lims = [35, 70]; % optional -- set limits and comment this line in
figure; hold on;
plot(mq_time, (180/pi)*mq_telem.torso_omega(:,1), 'DisplayName',"roll rate");
plot(mq_time, (180/pi)*mq_telem.torso_omega(:,2), 'DisplayName',"pitch rate");
plot(mq_time, (180/pi)*mq_telem.torso_omega(:,3), 'DisplayName',"yaw rate");
legend()
xlim(lims)
ylabel("Angular rate (degrees per second)")
xlabel("Time (s)")
title("Robot Estimated Angular Rate")
hold off;

%% python log - Torso desired roll pitch yaw and associated velocities
% Note: This will ONLY be nonzero when the robot is using MPC

figure;
subplot(2,1,1)
hold on
plot(mq_time, mq_telem.torso_rpy_des(:,1)*(180/pi), 'DisplayName',"des roll");
plot(mq_time, mq_telem.torso_rpy_des(:,2)*(180/pi), 'DisplayName',"des pitch");
plot(mq_time, mq_telem.torso_rpy_des(:,3)*(180/pi), 'DisplayName',"des yaw");
legend()
title("Body rpy des")
hold off;
subplot(2,1,2)
hold on
plot(mq_time, mq_telem.torso_v_ori_des(:,1)*(180/pi), 'DisplayName',"des roll rate");
plot(mq_time, mq_telem.torso_v_ori_des(:,2)*(180/pi), 'DisplayName',"des pitch rate");
plot(mq_time, mq_telem.torso_v_ori_des(:,3)*(180/pi), 'DisplayName',"des yaw rate");
legend()
title("Body Desired orientation velocity")
hold off;


%% python log - Torso position and velocity
xlims = [0, inf];
% xlims = [35, 70];

figure;
subplot(2,1,1)
hold on
plot(mq_time, mq_telem.torso_pos(:,1), 'DisplayName',"x pos");
plot(mq_time, mq_telem.torso_pos(:,2), 'DisplayName',"y pos");
plot(mq_time, mq_telem.torso_pos(:,3), 'DisplayName',"z pos");
legend()
xlim(lims)
title("robot estimated body position")
hold off;
subplot(2,1,2)
hold on
plot(mq_time, mq_telem.torso_vel(:,1), 'DisplayName',"x vel");
plot(mq_time, mq_telem.torso_vel(:,2), 'DisplayName',"y vel");
plot(mq_time, mq_telem.torso_vel(:,3), 'DisplayName',"z vel");
legend()
xlim(lims)
title("robot estimated body velocity")
hold off;


%% Torso position Desired

figure;
hold on
plot(mq_time, mq_telem.torso_pos_des(:,1)*100, 'DisplayName',"x");
plot(mq_time, mq_telem.torso_pos_des(:,2)*100, 'DisplayName',"y");
plot(mq_time, mq_telem.torso_pos_des(:,3)*100, 'DisplayName',"z");
legend()
title("body position desired")
hold off;

%% Torso position Error

figure;
hold on
plot(mq_time, (mq_telem.torso_pos_des(:,1)-mq_telem.torso_pos(:,1))*100, 'DisplayName',"x err");
plot(mq_time, (mq_telem.torso_pos_des(:,2)-mq_telem.torso_pos(:,2))*100, 'DisplayName',"y err");
plot(mq_time, (mq_telem.torso_pos_des(:,3)-mq_telem.torso_pos(:,3))*100, 'DisplayName',"z err");
legend()
% xlim([125, 126.7])
title("body position error")
hold off;

%% Feet Contact State
figure;
hold on
plot(mq_time, mq_telem.contact_state(:,1), 'DisplayName',"Contact State Foot 0");
plot(mq_time, mq_telem.contact_state(:,2), 'DisplayName',"Contact State Foot 1");
plot(mq_time, mq_telem.contact_state(:,3), 'DisplayName',"Contact State Foot 2");
plot(mq_time, mq_telem.contact_state(:,4), 'DisplayName',"Contact State Foot 3");
legend()
title("Feet Contact State")
hold off;

%% Torso 2D position

time_mask = mq_time > 125 & mq_time < 126.7;
time_mask = mq_time > 0;
time_indices = find(time_mask);
plot_mocap = true;

figure;
hold on
plot(-mq_telem.torso_pos(time_mask,2), mq_telem.torso_pos(time_mask,1));
plot(-mq_telem.torso_pos_des(time_mask,2), mq_telem.torso_pos_des(time_mask,1), 'm.')
plot(-mq_telem.torso_pos((time_indices(1)),2), mq_telem.torso_pos((time_indices(1)),1), "k*");
plot(-mq_telem.torso_pos((time_indices(end)),2), mq_telem.torso_pos((time_indices(end)),1), "r*");
if plot_mocap
    plot(-mocap_data.RB_pos(2,:)/1000, mocap_data.RB_pos(1,:)/1000, 'g.');
end
hold off
daspect([1 1 1])
title("x-y torso location")

%% Actuator X2 q's plotting

figure(3)
plot(mq_time, mq_telem.leg0_q_data(:,2) * 180/pi, 'r', 'DisplayName', 'leg0 femur q')
hold on
plot(mq_time, mq_telem.leg1_q_data(:,2) * 180/pi, 'g', 'DisplayName', 'leg1 femur q')
plot(mq_time, mq_telem.leg2_q_data(:,2) * 180/pi, 'b', 'DisplayName', 'leg2 femur q')
plot(mq_time, mq_telem.leg3_q_data(:,2) * 180/pi, 'k', 'DisplayName', 'leg3 femur q')
legend

%% actuator position tracking

figure;

lims = [57, 57.5];
% lims = [0, inf];
actuator_num = 3;

plot_time = mq_time;
time_mask = plot_time > 0;

periodize = false;
if periodize
    time_lims = [138.4, 142];
    cmd_diff = diff(mq_telem.leg0_tau_ff(:,3));
    cmd_diff = [cmd_diff; 0];
    start_mask = cmd_diff > 4;
    
    start_indices = find(start_mask);
    periodized_time = plot_time;
    for start_vec_idx = 1:length(start_indices)-1
        cycle_start = (start_indices(start_vec_idx));
        cycle_end = (start_indices(start_vec_idx+1)) - 1;
        cycle_start_time = plot_time(cycle_start);
        periodized_time(cycle_start:cycle_end) =...
            plot_time(cycle_start:cycle_end) - cycle_start_time;
    end
    plot_time = periodized_time;
    time_mask = (mq_time > time_lims(1)) & (mq_time < time_lims(2));
    gap_nans_mask = (diff([plot_time, 0]) < 0);
    plot_time(gap_nans_mask) = nan;
end

subplot(5,1,1)
hold on
plot(plot_time(time_mask), mq_telem.leg0_q_cmd(time_mask,actuator_num) * 180/pi, 'r', 'DisplayName',"mq0 x" + actuator_num + " cmd")
plot(plot_time(time_mask), mq_telem.leg0_q_data(time_mask,actuator_num) * 180/pi, 'r.', 'DisplayName',"mq0 x" + actuator_num + " data")
plot(plot_time(time_mask), mq_telem.leg1_q_cmd(time_mask,actuator_num) * 180/pi, 'b', 'DisplayName',"mq1 x" + actuator_num + " cmd")
plot(plot_time(time_mask), mq_telem.leg1_q_data(time_mask,actuator_num) * 180/pi, 'b.', 'DisplayName',"mq1 x" + actuator_num + " data")
plot(plot_time(time_mask), mq_telem.leg2_q_cmd(time_mask,actuator_num) * 180/pi, 'k', 'DisplayName',"mq2 x" + actuator_num + " cmd")
plot(plot_time(time_mask), mq_telem.leg2_q_data(time_mask,actuator_num) * 180/pi, 'k.', 'DisplayName',"mq2 x" + actuator_num + " data")
plot(plot_time(time_mask), mq_telem.leg3_q_cmd(time_mask,actuator_num) * 180/pi, 'm', 'DisplayName',"mq3 x" + actuator_num + " cmd")
plot(plot_time(time_mask), mq_telem.leg3_q_data(time_mask,actuator_num) * 180/pi, 'm.', 'DisplayName',"mq3 x" + actuator_num + " data")
title("position q")
legend("Location","best")
if ~periodize
xlim(lims)
end
hold off

subplot(5,1,2)
hold on
plot(plot_time(time_mask), mq_telem.leg0_qd_cmd(time_mask,actuator_num) * 180/pi, 'r', 'DisplayName',"mq0 x" + actuator_num + " cmd")
plot(plot_time(time_mask), mq_telem.leg0_qd_data(time_mask,actuator_num) * 180/pi, 'r.', 'DisplayName',"mq0 x" + actuator_num + " data")
plot(plot_time(time_mask(1:end-1)), diff(mq_telem.leg0_qd_data(time_mask,actuator_num) * 180/pi), 'b--')
plot(plot_time(time_mask), mq_telem.leg1_qd_cmd(time_mask,actuator_num) * 180/pi, 'b', 'DisplayName',"mq1 x" + actuator_num + " cmd")
plot(plot_time(time_mask), mq_telem.leg1_qd_data(time_mask,actuator_num) * 180/pi, 'b.', 'DisplayName',"mq1 x" + actuator_num + " data")
plot(plot_time(time_mask), mq_telem.leg2_qd_cmd(time_mask,actuator_num) * 180/pi, 'k', 'DisplayName',"mq2 x" + actuator_num + " cmd")
plot(plot_time(time_mask), mq_telem.leg2_qd_data(time_mask,actuator_num) * 180/pi, 'k.', 'DisplayName',"mq2 x" + actuator_num + " data")
plot(plot_time(time_mask), mq_telem.leg3_qd_cmd(time_mask,actuator_num) * 180/pi, 'm', 'DisplayName',"mq3 x" + actuator_num + " cmd")
plot(plot_time(time_mask), mq_telem.leg3_qd_data(time_mask,actuator_num) * 180/pi, 'm.', 'DisplayName',"mq3 x" + actuator_num + " data")
title("velocity qd")
legend()

if ~periodize
xlim(lims)
end
hold off

subplot(5,1,3)
hold on
title("torques")
cutoff  = -3;
cutoff2 = -20; 
plot(plot_time(time_mask), mq_telem.leg0_tau_ff(time_mask,actuator_num), 'r-', 'DisplayName', 'leg0 cmd')
plot(plot_time(time_mask), mq_telem.leg0_tau_est(time_mask,actuator_num), 'r.', 'DisplayName', 'leg0 est')
plot(plot_time(time_mask), cutoff*ones(size(plot_time(time_mask))), 'b--', 'DisplayName', 'contact\_cutoff')
plot(plot_time(time_mask), mq_telem.leg1_tau_ff(time_mask,actuator_num), 'b-', 'DisplayName', 'leg1 cmd')
plot(plot_time(time_mask), mq_telem.leg1_tau_est(time_mask,actuator_num), 'b.', 'DisplayName', 'leg1 est')
plot(plot_time(time_mask), mq_telem.leg2_tau_ff(time_mask,actuator_num), 'k-', 'DisplayName', 'leg2 cmd')
plot(plot_time(time_mask), mq_telem.leg2_tau_est(time_mask,actuator_num), 'k.', 'DisplayName', 'leg2 est')
plot(plot_time(time_mask), mq_telem.leg3_tau_ff(time_mask,actuator_num), 'm-', 'DisplayName', 'leg3 cmd')
plot(plot_time(time_mask), mq_telem.leg3_tau_est(time_mask,actuator_num), 'm.', 'DisplayName', 'leg3 est')
if ~periodize
    xlim(lims)
end
hold off

subplot(5,1,4)
hold on
plot(plot_time(time_mask), mq_telem.leg0_grf_cmd(time_mask,3), 'r', 'DisplayName', 'leg0 cmd')
plot(plot_time(time_mask), mq_telem.leg0_grf_est(time_mask,3), 'r.', 'DisplayName', 'leg0 est')
plot(plot_time(time_mask), cutoff2*ones(size(plot_time(time_mask))), 'b--', 'DisplayName', 'contact\_cutoff')
plot(plot_time(time_mask), mq_telem.leg1_grf_cmd(time_mask,3), 'b', 'DisplayName', 'leg1 cmd')
plot(plot_time(time_mask), mq_telem.leg1_grf_est(time_mask,3), 'b.', 'DisplayName', 'leg1 est')
plot(plot_time(time_mask), mq_telem.leg2_grf_cmd(time_mask,3), 'k', 'DisplayName', 'leg2 cmd')
plot(plot_time(time_mask), mq_telem.leg2_grf_est(time_mask,3), 'k.', 'DisplayName', 'leg2 est')
plot(plot_time(time_mask), mq_telem.leg3_grf_cmd(time_mask,3), 'm', 'DisplayName', 'leg3 cmd')
plot(plot_time(time_mask), mq_telem.leg3_grf_est(time_mask,3), 'm.', 'DisplayName', 'leg3 est')
legend()
title("GRF (N)")
if ~periodize
    xlim(lims);
end
hold off


% Temporary removal because I need to find best contact estimate method
% subplot(5,1,4)
% hold on
% plot(mq_time, (180/pi)*mq_telem.torso_omega(:,1), 'DisplayName',"roll");
% plot(mq_time, (180/pi)*mq_telem.torso_omega(:,2), 'DisplayName',"pitch");
% plot(mq_time, (180/pi)*mq_telem.torso_omega(:,3), 'DisplayName',"yaw");
% legend()
% title("angle rate")
% if ~periodize
% xlim(lims)
% end
% hold off;

subplot(5,1,5)
hold on
plot(plot_time(time_mask), mq_telem.contact_state(time_mask,1), 'DisplayName',"Contact State Foot 0");
plot(plot_time(time_mask), mq_telem.contact_state(time_mask,2), 'DisplayName',"Contact State Foot 1");
plot(plot_time(time_mask), mq_telem.contact_state(time_mask,3), 'DisplayName',"Contact State Foot 2");
plot(plot_time(time_mask), mq_telem.contact_state(time_mask,4), 'DisplayName',"Contact State Foot 3");
plot(plot_time(time_mask), (mq_telem.leg0_tau_est(time_mask,actuator_num) < cutoff), 'k--', 'DisplayName', 'contact\_state')
plot(plot_time(time_mask), (mq_telem.leg0_grf_est(time_mask,3) < cutoff2), 'm:', 'DisplayName', 'contact\_state')
legend()
title("Feet Contact State")
if ~periodize
    xlim(lims)
    ylim([-0.1, 1.1])
end
hold off;

% Temporary removal because I need to find best contact estimate method
% subplot(5,1,5)
% hold on
% plot(plot_time(time_mask), mq_telem.torso_vel(time_mask,1), 'DisplayName',"x");
% plot(plot_time(time_mask), mq_telem.torso_vel(time_mask,2), 'DisplayName',"y");
% plot(plot_time(time_mask), mq_telem.torso_vel(time_mask,3), 'DisplayName',"z");
% legend()
% title("state estimate velocities")
% if ~periodize
% xlim(lims)
% end
% hold off;


%% Actuator Kp and Kd plotting

figure;
lims = [0,inf];
actuator_num = 3;

subplot(4,1,1)
hold on

plot(mq_time, mq_telem.leg0_kp_joint(:,actuator_num), 'r.', 'DisplayName',"leg 0 x" + actuator_num + " kp")
plot(mq_time, mq_telem.leg1_kp_joint(:,actuator_num), 'bo', 'DisplayName',"leg 1 x" + actuator_num + " kp")
plot(mq_time, mq_telem.leg2_kp_joint(:,actuator_num), 'k*', 'DisplayName',"leg 2 x" + actuator_num + " kp")
plot(mq_time, mq_telem.leg3_kp_joint(:,actuator_num), 'mx', 'DisplayName',"leg 3 x" + actuator_num + " kp")
title("Controller commanded Kp joint")
legend("Location","best")
xlim(lims)
hold off

subplot(4,1,2)
hold on

plot(mq_time, mq_telem.leg0_kd_joint(:,actuator_num), 'r.', 'DisplayName',"leg 0 x" + actuator_num + " kd")
plot(mq_time, mq_telem.leg1_kd_joint(:,actuator_num), 'bo', 'DisplayName',"leg 1 x" + actuator_num + " kd")
plot(mq_time, mq_telem.leg2_kd_joint(:,actuator_num), 'k*', 'DisplayName',"leg 2 x" + actuator_num + " kd")
plot(mq_time, mq_telem.leg3_kd_joint(:,actuator_num), 'mx', 'DisplayName',"leg 3 x" + actuator_num + " kd")
title("Controller commanded Kd joint")
legend("Location","best")
xlim(lims)
hold off

subplot(4,1,3)
hold on

plot(mq_time, mq_telem.leg0_kp_cartesian(:,actuator_num), 'r.', 'DisplayName',"leg 0 x" + actuator_num + " kp")
plot(mq_time, mq_telem.leg1_kp_cartesian(:,actuator_num), 'bo', 'DisplayName',"leg 1 x" + actuator_num + " kp")
plot(mq_time, mq_telem.leg2_kp_cartesian(:,actuator_num), 'k*', 'DisplayName',"leg 2 x" + actuator_num + " kp")
plot(mq_time, mq_telem.leg3_kp_cartesian(:,actuator_num), 'mx', 'DisplayName',"leg 3 x" + actuator_num + " kp")
title("Controller commanded Kp cartesian")
legend("Location","best")
xlim(lims)
hold off

subplot(4,1,4)
hold on

plot(mq_time, mq_telem.leg0_kd_cartesian(:,actuator_num), 'r.', 'DisplayName',"leg 0 x" + actuator_num + " kd")
plot(mq_time, mq_telem.leg1_kd_cartesian(:,actuator_num), 'bo', 'DisplayName',"leg 1 x" + actuator_num + " kd")
plot(mq_time, mq_telem.leg2_kd_cartesian(:,actuator_num), 'k*', 'DisplayName',"leg 2 x" + actuator_num + " kd")
plot(mq_time, mq_telem.leg3_kd_cartesian(:,actuator_num), 'mx', 'DisplayName',"leg 3 x" + actuator_num + " kd")
title("Controller commanded Kd cartesian")
legend("Location","best")
xlim(lims)
hold off

%% write quaternion data

quaternions = [T.quat_0,T.quat_1,T.quat_2,T.quat_3, T.quat_0_1,T.quat_1_1,T.quat_2_1,T.quat_3_1];
quat_T = array2table(quaternions);
quat_T.Properties.VariableNames(1:8) = ["mit w","mit x","mit y","mit z","p3h w","p3h x","p3h y","p3h z"];
% writetable(quat_T,'quaternion_test.csv');

%% quaternion comparsion

mit_q = quaternions(:,1:4);
p3h_q = quaternions(:,5:8);

mit_q = quaternion(mit_q);
p3h_q = quaternion(p3h_q);
q180r = quaternion(rotm2quat([1 0 0; 0 -1 0; 0 0 -1]));


q_p3h_init = p3h_q(1);
q_mit_init = mit_q(1);
qtransform = q_mit_init*conj(q_p3h_init);


initial_p3h_yaw = -p3h_rpy(1,1);

yaw_mat = [cos(initial_p3h_yaw) -sin(initial_p3h_yaw) 0;...
    sin(initial_p3h_yaw) cos(initial_p3h_yaw) 0;...
    0 0 1];

qyaw = quaternion(rotm2quat(yaw_mat));

p3h_180r_q = p3h_q;
p3h_180r_q = qtransform*p3h_180r_q;

p3h_180r_q = q180r * p3h_180r_q * conj(q180r);
mit_rpy = quat2eul(mit_q, "XYZ");
p3h_rpy = quat2eul(p3h_q, "XYZ");
p3h_rpy_180r = quat2eul(p3h_180r_q, "XYZ");

figure;

subplot(3,1,1);
hold on;
plot((180/pi)*mit_rpy(:,1), 'DisplayName',"mit roll");
plot((180/pi)*mit_rpy(:,2), 'DisplayName',"mit pitch");
plot((180/pi)*mit_rpy(:,3), 'DisplayName',"mit yaw");
legend()
hold off;

subplot(3,1,2);
hold on;
plot((180/pi)*p3h_rpy(:,1), 'DisplayName',"p3h roll");
plot((180/pi)*p3h_rpy(:,2), 'DisplayName',"p3h pitch");
plot((180/pi)*p3h_rpy(:,3), 'DisplayName',"p3h yaw");
legend()
hold off;

subplot(3,1,3);
hold on;
plot((180/pi)*p3h_rpy_180r(:,1), 'DisplayName',"p3h\_180r roll");
plot((180/pi)*p3h_rpy_180r(:,2), 'DisplayName',"p3h\_180r pitch");
plot((180/pi)*p3h_rpy_180r(:,3), 'DisplayName',"p3h\_180r yaw");
legend()
hold off;


%%
% subplot(3,1,3);
figure
hold on;
subplot(3,1,1); hold on;
plot(mq_time, (180/pi)*mq_telem.torso_rpy(:,1), 'r-', 'DisplayName',"mit roll");
subplot(3,1,2); hold on;
plot(mq_time, (180/pi)*mq_telem.torso_rpy(:,2), 'm-',  'DisplayName',"mit pitch");
subplot(3,1,3); hold on;
plot(mq_time, (180/pi)*mq_telem.torso_rpy(:,3), 'b-',  'DisplayName',"mit yaw");

subplot(3,1,1); hold on;
plot(mq_time, (180/pi)*mit_rpy(:,1), 'r:',  'DisplayName',"mit q roll");
subplot(3,1,2); hold on;
plot(mq_time, (180/pi)*mit_rpy(:,2), 'm:', 'DisplayName',"mit q pitch");
subplot(3,1,3); hold on;
plot(mq_time, (180/pi)*mit_rpy(:,3), 'b:', 'DisplayName',"mit q yaw");

subplot(3,1,1); hold on;
plot(mq_time, (180/pi)*p3h_rpy_180r(:,1), 'r--', 'DisplayName',"p3h\_180r roll");
subplot(3,1,2); hold on;
plot(mq_time, (180/pi)*p3h_rpy_180r(:,2), 'm--', 'DisplayName',"p3h\_180r pitch");
subplot(3,1,3); hold on;
plot(mq_time, (180/pi)*p3h_rpy_180r(:,3), 'b--', 'DisplayName',"p3h\_180r yaw");
legend()
hold off;

%% Accelerometer Data Plotting

figure;

hold on
plot(mq_time, mq_telem.accelerometer(:,1), 'DisplayName',"IMU X acc")
plot(mq_time, mq_telem.accelerometer(:,2), 'DisplayName',"IMU Y acc")
plot(mq_time, mq_telem.accelerometer(:,3), 'DisplayName',"IMU Z acc")
plot(mq_time,  2*ones(size(mq_time)), 'r--', 'DisplayName',"validity thresh")
plot(mq_time, -2*ones(size(mq_time)), 'r--', 'DisplayName',"validity thresh")
legend()
ylabel('Gravity-compensated Acceleration (m/s/s)')
xlabel('Time (s)')
title("IMU Accelerometer Data")
hold off;

figure;
hold on;
f = 1000;
Fs = 500;
[b,a] = butter(2,.001,'low');
filtered_x_acc = filter(b,a,mq_telem.accelerometer(:,1));
filtered_y_acc = filter(b,a,mq_telem.accelerometer(:,2));
filtered_z_acc = filter(b,a,mq_telem.accelerometer(:,3));
plot(mq_time, filtered_x_acc, 'DisplayName',"Filtered IMU X acc")
plot(mq_time, filtered_y_acc, 'DisplayName',"Filtered IMU Y acc")
plot(mq_time, filtered_z_acc, 'DisplayName',"Filtered IMU Z acc")
plot(mq_time,  2*ones(size(mq_time)), 'r--', 'DisplayName',"validity thresh")
plot(mq_time, -2*ones(size(mq_time)), 'r--', 'DisplayName',"validity thresh")
legend()
ylabel('Gravity-compensated Acceleration (m/s/s)')
xlabel('Time (s)')
title("Filtered IMU Accelerometer Data")
hold off;

figure; hold on;
% y_axis = cumtrapz(mq_time, filtered_y_acc);
% plot(mq_time, y_axis, 'DisplayName',"Integged filtered IMU Y acc")
og_x_axis = cumtrapz(mq_time,cumtrapz(mq_time, mq_telem.accelerometer(:,1)));
og_y_axis = cumtrapz(mq_time,cumtrapz(mq_time, mq_telem.accelerometer(:,2)));
og_z_axis = cumtrapz(mq_time,cumtrapz(mq_time, mq_telem.accelerometer(:,3)));
plot(mq_time, og_x_axis, 'DisplayName',"Integged og IMU X acc")
plot(mq_time, og_y_axis, 'DisplayName',"Integged og IMU Y acc")
plot(mq_time, og_z_axis, 'DisplayName',"Integged og IMU Z acc")
legend()
ylabel('Double Integrated Acceleration (m)')
xlabel('Time (s)')
title("Double Integrated IMU Accelerometer Data")

hold off;

%% Gyro Data Plotting

figure; hold on
plot(mq_time, mq_telem.gyro(:,1) * 180 / pi, 'DisplayName',"IMU X w")
plot(mq_time, mq_telem.gyro(:,2) * 180 / pi, 'DisplayName',"IMU Y w")
plot(mq_time, mq_telem.gyro(:,3) * 180 / pi, 'DisplayName',"IMU Z w")
legend()
ylabel('Gyroscope Measured Rate (degree/s)')
xlabel('Time (s)')
title("IMU Gyroscope Data")
hold off;

% figure; hold on; 
% f = 1000;
% Fs = 500;
% [b,a] = butter(2,.002,'low');
% filtered_x_w = filter(b,a,mq_telem.gyro(:,1));
% filtered_y_w = filter(b,a,mq_telem.gyro(:,2));
% filtered_z_w = filter(b,a,mq_telem.gyro(:,3));
% plot(mq_time, filtered_x_w, 'DisplayName',"Filtered IMU X w")
% plot(mq_time, filtered_y_w, 'DisplayName',"Filtered IMU Y w")
% plot(mq_time, filtered_z_w, 'DisplayName',"Filtered IMU Z w")
% legend()
% title("Filtered IMU Gyroscope Data")
% hold off;

figure; hold on;
roll = cumtrapz(mq_time, filtered_x_w);
pitch = cumtrapz(mq_time, filtered_y_w);
yaw = cumtrapz(mq_time, filtered_z_w);
og_roll = cumtrapz(mq_time, mq_telem.gyro(:,1));
og_pitch = cumtrapz(mq_time, mq_telem.gyro(:,2));
og_yaw = cumtrapz(mq_time, mq_telem.gyro(:,3));
plot(mq_time, (180/pi)*roll, 'DisplayName',"Integged filtered IMU X w")
plot(mq_time, (180/pi)*pitch, 'DisplayName',"Integged filtered IMU Y w")
% plot(mq_time, (180/pi)*yaw, 'DisplayName',"Integged filtered IMU Z w")
plot(mq_time, (180/pi)*og_roll, 'DisplayName',"Integged og IMU X w")
plot(mq_time, (180/pi)*og_pitch, 'DisplayName',"Integged og IMU Y w")
% plot(mq_time, (180/pi)*og_yaw, 'DisplayName',"Integged og IMU Z w")
legend()
title("Integrated IMU Gyroscope Data")
hold off;
