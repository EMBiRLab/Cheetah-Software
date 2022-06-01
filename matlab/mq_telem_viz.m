
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

trot_12_1_1 = "data/mq_telem_01_06_2022_16-29-43.csv"; % 12:1 actuators on all x3's, rapid walk backwards when going to trot
trot_12_1_2 = "data/mq_telem_01_06_2022_17-09-25.csv"; % same as above, but did torso orientation demo to verify wbc without mpc

sample_freq = 500; % Hz
T = readtable(trot_12_1_2);
mq_time = (1:1:height(T))/sample_freq;
headers = T.Properties.VariableNames;
mq_telem = parse_table(T);

%% foot pos 3d plotting

figure;
subplot(1,2,1)
hold on

plot3(mq_telem.leg0_foot_pos(:,1), mq_telem.leg0_foot_pos(:,2), mq_telem.leg0_foot_pos(:,3))
plot3(mq_telem.leg1_foot_pos(:,1), mq_telem.leg1_foot_pos(:,2), mq_telem.leg1_foot_pos(:,3))
plot3(mq_telem.leg2_foot_pos(:,1), mq_telem.leg2_foot_pos(:,2), mq_telem.leg2_foot_pos(:,3))
plot3(mq_telem.leg3_foot_pos(:,1), mq_telem.leg3_foot_pos(:,2), mq_telem.leg3_foot_pos(:,3))

grid on
daspect([1 1 1])
xlabel("x [m] (fore-aft)")
ylabel("y [m] (latero-medial)")
zlabel("z [m] (vertical)")
view([-140 30])
title("3D foot positions")

hold off;

% foot grf 3d plotting

subplot(1,2,2)
hold on

plot3(mq_telem.leg0_grf_cmd(:,1), mq_telem.leg0_grf_cmd(:,2), mq_telem.leg0_grf_cmd(:,3), 'DisplayName', 'leg0')
plot3(mq_telem.leg1_grf_cmd(:,1), mq_telem.leg1_grf_cmd(:,2), mq_telem.leg1_grf_cmd(:,3), 'DisplayName', 'leg1')
plot3(mq_telem.leg2_grf_cmd(:,1), mq_telem.leg2_grf_cmd(:,2), mq_telem.leg2_grf_cmd(:,3), 'DisplayName', 'leg2')
plot3(mq_telem.leg3_grf_cmd(:,1), mq_telem.leg3_grf_cmd(:,2), mq_telem.leg3_grf_cmd(:,3), 'DisplayName', 'leg3')

plot3(mq_telem.leg0_grf_est(:,1), mq_telem.leg0_grf_est(:,2), mq_telem.leg0_grf_est(:,3), 'DisplayName', 'leg0')
plot3(mq_telem.leg1_grf_est(:,1), mq_telem.leg1_grf_est(:,2), mq_telem.leg1_grf_est(:,3), 'DisplayName', 'leg1')
plot3(mq_telem.leg2_grf_est(:,1), mq_telem.leg2_grf_est(:,2), mq_telem.leg2_grf_est(:,3), 'DisplayName', 'leg2')
plot3(mq_telem.leg3_grf_est(:,1), mq_telem.leg3_grf_est(:,2), mq_telem.leg3_grf_est(:,3), 'DisplayName', 'leg3')

grid on
daspect([1 1 1])
xlabel("x [N] (fore-aft)")
ylabel("y [N] (latero-medial)")
zlabel("z [N] (vertical)")
view([-140 30])
legend()
title("3D foot GRFs")

hold off;

%% tibia plotting

figure;
hold on;

% yyaxis left;
plot(mq_time, mq_telem.leg0_tau_ff(:,3), 'r-', 'DisplayName', 'leg0 cmd')
plot(mq_time, mq_telem.leg0_tau_est(:,3), 'r.', 'DisplayName', 'leg0 est')

plot(mq_time, mq_telem.leg1_tau_ff(:,3), 'b-', 'DisplayName', 'leg1 cmd')
plot(mq_time, mq_telem.leg1_tau_est(:,3), 'b.', 'DisplayName', 'leg1 est')

plot(mq_time, mq_telem.leg2_tau_ff(:,3), 'k-', 'DisplayName', 'leg2 cmd')
plot(mq_time, mq_telem.leg2_tau_est(:,3), 'k.', 'DisplayName', 'leg2 est')

plot(mq_time, mq_telem.leg3_tau_ff(:,3), 'm-', 'DisplayName', 'leg3 cmd')
plot(mq_time, mq_telem.leg3_tau_est(:,3), 'm.', 'DisplayName', 'leg3 est')

% xlim([34.5, 35.5])

% yyaxis right;
% plot(mq_time, mq_telem.leg0_qd_data(:,3), 'r:', 'DisplayName', 'leg0 vel')
% plot(mq_time, mq_telem.leg1_qd_data(:,3), 'b:', 'DisplayName', 'leg1 vel')
% plot(mq_time, mq_telem.leg2_qd_data(:,3), 'k:', 'DisplayName', 'leg2 vel')
% plot(mq_time, mq_telem.leg3_qd_data(:,3), 'm:', 'DisplayName', 'leg3 vel')

title("tibia torque")
legend()

hold off

%% grf z plotting

figure
hold on

plot(mq_time, mq_telem.leg0_grf_cmd(:,3), 'r', 'DisplayName', 'leg0 cmd')
plot(mq_time, mq_telem.leg0_grf_est(:,3), 'r.', 'DisplayName', 'leg0 est')

plot(mq_time, mq_telem.leg1_grf_cmd(:,3), 'b', 'DisplayName', 'leg1 cmd')
plot(mq_time, mq_telem.leg1_grf_est(:,3), 'b.', 'DisplayName', 'leg1 est')

plot(mq_time, mq_telem.leg2_grf_cmd(:,3), 'k', 'DisplayName', 'leg2 cmd')
plot(mq_time, mq_telem.leg2_grf_est(:,3), 'k.', 'DisplayName', 'leg2 est')

plot(mq_time, mq_telem.leg3_grf_cmd(:,3), 'm', 'DisplayName', 'leg3 cmd')
plot(mq_time, mq_telem.leg3_grf_est(:,3), 'm.', 'DisplayName', 'leg3 est')

plot(mq_time, mq_telem.leg0_grf_cmd(:,3)...
    +mq_telem.leg1_grf_cmd(:,3)...
    +mq_telem.leg2_grf_cmd(:,3)...
    +mq_telem.leg3_grf_cmd(:,3), 'g', 'DisplayName', 'total cmd')
plot(mq_time, mq_telem.leg0_grf_est(:,3)...
    +mq_telem.leg1_grf_est(:,3)...
    +mq_telem.leg2_grf_est(:,3)...
    +mq_telem.leg3_grf_est(:,3), 'g.', 'DisplayName', 'total est')
ylabel("GRF z [N]")
legend()
% xlim([35.5, 40])
hold off

%% torso pos z plotting
figure;
plot(mq_time, mq_telem.torso_pos(:,3));

%% torso rpy plotting
figure;

time_mask = mq_time > 0;

subplot(2,1,1)
hold on
plot(mq_time(time_mask), (180/pi)*mq_telem.torso_rpy((time_mask),1), 'DisplayName',"roll");
plot(mq_time(time_mask), (180/pi)*mq_telem.torso_rpy((time_mask),2), 'DisplayName',"pitch");
plot(mq_time(time_mask), (180/pi)*mq_telem.torso_rpy((time_mask),3), 'DisplayName',"yaw");
legend()
title("angle")
hold off;

subplot(2,1,2)
hold on
plot(mq_time(time_mask), (180/pi)*mq_telem.torso_omega((time_mask),1), 'DisplayName',"roll");
plot(mq_time(time_mask), (180/pi)*mq_telem.torso_omega((time_mask),2), 'DisplayName',"pitch");
plot(mq_time(time_mask), (180/pi)*mq_telem.torso_omega((time_mask),3), 'DisplayName',"yaw");
legend()
title("angle rate")
hold off;

%% Torso velocity

figure;
hold on
plot(mq_time, mq_telem.torso_vel(:,1), 'DisplayName',"x");
plot(mq_time, mq_telem.torso_vel(:,2), 'DisplayName',"y");
plot(mq_time, mq_telem.torso_vel(:,3), 'DisplayName',"z");
legend()
title("body velocity")
hold off;
%% Torso position

figure;
hold on
plot(mq_time, mq_telem.torso_pos(:,1), 'DisplayName',"x");
plot(mq_time, mq_telem.torso_pos(:,2), 'DisplayName',"y");
plot(mq_time, mq_telem.torso_pos(:,3), 'DisplayName',"z");
legend()
title("body position")
hold off;

%% Torso 2D position

time_mask = mq_time > 125 & mq_time < 126.7;

% time_mask = mq_time > 0 & mq_time < inf;
time_indices = find(time_mask);

figure;
hold on
plot(-mq_telem.torso_pos(time_mask,2), mq_telem.torso_pos(time_mask,1));

plot(-mq_telem.torso_pos((time_indices(1)),2), mq_telem.torso_pos((time_indices(1)),1), "k*");

plot(-mq_telem.torso_pos((time_indices(end)),2), mq_telem.torso_pos((time_indices(end)),1), "r*");
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

lims = [26.8, 27.8];
lims = [0, inf];
actuator_num = 3;

subplot(3,1,1)
hold on

plot(mq_time, mq_telem.leg0_q_cmd(:,actuator_num) * 180/pi, 'r', 'DisplayName',"mq0 x" + actuator_num + " cmd")
plot(mq_time, mq_telem.leg0_q_data(:,actuator_num) * 180/pi, 'r.', 'DisplayName',"mq0 x" + actuator_num + " data")

plot(mq_time, mq_telem.leg1_q_cmd(:,actuator_num) * 180/pi, 'b', 'DisplayName',"mq1 x" + actuator_num + " cmd")
plot(mq_time, mq_telem.leg1_q_data(:,actuator_num) * 180/pi, 'b.', 'DisplayName',"mq1 x" + actuator_num + " data")

plot(mq_time, mq_telem.leg2_q_cmd(:,actuator_num) * 180/pi, 'k', 'DisplayName',"mq2 x" + actuator_num + " cmd")
plot(mq_time, mq_telem.leg2_q_data(:,actuator_num) * 180/pi, 'k.', 'DisplayName',"mq2 x" + actuator_num + " data")

plot(mq_time, mq_telem.leg3_q_cmd(:,actuator_num) * 180/pi, 'm', 'DisplayName',"mq3 x" + actuator_num + " cmd")
plot(mq_time, mq_telem.leg3_q_data(:,actuator_num) * 180/pi, 'm.', 'DisplayName',"mq3 x" + actuator_num + " data")
title("position q")
legend("Location","best")
xlim(lims)
hold off

subplot(3,1,2)
hold on

plot(mq_time, mq_telem.leg0_qd_cmd(:,actuator_num) * 180/pi, 'r', 'DisplayName',"mq0 x" + actuator_num + " cmd")
plot(mq_time, mq_telem.leg0_qd_data(:,actuator_num) * 180/pi, 'r.', 'DisplayName',"mq0 x" + actuator_num + " data")

plot(mq_time, mq_telem.leg1_qd_cmd(:,actuator_num) * 180/pi, 'b', 'DisplayName',"mq1 x" + actuator_num + " cmd")
plot(mq_time, mq_telem.leg1_qd_data(:,actuator_num) * 180/pi, 'b.', 'DisplayName',"mq1 x" + actuator_num + " data")

plot(mq_time, mq_telem.leg2_qd_cmd(:,actuator_num) * 180/pi, 'k', 'DisplayName',"mq2 x" + actuator_num + " cmd")
plot(mq_time, mq_telem.leg2_qd_data(:,actuator_num) * 180/pi, 'k.', 'DisplayName',"mq2 x" + actuator_num + " data")

plot(mq_time, mq_telem.leg3_qd_cmd(:,actuator_num) * 180/pi, 'm', 'DisplayName',"mq3 x" + actuator_num + " cmd")
plot(mq_time, mq_telem.leg3_qd_data(:,actuator_num) * 180/pi, 'm.', 'DisplayName',"mq3 x" + actuator_num + " data")
title("velocity qd")
% legend()
xlim(lims)
hold off

subplot(3,1,3)
hold on
title("torques")
plot(mq_time, mq_telem.leg0_tau_ff(:,actuator_num), 'r-', 'DisplayName', 'leg0 cmd')
plot(mq_time, mq_telem.leg0_tau_est(:,actuator_num), 'r.', 'DisplayName', 'leg0 est')

plot(mq_time, mq_telem.leg1_tau_ff(:,actuator_num), 'b-', 'DisplayName', 'leg1 cmd')
plot(mq_time, mq_telem.leg1_tau_est(:,actuator_num), 'b.', 'DisplayName', 'leg1 est')

plot(mq_time, mq_telem.leg2_tau_ff(:,actuator_num), 'k-', 'DisplayName', 'leg2 cmd')
plot(mq_time, mq_telem.leg2_tau_est(:,actuator_num), 'k.', 'DisplayName', 'leg2 est')

plot(mq_time, mq_telem.leg3_tau_ff(:,actuator_num), 'm-', 'DisplayName', 'leg3 cmd')
plot(mq_time, mq_telem.leg3_tau_est(:,actuator_num), 'm.', 'DisplayName', 'leg3 est')

xlim(lims)
hold off

%% Actuator Kp and Kd plotting

figure;

lims = [27, 27.6];
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







function mq_telem = parse_table(T)
    mq_telem = struct;

    mq_telem.leg0_qd_data = [T.data_qd_0, T.data_qd_1,  T.data_qd_2];
    mq_telem.leg1_qd_data = [T.data_qd_3, T.data_qd_4,  T.data_qd_5];
    mq_telem.leg2_qd_data = [T.data_qd_6, T.data_qd_7,  T.data_qd_8];
    mq_telem.leg3_qd_data = [T.data_qd_9, T.data_qd_10, T.data_qd_11];

    mq_telem.leg0_qd_cmd = [T.cmd_qd_des_0, T.cmd_qd_des_1,  T.cmd_qd_des_2];
    mq_telem.leg1_qd_cmd = [T.cmd_qd_des_3, T.cmd_qd_des_4,  T.cmd_qd_des_5];
    mq_telem.leg2_qd_cmd = [T.cmd_qd_des_6, T.cmd_qd_des_7,  T.cmd_qd_des_8];
    mq_telem.leg3_qd_cmd = [T.cmd_qd_des_9, T.cmd_qd_des_10, T.cmd_qd_des_11];
    
    mq_telem.leg0_foot_pos = [T.data_p_0, T.data_p_1, T.data_p_2];
    mq_telem.leg1_foot_pos = [T.data_p_3, T.data_p_4, T.data_p_5];
    mq_telem.leg2_foot_pos = [T.data_p_6, T.data_p_7, T.data_p_8];
    mq_telem.leg3_foot_pos = [T.data_p_9, T.data_p_10, T.data_p_11];

    mq_telem.leg0_q_data = [T.data_q_0, T.data_q_1, T.data_q_2];
    mq_telem.leg1_q_data = [T.data_q_3, T.data_q_4, T.data_q_5];
    mq_telem.leg2_q_data = [T.data_q_6, T.data_q_7, T.data_q_8];
    mq_telem.leg3_q_data = [T.data_q_9, T.data_q_10, T.data_q_11];
    
    mq_telem.leg0_q_cmd = [T.cmd_q_des_0, T.cmd_q_des_1,  T.cmd_q_des_2];
    mq_telem.leg1_q_cmd = [T.cmd_q_des_3, T.cmd_q_des_4,  T.cmd_q_des_5];
    mq_telem.leg2_q_cmd = [T.cmd_q_des_6, T.cmd_q_des_7,  T.cmd_q_des_8];
    mq_telem.leg3_q_cmd = [T.cmd_q_des_9, T.cmd_q_des_10, T.cmd_q_des_11];

    mq_telem.leg0_tau_ff = [T.cmd_tau_ff_0, T.cmd_tau_ff_1, T.cmd_tau_ff_2];
    mq_telem.leg1_tau_ff = [T.cmd_tau_ff_3, T.cmd_tau_ff_4, T.cmd_tau_ff_5];
    mq_telem.leg2_tau_ff = [T.cmd_tau_ff_6, T.cmd_tau_ff_7, T.cmd_tau_ff_8];
    mq_telem.leg3_tau_ff = [T.cmd_tau_ff_9, T.cmd_tau_ff_10, T.cmd_tau_ff_11];

    mq_telem.leg0_tau_est = [T.data_tau_est_0, T.data_tau_est_1, T.data_tau_est_2];
    mq_telem.leg1_tau_est = [T.data_tau_est_3, T.data_tau_est_4, T.data_tau_est_5];
    mq_telem.leg2_tau_est = [T.data_tau_est_6, T.data_tau_est_7, T.data_tau_est_8];
    mq_telem.leg3_tau_est = [T.data_tau_est_9, T.data_tau_est_10, T.data_tau_est_11];

    mq_telem.leg0_tau_est = [T.data_tau_est_0, T.data_tau_est_1, T.data_tau_est_2];
    mq_telem.leg1_tau_est = [T.data_tau_est_3, T.data_tau_est_4, T.data_tau_est_5];
    mq_telem.leg2_tau_est = [T.data_tau_est_6, T.data_tau_est_7, T.data_tau_est_8];
    mq_telem.leg3_tau_est = [T.data_tau_est_9, T.data_tau_est_10, T.data_tau_est_11];

    mq_telem.leg0_kp_joint = [T.kp_joint_0, T.kp_joint_1, T.kp_joint_2];
    mq_telem.leg1_kp_joint = [T.kp_joint_3, T.kp_joint_4, T.kp_joint_5];
    mq_telem.leg2_kp_joint = [T.kp_joint_6, T.kp_joint_7, T.kp_joint_8];
    mq_telem.leg3_kp_joint = [T.kp_joint_9, T.kp_joint_10, T.kp_joint_11];

    mq_telem.leg0_kd_joint = [T.kd_joint_0, T.kd_joint_1, T.kd_joint_2];
    mq_telem.leg1_kd_joint = [T.kd_joint_3, T.kd_joint_4, T.kd_joint_5];
    mq_telem.leg2_kd_joint = [T.kd_joint_6, T.kd_joint_7, T.kd_joint_8];
    mq_telem.leg3_kd_joint = [T.kd_joint_9, T.kd_joint_10, T.kd_joint_11];

    mq_telem.leg0_kp_cartesian = [T.kp_cartesian_0, T.kp_cartesian_1, T.kp_cartesian_2];
    mq_telem.leg1_kp_cartesian = [T.kp_cartesian_3, T.kp_cartesian_4, T.kp_cartesian_5];
    mq_telem.leg2_kp_cartesian = [T.kp_cartesian_6, T.kp_cartesian_7, T.kp_cartesian_8];
    mq_telem.leg3_kp_cartesian = [T.kp_cartesian_9, T.kp_cartesian_10, T.kp_cartesian_11];

    mq_telem.leg0_kd_cartesian = [T.kd_cartesian_0, T.kd_cartesian_1, T.kd_cartesian_2];
    mq_telem.leg1_kd_cartesian = [T.kd_cartesian_3, T.kd_cartesian_4, T.kd_cartesian_5];
    mq_telem.leg2_kd_cartesian = [T.kd_cartesian_6, T.kd_cartesian_7, T.kd_cartesian_8];
    mq_telem.leg3_kd_cartesian = [T.kd_cartesian_9, T.kd_cartesian_10, T.kd_cartesian_11];

    mq_telem.leg0_grf_cmd = mq_telem.leg0_tau_est;
    mq_telem.leg1_grf_cmd = mq_telem.leg1_tau_est;
    mq_telem.leg2_grf_cmd = mq_telem.leg2_tau_est;
    mq_telem.leg3_grf_cmd = mq_telem.leg3_tau_est;

    mq_telem.leg0_grf_est = mq_telem.leg0_tau_est;
    mq_telem.leg1_grf_est = mq_telem.leg1_tau_est;
    mq_telem.leg2_grf_est = mq_telem.leg2_tau_est;
    mq_telem.leg3_grf_est = mq_telem.leg3_tau_est;

    mq_telem.torso_pos = [T.p_0, T.p_1, T.p_2];
    mq_telem.torso_vel = [T.vBody_0, T.vBody_1, T.vBody_2];

    mq_telem.torso_rpy = [T.rpy_0, T.rpy_1, T.rpy_2];
    mq_telem.torso_omega = [T.omegaBody_0, T.omegaBody_1, T.omegaBody_2];

    for ii = 1:length(T.data_p_0)
        q_vec = mq_telem.leg0_q_data(ii,:)';
        sideSign = -1;
        J = foot_jacobian(q_vec, sideSign);
        u_vec = mq_telem.leg0_tau_ff(ii,:)';
        mq_telem.leg0_grf_cmd(ii,:) = (inv(J')*u_vec)';
        u_vec = mq_telem.leg0_tau_est(ii,:)';
        mq_telem.leg0_grf_est(ii,:) = (inv(J')*u_vec)';

        q_vec = mq_telem.leg1_q_data(ii,:)';
        sideSign = 1;
        J = foot_jacobian(q_vec, sideSign);
        u_vec = mq_telem.leg1_tau_ff(ii,:)';
        mq_telem.leg1_grf_cmd(ii,:) = (inv(J')*u_vec)';
        u_vec = mq_telem.leg1_tau_est(ii,:)';
        mq_telem.leg1_grf_est(ii,:) = (inv(J')*u_vec)';

        q_vec = mq_telem.leg2_q_data(ii,:)';
        sideSign = -1;
        J = foot_jacobian(q_vec, sideSign);
        u_vec = mq_telem.leg2_tau_ff(ii,:)';
        mq_telem.leg2_grf_cmd(ii,:) = (inv(J')*u_vec)';
        u_vec = mq_telem.leg2_tau_est(ii,:)';
        mq_telem.leg2_grf_est(ii,:) = (inv(J')*u_vec)';

        q_vec = mq_telem.leg3_q_data(ii,:)';
        sideSign = 1;
        J = foot_jacobian(q_vec, sideSign);
        u_vec = mq_telem.leg3_tau_ff(ii,:)';
        mq_telem.leg3_grf_cmd(ii,:) = (inv(J')*u_vec)';
        u_vec = mq_telem.leg3_tau_est(ii,:)';
        mq_telem.leg3_grf_est(ii,:) = (inv(J')*u_vec)';
    end

end
