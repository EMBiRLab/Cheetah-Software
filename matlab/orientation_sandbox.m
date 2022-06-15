%% Load Data

trot_12_1_2 = "data/mq_telem_01_06_2022_17-09-25.csv"; % same as above, but did torso orientation demo to verify wbc without mpc

% Set mounting-roll to 180, mounting-pitch to 90, and applied 180deg 
% rotation about roll in MIT data unpacking. Less drift for robot, still
% pitch back in locomotion state
imu_drift_test_4 = "data/mq_telem_07_06_2022_15-01-45.csv";
imu_drift_test_4_mocap = "data/muadquad_6_7_22_004.csv";

% orientation test in balance stand
orientation_test_1 = "data/mq_telem_08_06_2022_15-54-05.csv";
orientation_test_1_mocap = "data/muadquad_6_7_22_005.csv";

sample_freq = 500; % Hz
T = readtable(imu_drift_test_4);
% mq_time = (1:1:height(T))/sample_freq;
headers = T.Properties.VariableNames;
mq_telem = parse_mq_telem_table(T);
mq_time = mq_telem.time;


mocap_T = readtable(imu_drift_test_4_mocap);
mocap_data = parse_mocap(mocap_T);

%% RPY Preview

mq_telem_time_mark = 39.7134;
mocap_time_mark = 19.2115;

mq_telem_time_mark = 35.9679;
mocap_time_mark = 48.1823;

mocap_offset_back = mocap_time_mark - mq_telem_time_mark;

figure;

hold on
plot(mq_time, mq_telem.torso_rpy(:,1)*180/pi, 'r-', 'DisplayName','mit roll');
plot(mq_time, mq_telem.torso_rpy(:,2)*180/pi, 'm-', 'DisplayName','mit pitch');
plot(mq_time, mq_telem.torso_rpy(:,3)*180/pi, 'k-', 'DisplayName','mit yaw');


plot(mocap_data.time - mocap_offset_back, mocap_data.RB_rpy(:,1)*180/pi, 'r--', 'DisplayName','mocap roll');
plot(mocap_data.time - mocap_offset_back, mocap_data.RB_rpy(:,2)*180/pi, 'm--', 'DisplayName','mocap pitch');
plot(mocap_data.time - mocap_offset_back, mocap_data.RB_rpy(:,3)*180/pi, 'k--', 'DisplayName','mocap yaw');

xlabel("time [s]")
ylabel("angle [deg]")
legend("Location","best")

hold off

%% Calculation of Orientation from Legs
time_thresh = 79.2;
time_thresh = 87.24;
% time_thresh = 115.24;
index = find(mq_time > time_thresh);
% index = find(mq_time > 87.24);

% index = find(mq_time > 115.24);
index = index(1);

fit_roll = zeros([1, length(mq_time)]);
fit_pitch = zeros([1, length(mq_time)]);
fit_yaw = zeros([1, length(mq_time)]);

normals = zeros([3, length(mq_time)]);

zyx_eul = zeros([3, length(mq_time)]);
xyz_eul = zeros([3, length(mq_time)]);
zyz_eul = zeros([3, length(mq_time)]);

zyx_eul_trans = zeros([3, length(mq_time)]);
xyz_eul_trans = zeros([3, length(mq_time)]);
zyz_eul_trans = zeros([3, length(mq_time)]);

for index = 1:length(mq_time)
% for index = index

x_half_length = .2;
y_half_length = .1;
z_offset = -.068;

leg0_offset = [x_half_length, -y_half_length, z_offset];
leg1_offset = [x_half_length, y_half_length, z_offset];
leg2_offset = [-x_half_length, -y_half_length, z_offset];
leg3_offset = [-x_half_length, y_half_length, z_offset];

torso_points = [leg0_offset;
    leg1_offset;
    leg2_offset;
    leg3_offset;
    leg0_offset;];

points = ...
    [mq_telem.leg0_foot_pos(index, :) + leg0_offset; ...
     mq_telem.leg1_foot_pos(index, :) + leg1_offset; ...
     mq_telem.leg2_foot_pos(index, :) + leg2_offset; ...
     mq_telem.leg3_foot_pos(index, :) + leg3_offset];

fit_results = plane_fit_tls(points);
normal_vec = fit_results.normal;

normals(:,index) = normal_vec;

z_vec = [0;0;1];

% https://math.stackexchange.com/questions/1956699/getting-a-transformation-matrix-from-a-normal-vector
u1 = cross(normal_vec, z_vec);
% u1 = cross(z_vec, normal_vec);
u1 = u1/norm(u1);
u2 = cross(normal_vec, u1);
u2 = u2/norm(u2);
u3 = cross(u1, u2);
u3 = u3/norm(u3);

R = [u1';u2';u3'];
% R = R';
R = inv(R);
eul = rotm2eul(R, "ZYX");

fit_roll(index) = eul(3);
fit_pitch(index) = eul(2);
fit_yaw(index) = eul(1);

zyx_eul(:,index) = rotm2eul(R, "ZYX");
xyz_eul(:,index) = rotm2eul(R, "XYZ");
zyz_eul(:,index) = rotm2eul(R, "ZYZ");

zyx_eul_trans(:,index) = rotm2eul(R', "ZYX");
xyz_eul_trans(:,index) = rotm2eul(R', "XYZ");
zyz_eul_trans(:,index) = rotm2eul(R', "ZYZ");

if ~mod(index, 1000)
fprintf("\ridx = %d / %d", index, length(mq_time));
end

end
% fprintf("time = %.2f, r = %.2f, p = %.2f, y = %.2f\n", time_thresh, eul(1)*180/pi, eul(2)*180/pi, eul(3)*180/pi)
%% Comparison

figure;
hold on

% plot(mq_time, zyx_eul(1,:)*180/pi, '--', "displayname", "zyx\_eul(1,:)")
% plot(mq_time, zyx_eul(2,:)*180/pi, '--', "displayname", "zyx\_eul(2,:)")
% plot(mq_time, zyx_eul(3,:)*180/pi, '--', "displayname", "zyx\_eul(3,:)")
% 
% plot(mq_time, xyz_eul(1,:)*180/pi, ':', "displayname", "xyz\_eul(1,:)")
% plot(mq_time, xyz_eul(2,:)*180/pi, ':', "displayname", "xyz\_eul(2,:)")
% plot(mq_time, xyz_eul(3,:)*180/pi, ':', "displayname", "xyz\_eul(3,:)")

% plot(mq_time, zyx_eul_trans(1,:)*180/pi, '-.', "displayname", "zyx\_eul\_trans(1,:)")
plot(mq_time, zyx_eul_trans(2,:)*180/pi, 'k:', "displayname", "zyx\_eul\_trans(2,:)")
plot(mq_time, zyx_eul_trans(3,:)*180/pi, 'r:', "displayname", "zyx\_eul\_trans(3,:)")

% plot(mq_time, xyz_eul_trans(1,:)*180/pi, '-', "displayname", "xyz\_eul\_trans(1,:)")
% plot(mq_time, xyz_eul_trans(2,:)*180/pi, '-', "displayname", "xyz\_eul\_trans(2,:)")
% plot(mq_time, xyz_eul_trans(3,:)*180/pi, '-', "displayname", "xyz\_eul\_trans(3,:)")

plot(mq_time, mq_telem.torso_rpy(:,1)*180/pi, 'r-', "displayname", "mq roll")
plot(mq_time, mq_telem.torso_rpy(:,2)*180/pi, 'k-', "displayname", "mq pitch")
plot(mq_time, mq_telem.torso_rpy(:,3)*180/pi, 'm-', "displayname", "mq yaw")


plot(mocap_data.time - mocap_offset_back, mocap_data.RB_rpy(:,1)*180/pi, 'r--', 'DisplayName','mocap roll');
plot(mocap_data.time - mocap_offset_back, mocap_data.RB_rpy(:,2)*180/pi, 'k--', 'DisplayName','mocap pitch');
plot(mocap_data.time - mocap_offset_back, mocap_data.RB_rpy(:,3)*180/pi, 'm--', 'DisplayName','mocap yaw');

xlabel("time [s]")
ylabel("angle [deg]")
xlim([0 120])
legend("Location","best")

%% Plot Error of mq vs mocap against augment vs mocap
figure;
mocap_time = mocap_data.time - mocap_offset_back;
mocap_positive_time = find(mocap_time > 0);
mocap_positive_time = mocap_positive_time(1);

mq_error_r = abs(mq_telem.torso_rpy(:,1)*180/pi - interp1(mocap_time(mocap_positive_time:end), mocap_data.RB_rpy(mocap_positive_time:end,1)*180/pi, ...
            linspace( mocap_time(mocap_positive_time), mocap_time(end), length(mq_telem.torso_rpy(:,1)) )' ));

mq_error_p = abs(mq_telem.torso_rpy(:,2)*180/pi - interp1(mocap_time(mocap_positive_time:end), mocap_data.RB_rpy(mocap_positive_time:end,2)*180/pi, ...
            linspace( mocap_time(mocap_positive_time), mocap_time(end), length(mq_telem.torso_rpy(:,2)) )' ));
        
mq_error_y = abs(mq_telem.torso_rpy(:,3)*180/pi - interp1(mocap_time(mocap_positive_time:end), mocap_data.RB_rpy(mocap_positive_time:end,3)*180/pi, ...
            linspace( mocap_time(mocap_positive_time), mocap_time(end), length(mq_telem.torso_rpy(:,3)) )' ));
        
        
zyx_eul_err_r = abs(mq_telem.torso_rpy(:,1)*180/pi - zyx_eul_trans(3,:)'*180/pi);        
zyx_eul_err_p = abs(mq_telem.torso_rpy(:,2)*180/pi - zyx_eul_trans(2,:)'*180/pi);
zyx_eul_err_y = abs(mq_telem.torso_rpy(:,3)*180/pi - zyx_eul_trans(1,:)'*180/pi);

subplot(2,1,1); hold on;
title("mq\_telem error")
plot(mq_time, mq_error_r, 'b-')
plot(mq_time, mq_error_p)
% plot(mq_time, mq_error_y)
% plot(mq_time, interp1(mocap_time(mocap_positive_time:end), mocap_data.RB_rpy(mocap_positive_time:end,1)*180/pi, ...
%             linspace( mocap_time(mocap_positive_time), mocap_time(end), length(mq_telem.torso_rpy(:,1)) )' ), 'o--');
% plot(mq_time, mq_telem.torso_rpy(:,1)*180/pi, 'k:');
ylim([0 15])

subplot(2,1,2); hold on;
title("zyx\_eul error")
plot(mq_time, zyx_eul_err_r)
plot(mq_time, zyx_eul_err_p)
% plot(mq_time, zyx_eul_err_y)
ylim([0 15])

%% 3D Plotting

mean_foot_pos = mean(points, 1);

figure;

hold on;
plot3(torso_points(:,1), torso_points(:,2), torso_points(:,3), "k-");

leg0_3d = [leg0_offset; mq_telem.leg0_foot_pos(index, :) + leg0_offset];
leg1_3d = [leg1_offset; mq_telem.leg1_foot_pos(index, :) + leg1_offset];
leg2_3d = [leg2_offset; mq_telem.leg2_foot_pos(index, :) + leg2_offset];
leg3_3d = [leg3_offset; mq_telem.leg3_foot_pos(index, :) + leg3_offset];
plot3(leg0_3d(:,1), leg0_3d(:,2), leg0_3d(:,3), "k-");
plot3(leg1_3d(:,1), leg1_3d(:,2), leg1_3d(:,3), "k-");
plot3(leg2_3d(:,1), leg2_3d(:,2), leg2_3d(:,3), "k-");
plot3(leg3_3d(:,1), leg3_3d(:,2), leg3_3d(:,3), "k-");

plot3(...
    [mean_foot_pos(1), mean_foot_pos(1) + normal_vec(1)/10],...
    [mean_foot_pos(2), mean_foot_pos(2) + normal_vec(2)/10],...
    [mean_foot_pos(3), mean_foot_pos(3) + normal_vec(3)/10],...
    "r-", 'LineWidth',3)

xlabel("x [m]")
ylabel("y [m]")
zlabel("z [m]")

daspect([1 1 1])
view([-54,19])

hold off;