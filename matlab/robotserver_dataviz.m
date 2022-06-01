torsopos1 = "data\robot_server_05_05_2022_13-57-11.csv";

grf_tes_rs1 = "data\robot_server_06_05_2022_01-50-31.csv";

temperature_rs1 = "data\robot_server_06_05_2022_16-34-25.csv"; % derate temp at 60C

temperature_rs2 = "data\robot_server_09_05_2022_15-22-43.csv"; % heatsinks on mq2 and mq4

wbc_test_2 = "data\robot_server_10_05_2022_14-38-17.csv"; % mq leg 2 femur vibrated off. Kp 400,Kd 3.25,f_ff -20

temperature_rs3 = "data\robot_server_10_05_2022_16-42-06"; % a23 has vented motor housing
standup_rs1 = "data\robot_server_10_05_2022_16-59-06.csv"; % rear legs more ff in z
standup_rs2 = "data\robot_server_10_05_2022_17-14-48.csv"; % mq leg 3 popped off lmao

wbc_tuning_1 = "data\robot_server_12_05_2022_13-37-29.csv"; % front legs jumped, intermittent communication

temperature_rs4 = "data\robot_server_13_05_2022_15-56-54.csv"; % WBC balance stand, big fan on
temperature_rs5 = "data\robot_server_13_05_2022_16-10-19.csv"; % big fan off
temperature_rs6 = "data\robot_server_13_05_2022_16-47-30.csv"; % heat spreaders
temperature_rs7 = "data\robot_server_16_05_2022_14-06-17.csv"; % heat spreaders with big heatsink

bad_balancestand1 = "data\robot_server_16_05_2022_16-59-26.csv";

sine_wave_debug_1 = "data\robot_server_16_05_2022_19-00-28.csv";
sine_wave_debug_2 = "data\robot_server_16_05_2022_19-06-36.csv";

trot_attempt_10_rs = "data\robot_server_24_05_2022_15-05-35.csv";

new_actuator_thermal_1 = "data\robot_server_26_05_2022_21-39-51.csv";

T_rs = readtable(new_actuator_thermal_1);

offset_angles = false;
MQ_MOT_ROT = pi/7.5;
MQ_MOT_ROT_12 = pi/12;
angles = [0,-2*MQ_MOT_ROT,4*MQ_MOT_ROT,...
           0,2*MQ_MOT_ROT,-4*MQ_MOT_ROT,...
           0,-2*MQ_MOT_ROT,8*MQ_MOT_ROT_12,...
           0,2*MQ_MOT_ROT,-8*MQ_MOT_ROT_12];
if offset_angles
    T_rs.a11PositionCmd_rad_ = T_rs.a11PositionCmd_rad_ - angles(1);
    T_rs.a12PositionCmd_rad_ = T_rs.a12PositionCmd_rad_ - angles(2);
    T_rs.a13PositionCmd_rad_ = T_rs.a13PositionCmd_rad_ - angles(3);
    T_rs.a21PositionCmd_rad_ = T_rs.a21PositionCmd_rad_ - angles(4);
    T_rs.a22PositionCmd_rad_ = T_rs.a22PositionCmd_rad_ - angles(5);
    T_rs.a23PositionCmd_rad_ = T_rs.a23PositionCmd_rad_ - angles(6);
    T_rs.a31PositionCmd_rad_ = T_rs.a31PositionCmd_rad_ - angles(7);
    T_rs.a32PositionCmd_rad_ = T_rs.a32PositionCmd_rad_ - angles(8);
    T_rs.a33PositionCmd_rad_ = T_rs.a33PositionCmd_rad_ - angles(9);
    T_rs.a41PositionCmd_rad_ = T_rs.a41PositionCmd_rad_ - angles(10);
    T_rs.a42PositionCmd_rad_ = T_rs.a42PositionCmd_rad_ - angles(11);
    T_rs.a43PositionCmd_rad_ = T_rs.a43PositionCmd_rad_ - angles(12);
end
%%

figure;

subplot(2,1,1)
hold on
plot(T_rs.time_s_, -T_rs.a13PositionCmd_rad_, 'DisplayName',"a13 cmd")
plot(T_rs.time_s_, T_rs.a23PositionCmd_rad_, 'DisplayName',"a23 cmd")
plot(T_rs.time_s_, -T_rs.a33PositionCmd_rad_, 'DisplayName',"a33 cmd")
plot(T_rs.time_s_, T_rs.a43PositionCmd_rad_, 'DisplayName',"a43 cmd")
% plot(T.time_s_, T.a43Position_rad_, 'DisplayName',"pos")
legend()
title("position")
hold off;

%%

% subplot(2,1,2)
figure;
hold on

yyaxis left
% plot(T.time_s_, T.a43PositionCmd_rad_, 'DisplayName',"cmd")
plot(T_rs.time_s_, -T_rs.a13Torque_Nm_, 'm.', 'DisplayName',"tau13")
plot(T_rs.time_s_, T_rs.a63Torque_Nm_, 'r.', 'DisplayName',"tau63")
plot(T_rs.time_s_, -T_rs.a33Torque_Nm_, 'b.', 'DisplayName',"tau33")
plot(T_rs.time_s_, T_rs.a43Torque_Nm_, 'k.', 'DisplayName',"tau43")

plot(T_rs.time_s_, -T_rs.a13FfTorqueCmd_Nm_, 'm-', 'DisplayName',"tau13 cmd")
plot(T_rs.time_s_, T_rs.a63FfTorqueCmd_Nm_, 'r-', 'DisplayName',"tau63 cmd")
plot(T_rs.time_s_, -T_rs.a33FfTorqueCmd_Nm_, 'b-', 'DisplayName',"tau33 cmd")
plot(T_rs.time_s_, T_rs.a43FfTorqueCmd_Nm_, 'k-', 'DisplayName',"tau43 cmd")

plot_temp = true;
if plot_temp
    yyaxis right
    plot(T_rs.time_s_, T_rs.a13Temp_C_, 'm:', 'DisplayName','a13 temp');
    plot(T_rs.time_s_, T_rs.a63Temp_C_, 'r:', 'DisplayName','a63 temp');
    plot(T_rs.time_s_, T_rs.a33Temp_C_, 'b:', 'DisplayName','a33 temp');
    plot(T_rs.time_s_, T_rs.a43Temp_C_, 'k:', 'DisplayName','a43 temp');
end

plot_faults = false;
if plot_faults
    yyaxis right
    plot(T_rs.time_s_, T_rs.a13Fault, 'mo', 'DisplayName','a13 fault');
    plot(T_rs.time_s_, T_rs.a63Fault, 'r.', 'DisplayName','a63 fault');
    plot(T_rs.time_s_, T_rs.a33Fault, 'bx', 'DisplayName','a33 fault');
    plot(T_rs.time_s_, T_rs.a43Fault, 'kd', 'DisplayName','a43 fault');
end

plot_thermal_model = true;
if plot_thermal_model
    RWH = .828; RHA = 1.44; CWH = 15.9; CWA = 146;
    syms s;
    TF = (RHA + RWH + CHA*RHA*RWH*s) /...
        ( (CHA*RHA*s+1)*(CQA*RWH*s + 1) );
end


% plot(T_rs.time_s_(T_rs.a13Temp_C_ > 50), T_rs.a13Temp_C_(T_rs.a13Temp_C_ > 50), 'm:', 'linewidth', 3,'HandleVisibility','off');
% plot(T_rs.time_s_(T_rs.a23Temp_C_ > 50), T_rs.a23Temp_C_(T_rs.a23Temp_C_ > 50), 'r:', 'linewidth', 3,'HandleVisibility','off');
% plot(T_rs.time_s_(T_rs.a33Temp_C_ > 50), T_rs.a33Temp_C_(T_rs.a33Temp_C_ > 50), 'b:', 'linewidth', 3,'HandleVisibility','off');
% plot(T_rs.time_s_(T_rs.a43Temp_C_ > 50), T_rs.a43Temp_C_(T_rs.a43Temp_C_ > 50), 'k:', 'linewidth', 3,'HandleVisibility','off');

legend()
title("torque")
hold off;

%% thermal
figure

hold on;
plot(T_rs.time_s_, T_rs.a13Temp_C_, 'm:', 'DisplayName','a13 temp');
plot(T_rs.time_s_, T_rs.a63Temp_C_, 'r:', 'DisplayName','a23 temp');
plot(T_rs.time_s_, T_rs.a33Temp_C_, 'b:', 'DisplayName','a33 temp');
plot(T_rs.time_s_, T_rs.a43Temp_C_, 'k:', 'DisplayName','a43 temp');
legend('Location','best')
title("temperature")
ylabel("x3 moteus temp [degC]")
xlabel("time")
hold off

%% Temperature

figure;

fnames = [temperature_rs4, temperature_rs5, temperature_rs6, temperature_rs7];
fnames = [new_actuator_thermal_1];
tstarts = [50.2, 25.6, 44.5, 34.6];
tstarts = [33.75];
labels = ["big fan on", "big fan off", "heat spreader", "big heatsink"];
labels = ["new actuator"];
colors = ["r", "b", "k", "m"];
colors = ["r"];
hold on;

for ii = 1:length(fnames)
    fname = fnames(ii);
    T_rs = readtable(fname);
    mean_temp = T_rs.a13Temp_C_ + T_rs.a63Temp_C_ ...
        +T_rs.a33Temp_C_ + T_rs.a43Temp_C_;
    mean_temp = mean_temp/4;
    tstart = tstarts(ii);
    time = T_rs.time_s_ - tstart;
    plot(time, mean_temp, colors(ii), 'DisplayName',labels(ii),'LineWidth',3);

    downsample_mask = 1:100:length(time);

    plot(time(downsample_mask), T_rs.a13Temp_C_(downsample_mask), colors(ii)+':', 'HandleVisibility','off','LineWidth',0.5);
    plot(time(downsample_mask), T_rs.a63Temp_C_(downsample_mask), colors(ii)+':', 'HandleVisibility','off','LineWidth',0.5);
    plot(time(downsample_mask), T_rs.a33Temp_C_(downsample_mask), colors(ii)+':', 'HandleVisibility','off','LineWidth',0.5);
    plot(time(downsample_mask), T_rs.a43Temp_C_(downsample_mask), colors(ii)+':', 'HandleVisibility','off','LineWidth',0.5);
end
xlim([-4 inf]);
legend('Location','best')
title("temperature")
ylabel("x3 moteus mean temp [degC]")
xlabel("time")
hold off