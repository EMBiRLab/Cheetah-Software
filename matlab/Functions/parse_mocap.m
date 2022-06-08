
function mocap_data = parse_mocap(mT)
    md = struct;
    md.marker_ids = unique(mT.id( char(mT.type) == 'M' ) );

    forward_marker_id = 7;
    rear_marker_id = 10;
    if (~ismember(md.marker_ids, forward_marker_id))
        forward_marker_id = forward_marker_id + 8;
        rear_marker_id = rear_marker_id + 8;
    end
    md.time_stamps = unique(mT.time);
    num_samples = length(md.time_stamps);
    fprintf("%d samples\n", num_samples)

    calib_time_begin = md.time_stamps(1);
    calib_time_end = md.time_stamps(10);

    % find robot x-axis and origin
    init_y_offset_mm = 108+10+1; % height of forward/rear markers relative to torso origin (extra 1mm to account for marker thickness)
    
    calib_mocap_x_forward = mT.pos_x(mT.id == forward_marker_id);
    calib_mocap_x_rear = mT.pos_x(mT.id == rear_marker_id);
    calib_mocap_y_forward = mT.pos_y(mT.id == forward_marker_id);
    calib_mocap_y_rear = mT.pos_y(mT.id == rear_marker_id);
    calib_mocap_z_forward = mT.pos_z(mT.id == forward_marker_id);
    calib_mocap_z_rear = mT.pos_z(mT.id == rear_marker_id);

    origin = [...
        mean(0.5*calib_mocap_x_forward(10:20) + 0.5*calib_mocap_x_rear(10:20));...
%         mean(0.5*calib_mocap_y_forward(10:20) + 0.5*calib_mocap_y_rear(10:20)) - init_y_offset_mm;...
        mean(0.5*calib_mocap_y_forward(10:20) + 0.5*calib_mocap_y_rear(10:20));...
        mean(0.5*calib_mocap_z_forward(10:20) + 0.5*calib_mocap_z_rear(10:20))];
    
    % robot_forward_vec is how the robot's +x axis looks in the mocap frame
    robot_forward_vec = [...
        (calib_mocap_x_forward(1:num_samples-2000) - calib_mocap_x_rear(1:num_samples-2000))';...
        (calib_mocap_y_forward(1:num_samples-2000) - calib_mocap_y_rear(1:num_samples-2000))';...
        (calib_mocap_z_forward(1:num_samples-2000) - calib_mocap_z_rear(1:num_samples-2000))'];
%     robot_forward_vec = robot_forward_vec/norm(robot_forward_vec);

    cal_vec = mean(robot_forward_vec(:,3:10), 2);

    % center robot rigid body data on origin
    RB_mask = char(mT.type) == 'R';
    RB_pos = [mT.pos_x(RB_mask)'; mT.pos_y(RB_mask)'; mT.pos_z(RB_mask)'];
    md.RB_pos_mocap = RB_pos;
    RB_pos_init = RB_pos(:,1);
%     RB_pos_offset = RB_pos_init - origin;
    RB_pos_offset = origin;
    RB_pos(1,:) = RB_pos(1,:) - RB_pos_offset(1);
    RB_pos(2,:) = RB_pos(2,:) - RB_pos_offset(2);
    RB_pos(3,:) = RB_pos(3,:) - RB_pos_offset(3);

    % physically -- yaw orientation of robot (y-axis in mocap frame)
    y_angle = atan2(cal_vec(3), cal_vec(1));
    Ry = [cos(y_angle) 0 sin(y_angle); 0 1 0; -sin(y_angle) 0 cos(y_angle)];
    Rx = [1 0 0; 0 cos(pi/2) -sin(pi/2); 0 sin(pi/2) cos(pi/2)];

    % reorient RB_pos
    RB_pos = Ry*RB_pos; % "yaw" in mocap frame (y-axis rotation)
    RB_pos = Rx*RB_pos;
    md.RB_pos = RB_pos;

    % reorient quaterions
    RB_quat = [mT.rot_w(RB_mask)'; mT.rot_x(RB_mask)'; mT.rot_y(RB_mask)'; mT.rot_z(RB_mask)'];
    qy = quaternion(rotm2quat(Ry));
    qx = quaternion(rotm2quat(Rx));

    RB_quat = quaternion(RB_quat');
    md.RB_quat_mocap = RB_quat;

    RB_quat = qy*RB_quat*conj(qy);
%     RB_quat = RB_quat*qy;
    RB_quat = qx*RB_quat*conj(qx);
%     RB_quat = RB_quat*qx;

    q_start = RB_quat(3:20);
    q_start = meanrot(q_start);
    qtransform = quaternion([1 0 0 0]) * q_start';
    RB_quat = qtransform*RB_quat;
    
    md.RB_quat = RB_quat;
    md.RB_rpy = quat2eul(RB_quat, "XYZ");
    md.num_samples = num_samples - 1;
    md.time = (md.time_stamps(2:end) - md.time_stamps(2)) * 1e-6;

    mocap_data = md;

    % test angle math

    test_angles = atan2(...
        calib_mocap_z_forward(10:num_samples-2000) - calib_mocap_z_rear(10:num_samples-2000),...
        calib_mocap_x_forward(10:num_samples-2000) - calib_mocap_x_rear(10:num_samples-2000))...
        - y_angle;
    test_angles = unwrap(mod(test_angles+10*pi, 2*pi));
    quat_angles = quat2eul(RB_quat, "XYZ");
    new_yaws = unwrap(mod(quat_angles(10:num_samples-2000, 3) + 2*pi, 2*pi));
    new_pitch = unwrap(mod(quat_angles(10:num_samples-2000, 2) + 2*pi, 2*pi));
    new_roll = unwrap(mod(quat_angles(10:num_samples-2000, 1) + 2*pi, 2*pi));
    diff_angles = (new_yaws-test_angles);
    fprintf("mean: %f, std: %f\n", mean(diff_angles), std(diff_angles));
%     figure;
%     hold on;
%     plot(new_yaws*180/pi, "displayname","new yaws");
%     plot(test_angles*180/pi, "displayname","test angles");
%     legend("location","best");
%     hold off;

end
