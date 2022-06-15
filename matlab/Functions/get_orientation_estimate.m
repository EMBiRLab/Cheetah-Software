function orientation_est = get_orientation_estimate(data_t, contact_states)
orientation_est = nan(1,4);


x_half_length = .2;
y_half_length = .1;
z_offset = -.068;

leg0_offset = [ x_half_length, -y_half_length, z_offset];
leg1_offset = [ x_half_length,  y_half_length, z_offset];
leg2_offset = [-x_half_length, -y_half_length, z_offset];
leg3_offset = [-x_half_length,  y_half_length, z_offset];

z_vec = [0;0;1];

torso_points = [leg0_offset;
    leg1_offset;
    leg2_offset;
    leg3_offset;
    leg0_offset;];

points = ...
    [data_t.leg0_foot_pos + leg0_offset; ...
     data_t.leg1_foot_pos + leg1_offset; ...
     data_t.leg2_foot_pos + leg2_offset; ...
     data_t.leg3_foot_pos + leg3_offset];

% we have 5 cases for foot contact:
% no feet, 1 foot, 2 feet, 3 feet, or all 4 feet in contact
if sum(contact_states) > 2
    if (sum(contact_states) == 3)
        passssss
    end
    % we have 3 or 4 feet in contact
    % we should use least squares to fit a plane to the feet, and then
    % figure out the torso orientation
    fit_results = plane_fit_tls(points);
    normal_vec = fit_results.normal;
    
    % https://math.stackexchange.com/questions/1956699/getting-a-transformation-matrix-from-a-normal-vector
    u1 = cross(normal_vec, z_vec);
    u1 = u1/norm(u1);
    u2 = cross(normal_vec, u1);
    u2 = u2/norm(u2);
    u3 = cross(u1, u2);
    u3 = u3/norm(u3);
   
    R = [u1';u2';u3'];
    R = R; % == inv(R);
    
    eul = rotm2eul(R, "ZYX");
    orientation_est = eul2quat(eul);    
    
elseif sum(contact_states) == 2
    % we have 2 feet in contact
    % we should use the orthogonal vector method
    % ASSUMPTION: the swing feet are at the same height
    fit_results = plane_fit_tls(points);
    normal_vec = fit_results.normal;
    
    % https://math.stackexchange.com/questions/1956699/getting-a-transformation-matrix-from-a-normal-vector
    u1 = cross(normal_vec, z_vec);
    u1 = u1/norm(u1);
    u2 = cross(normal_vec, u1);
    u2 = u2/norm(u2);
    u3 = cross(u1, u2);
    u3 = u3/norm(u3);
   
    R = [u1';u2';u3'];
    R = R; % == inv(R);
    
    eul = rotm2eul(R, "ZYX");
    orientation_est = eul2quat(eul);
end

% normals(:,index) = normal_vec;


% u1 = cross(z_vec, normal_vec);


% fit_roll(index) = eul(3);
% fit_pitch(index) = eul(2);
% fit_yaw(index) = eul(1);
% 
% zyx_eul(:,index) = rotm2eul(R, "ZYX");
% xyz_eul(:,index) = rotm2eul(R, "XYZ");
% zyz_eul(:,index) = rotm2eul(R, "ZYZ");
% 
% zyx_eul_trans(:,index) = rotm2eul(R', "ZYX");
% xyz_eul_trans(:,index) = rotm2eul(R', "XYZ");
% zyz_eul_trans(:,index) = rotm2eul(R', "ZYZ");
% 
% if ~mod(index, 1000)
% fprintf("\ridx = %d / %d", index, length(mq_time));
% end

end