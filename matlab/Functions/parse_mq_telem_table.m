
function mq_telem = parse_mq_telem_table(T)
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
    
    mq_telem.torso_pos_des = [T.pBody_des_0, T.pBody_des_1, T.pBody_des_2];
    mq_telem.torso_vel_des = [T.vBody_des_0, T.vBody_des_1, T.vBody_des_2];
    mq_telem.torso_accel_des = [T.aBody_des_0, T.aBody_des_1, T.aBody_des_2];
    mq_telem.torso_rpy_des = [T.pBody_rpy_des_0,T.pBody_rpy_des_1,T.pBody_rpy_des_2];
    mq_telem.torso_v_ori_des = [T.vBody_ori_des_0,T.vBody_ori_des_1,T.vBody_ori_des_2];
    mq_telem.contact_state = [T.contact_state_0,T.contact_state_1,T.contact_state_2,T.contact_state_3];

    if any(ismember(T.Properties.VariableNames,'accellerometer_0'))
        mq_telem.accelerometer = [T.accelerometer_0,T.accelerometer_1,T.accelerometer_2];
        mq_telem.gyro = [T.gyro_0,T.gyro_1,T.gyro_2];
    end

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
