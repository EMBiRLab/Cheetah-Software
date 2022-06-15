function data_struct = get_mq_data(mq_telem, t)
    data_struct = struct;
    data_struct.quat = mq_telem.quat(t,:);
    data_struct.leg0_foot_pos = mq_telem.leg0_foot_pos(t,:);
    data_struct.leg1_foot_pos = mq_telem.leg1_foot_pos(t,:);
    data_struct.leg2_foot_pos = mq_telem.leg2_foot_pos(t,:);
    data_struct.leg3_foot_pos = mq_telem.leg3_foot_pos(t,:);
%     data_struct.leg0_tau_est = mq_telem.leg0_tau_est(t,:);
%     data_struct.leg1_tau_est = mq_telem.leg1_tau_est(t,:);
%     data_struct.leg2_tau_est = mq_telem.leg2_tau_est(t,:);
%     data_struct.leg3_tau_est = mq_telem.leg3_tau_est(t,:);
    data_struct.grf_est = [mq_telem.leg0_grf_est(t,:),mq_telem.leg1_grf_est(t,:),mq_telem.leg2_grf_est(t,:),mq_telem.leg3_grf_est(t,:)];
%     data_struct.leg1_grf_est = mq_telem.leg1_grf_est(t,:);
%     data_struct.leg2_grf_est = mq_telem.leg2_grf_est(t,:);
%     data_struct.leg3_grf_est = mq_telem.leg3_grf_est(t,:);
end