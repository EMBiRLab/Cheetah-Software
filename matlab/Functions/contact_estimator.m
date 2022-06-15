function estimate = contact_estimator(data_t, progress, rob_mass_kg)
    estimate = zeros(1,4);

    % threshold standing grf by applying more than 20% body mass
    threshold = (rob_mass_kg * 9.81) * 0.2;
    
    % for the progress signal, ignore foot data if we are in middle 50% of
    % swing phase. (That means .075 to .225 of the progress signal because 
    % of our specific test gait) 
    for leg = 1:4
        if (progress(leg) < .075 || progress(leg) > 0.225)
            %we dont ignore leg
            leg_grf = data_t.grf_est(3*(leg-1)+1:3*(leg-1)+3);
            if(leg_grf(3) < threshold)
                estimate(leg) = 1;
            end
        end
    end
end