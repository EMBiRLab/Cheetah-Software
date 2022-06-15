function progress = populate_progress(contact_states)
    progress = zeros(4,length(contact_states));
    assumed_contact = contact_states > 0;
    
    for leg = 1:4
        % the ends of the gaits
        gait_ends = find(diff(assumed_contact(:,leg)) < 0) + 1;
        for i = 1:size(gait_ends)-1
%             progress(i) = find(gait_ends > i, 1) - find(gait_ends <= i, 1, 'last');
%             cycle_start = find(gait_ends <= i, 1, 'last');
%             cycle_end = find(gait_ends > i, 1);
            cycle_start = gait_ends(i);
            cycle_end = gait_ends(i+1);
            
            if (cycle_end - cycle_start > 500)
                progress(leg,cycle_start:cycle_end) = 0;
            elseif( ~isempty( cycle_start ))
                progress(leg,cycle_start:cycle_end) = ((cycle_start:cycle_end) - cycle_start) / (cycle_end - cycle_start);
            else
                progress(leg,1:cycle_end) = 0;
            end
        end
    end
%     figure;
%     plot(progress(1,:))
end