function [xk1,yk1,xk,yk, last] = WP_selector(x_pos,y_pos)
    persistent wp_index;
    persistent waypoints

    if isempty(wp_index)
        wp_index = 1;
    end

    if isempty(waypoints)
        waypoints = load('WP.mat').WP;
    end

    last = 0;

    current_wp = waypoints(:,wp_index);
    next_wp = waypoints(:,wp_index+1);

    current_pos = [x_pos;y_pos];

    distance_to_next = next_wp - current_pos;

    error = distance_to_next' * distance_to_next;

    % fprintf("Index: %d, error: %d \n",wp_index,error)

    if error < (161*2)^2 % two ship lengths
        if wp_index < (size(waypoints,2)-1)
            wp_index = wp_index + 1;
            current_wp = next_wp;
            next_wp = waypoints(:,wp_index+1);
        else
            last = 1;
        end
    end

    xk1 = next_wp(1);
    yk1 = next_wp(2);
    xk = current_wp(1);
    yk = current_wp(2);




end

