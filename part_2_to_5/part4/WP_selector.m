function [xk1,yk1,xk,yk] = WP_selector(x_pos,y_pos)
    persistent wp_index;
    persistent waypoints

    if isempty(wp_index)
        wp_index = 1;
    end

    if isempty(waypoints)
        waypoints = load('WP.mat').WP;
    end

    current_wp = waypoints(:,wp_index);
    next_wp = waypoints(:,wp_index+1);

    current_pos = [x_pos;y_pos];

    distance_to_next = next_wp - current_pos;

    error = distance_to_next' * distance_to_next;

    if error < 161*2 % two ship lengths
        if wp_index < (size(waypoints,2)-1)
            wp_index = wp_index + 1;
            fprintf("Index: %d, x_pos: %d, y_pos: %d", wp_index, x_pos, y_pos)
            current_wp = next_wp;
            next_wp = waypoints(:,wp_index+1);
        end
    end

    xk1 = next_wp(1);
    yk1 = next_wp(2);
    xk = current_wp(1);
    yk = current_wp(2);




end

