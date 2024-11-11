function [e_y,pi_p] = cross_track_error(xk1,yk1,xk,yk,x_pos,y_pos)

    pi_p = atan2(yk1 - yk, xk1 - xk);
    
    R_np = [cos(pi_p), -sin(pi_p);
            sin(pi_p), cos(pi_p)];

    position_vector = [x_pos - xk; y_pos - yk];

    r = R_np' * position_vector;

    e_y = r(2);
end

