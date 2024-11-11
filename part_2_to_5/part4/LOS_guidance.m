function chi_d = LOS_guidance(y_e, pi_p)

look_ahead_lenght = 161*15;

chi_d = pi_p - atan2(y_e, look_ahead_lenght);

% display(chi_d)

end

