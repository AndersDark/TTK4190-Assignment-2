function chi_d = LOS_guidance(pi_p,y_e)

look_ahead_lenght = 161*10;

chi_d = pi_p * atan2(y_e, look_ahead_lenght);

end

