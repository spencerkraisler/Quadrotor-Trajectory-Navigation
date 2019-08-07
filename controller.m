function [U1 U2 U3 U4] = controller(Q)
    s = Q.measured_state; d = Q.des_state;
    Kp_x = .008; Kd_x = .016; 
    Kp_y = .007; Kd_y = .02;
    Kp_z = 15; Kd_z = 15;
    
    Kp_phi = .16; Kd_phi = .009; 
    Kp_theta = .09; Kd_theta = .009; 
    Kp_yaw = .02; Kd_yaw = .005;
    
    gains = [Kp_x Kp_y Kp_z Kd_x Kd_y Kd_z Kp_phi Kp_theta Kp_yaw Kd_phi Kd_theta Kd_yaw];
    yawRotate = @(s) [cos(s(9))*s(1)+sin(s(9))*s(2) -sin(s(9))*s(1)+cos(s(9))*s(2) s(3) cos(s(9))*s(4)+sin(s(9))*s(5)+s(12)*(cos(s(9))*s(2)-sin(s(9))*s(1)) cos(s(9))*s(5)-sin(s(9))*s(4)-s(12)*(sin(s(9))*s(2)+cos(s(9))*s(1)) s(6) s(7) s(8) s(9) s(10) s(11) s(12)]';
    s_rot = yawRotate(s);
    d_rot = yawRotate(d);
    
    U1 = min(3,max(0, -gains(3) * (s(3) - d(3)) - gains(6) * (s(6) - d(6)) + Q.params.mass*Q.params.g));
    U2 = gains(2) * (s_rot(2) - d_rot(2)) + gains(5) * (s_rot(5) - d_rot(5)) - gains(7) * (s_rot(7) - d_rot(7)) - gains(10) * (s_rot(10) - d_rot(10));
    U3 = -gains(1) * (s_rot(1) - d_rot(1)) - gains(4) * (s_rot(4) - d_rot(4)) - gains(8) * (s_rot(8) - d_rot(8)) - gains(11) * (s_rot(11) - d_rot(11));
    U4 = -gains(9) * (s_rot(9) - d_rot(9)) - gains(12) * (s_rot(12) - d_rot(12));
end