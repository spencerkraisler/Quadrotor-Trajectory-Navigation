% takes the quadrotor state and controller output of quadrotor
% and computes the state time derivative
% n = position
% alpha = attitude (euler angles)
function sdot = quadEOM(Q,controlhandle)
    % update actions on every control update
    % otherwise, use the last action
    if mod(Q.step, Q.params.control_response_time/Q.params.dt) == 0
        [U1 U2 U3 U4] = controlhandle(Q);
        Q.last_action = [U1 U2 U3 U4];
    else
        U = Q.last_action;
        U1 = U(1); U2 = U(2); U3 = U(3); U4 = U(4);
    end
    
    R = Q.R(Q.state(7:9)); % rotation matrix
    f_ndot = U1/Q.params.mass*R*[0;0;1] - Q.params.g*[0;0;1]; % n'' = F/m*R*e3 - g*e3
    f_alphadot = cross(-Q.state(10:12),Q.state(10:12)) + Q.params.I\[U2;U3;U4]; % alpha'' = -alpha' x alpha + inv(I)*T
    
    sdot = [Q.state(4:6)' f_ndot' Q.state(10:12)' f_alphadot']';
    
end