% takes quad class, world_params, and obstacle position+size, and returns a desired state
function des_state = RoussosTrajectory(Q,world_params,obstacles)
    norm2 = @(u) u'*u; % squared norm of vector u 
    n = Q.state(1:3); % current position
    ndot = Q.state(4:6); % current velocity
    if norm2(Q.params.n_goal - n) <= .3^2 % deadband region (.3 m is a good deadband for a radius=2m flight space. 1m is good for a 10m flight space)
        ndot_d = [0;0;0]; % desired vel
        n_d = Q.params.n_goal; % desired pos
    else
        % computes cost at the current quad position 
        Phi = @(n) RoussosNavigationFunction(n,Q,obstacles,world_params); % must be anonymous function to calculate gradient
        gradient = grad(Phi,n,world_params.ds); % computes gradient using numerical differentiation
       
        p = .85; scale = 2; % constants for optimization method

        % optimization method
        ndot_d = scale*(-p*gradient/norm(gradient) + (1 - p)*(Q.params.n_goal - n)); % scale * (weighted average of (negative of normalized gradient) and (displacement from n_goal))
        for k = 1:3
            ndot_d(k) = bound(-2,2,ndot_d(k)); % bound desired velocity between -2 and 2 m/s
        end
        n_d = n + Q.params.dt*(ndot_d - ndot); % calculate desired position based on desired vel
    end
    des_state = [n_d' ndot_d' 0 0 0 0 0 0]';
end
