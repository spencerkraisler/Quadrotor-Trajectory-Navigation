% calculates the Roussos cost given the current position, obstacle
% positions+sizes, and world parameters 
function cost = RoussosNavigationFunction(n,Q,obstacles,world_params)
    norm2 = @(u) u'*u; % squared norm of vector u 
    
    gamma_d = norm2(Q.params.n_goal - n); % values to 0 when quad is near goal point
    
    beta = world_params.radius^2 - norm2(n) - Q.params.radius^2; % values to 0 when quad is near world boundary
    % beta = (world_params.height - n(3))*n(3)*(world_params.radius^2 - norm2(n(1:2)) - Q.params.radius^2); % values to 0 when quad is near world boundary
    
    H = world_params.epsilon + norm2(Q.params.J_Id .* (n - Q.params.n_goal)); 
   
    G = ObstacleProximity(n,Q,obstacles); % values to 0 when quad is near obstacle
    
    cost = gamma_d/((gamma_d^world_params.k + H*G*beta)^(1/world_params.k)); % Roussos cost
end