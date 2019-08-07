% inits world parameters
function world_params = InitWorldParams()
    world_params.radius = 2; % world radius
    world_params.epsilon = .01; % constant for Roussos trajectory
    world_params.k = 3; %3 constant for Roussos trajectroy (low => obstacles affect cost, high => goal affects cost)
    world_params.ds = .001; % discretization rate for calculating gradient of cost function
    % world_params.height = 2; % only used when world space is cylindrical 
end