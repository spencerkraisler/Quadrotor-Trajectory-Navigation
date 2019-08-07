% inits quad parameters
% measurments in terms of meters, kilograms, and seconds 
% measurments based on actual micro quadrotor
function params = InitQuadParams()
    J = @(alpha) [cos(alpha(3))*cos(alpha(2));sin(alpha(3))*cos(alpha(2));-sin(alpha(2))];
    params.dt = .001; % discretization rate for updating quad state
    tend = 20; params.max_iter = tend/params.dt; % number of iterations for simulation
    params.mass = .129; params.g = 9.81; params.radius = 75/1e3;
    Ixx = 44.897/1e6; Iyy = 48.764/1e6; Izz = 89.883/1e6; % moment of inertia values are small due to micro quadcopter
    params.I = [Ixx 0 0; 0 Iyy 0; 0 0 Izz];
    params.n_goal = [.6;.6;.6]; % position goal
    alpha_goal = [0;0;0]; % attitude goal
    params.J_Id = J(alpha_goal);
    params.control_response_time = .01; % discretization rate for control response
end