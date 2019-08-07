clear; close all; clc

rand_uni = @(a,b) a + (b - a)*rand();

% quad init
quadEOMhandle = @quadEOM;
controllerhandle = @controller;
init_state = [-.5 -.5 0 0 0 0 0 0 0 0 0 0]';
params = InitQuadParams();
Q = Quad(params,init_state);

% world constants
world_params = InitWorldParams();

% obstacles 
rand_obstacle = @() [rand_uni(-world_params.radius,world_params.radius) rand_uni(-world_params.radius,world_params.radius) rand_uni(0,world_params.radius) rand_uni(.1,.3)];
obstacles = [rand_obstacle(); rand_obstacle(); rand_obstacle()];

for t = 1:Q.params.max_iter - 1
    Q.UpdateDesiredQuadState(world_params,obstacles);
    Q.UpdateQuadState(quadEOMhandle,controllerhandle);
    Q.UpdateMeasuredQuadState();
    Q.UpdateQuadHistory();
end

% plotting
hold on
plotObstacles(obstacles) % plots obstacles
plot3(Q.params.n_goal(1),Q.params.n_goal(2),Q.params.n_goal(3),'go','MarkerFaceColor','g') % plots goal
grid
axis([-world_params.radius world_params.radius -world_params.radius world_params.radius 0 world_params.radius])
plotquad(Q,5)
hold off 

% plotRoussosTrajectory(Q,world_params,obstacles,10,.1)

