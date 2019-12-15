% plots the sphereical obstacles in 3D space
% obstacles is a matrix [ob1_x ob1_y ob1_z ob1_radius; ob2_x ob2_y ob2_z ob2_radius; ...];
function plotObstacles(obstacles)
    num_of_obstacles = size(obstacles,1);
    for k = 1:num_of_obstacles
        obstacle = obstacles(k,:);
        [X Y Z] = sphere();
        X = obstacle(4)*X + obstacle(1); 
        Y = obstacle(4)*Y + obstacle(2); 
        Z = obstacle(4)*Z + obstacle(3);
        surf(X,Y,Z);
    end
end