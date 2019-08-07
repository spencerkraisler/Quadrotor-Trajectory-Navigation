% calculates metric of how close quad is to an obstacle (returns 0 if quad is
% touching an obstacle)
function G = ObstacleProximity(n,Q,obstacles)
    norm2 = @(u) u'*u;
    g = @(n,obs) norm2(n - obs(1:3)) - (Q.params.radius + obs(4))^2;
    num_of_obstacles = size(obstacles,1);
    G = 1;
    for k = 1:num_of_obstacles
        obstacle = obstacles(k,:)';
        g_k = g(n,obstacle);
        G = G*g_k;
    end
end