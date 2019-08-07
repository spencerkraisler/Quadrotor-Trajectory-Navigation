% plot the actual Roussos cost function over time 
function plotRoussosTrajectory(Q,params,obs,speed,resolution)
    Phi = @(n) RoussosNavigationFunction(n,Q,obs,params);
    bound = params.radius;
    span = -bound:resolution:bound;
    [X Y] = meshgrid(span,span);
    len = size(X,1);
    Z = ones(len);
    for x = 1:len
        for y = 1:len
            n = [X(y,x) Y(y,x) 0]';
            if isreal(Phi(n))
                Z(y,x) = min(1,Phi(n));
            end
            
        end
    end

    hold on
    Phi_goal = Phi([Q.params.n_goal(1) Q.params.n_goal(2) 0]');
    g = plot3(Q.params.n_goal(1),Q.params.n_goal(2),Phi_goal,'go','MarkerFaceColor','g');
    q = plot3(Q.state_hist(1,1),Q.state_hist(2,1),Q.state_hist(3,1),'wo','MarkerFaceColor','w');
    h = surf(X,Y,Z);
    colormap jet
    xlabel('x');ylabel('y');zlabel('cost');
    axis([-params.radius params.radius -params.radius params.radius 0 1.01])
    grid
    for j = 1:Q.params.max_iter-1
        if mod(j,speed) == 0
            Z = ones(len);
            for x = 1:len
                for y = 1:len
                    n = [X(y,x) Y(y,x) Q.state_hist(3,j)]';
                    if isreal(Phi(n))
                        cost = min(1,Phi(n));
                        Z(y,x) = cost;
                    end
                end
            end
            Phi_goal = Phi([Q.params.n_goal(1) Q.params.n_goal(2) Q.state_hist(3,j)]');
            set(h,'Zdata',Z);
            set(g,'Zdata',Phi_goal + .01);
            Phi_z = Phi(Q.state_hist(1:3,j));
            set(q,'XData',Q.state_hist(1,j),'YData',Q.state_hist(2,j),'ZData',Phi_z + .01);
            title(sprintf('time = %.2f sec',j/Q.params.max_iter*50));
            drawnow
        end
    end
end

