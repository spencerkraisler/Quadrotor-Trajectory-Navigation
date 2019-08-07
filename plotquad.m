% powerful 3D visualizer to display quad position over time 
function plotquad(Q,speed)
    R = @(phi,theta,psi) rotx(phi/pi*180) * roty(theta/pi*180) * rotz(psi/pi*180); % rotation matrix
    r = Q.params.radius; % radius of quad
    N = Q.params.max_iter; % number of simulation iterations
    tend = N*Q.params.dt; % number of seconds simulation lasts
    phi = Q.state_hist(7,:);theta = Q.state_hist(8,:);psi = Q.state_hist(9,:); % roll, pitch, and yaw histories

    % body positions of the motors
    M1_body = [r; 0; 0] * ones(1,N); 
    M2_body = [0; -r; 0] * ones(1,N);
    M3_body = [-r; 0; 0] * ones(1,N);
    M4_body = [0; r; 0] * ones(1,N);
    M1 = zeros(3,N);M2 = zeros(3,N);M3 = zeros(3,N);M4 = zeros(3,N);

    % world positions of the motors
    for k = 1:N
        M1(:,k) = Q.state_hist(1:3,k) + R(phi(k),theta(k),psi(k))*M1_body(:,k);
        M2(:,k) = Q.state_hist(1:3,k) + R(phi(k),theta(k),psi(k))*M2_body(:,k);
        M3(:,k) = Q.state_hist(1:3,k) + R(phi(k),theta(k),psi(k))*M3_body(:,k);
        M4(:,k) = Q.state_hist(1:3,k) + R(phi(k),theta(k),psi(k))*M4_body(:,k);
    end

    % plot first point of quad path
    h = plot3(Q.state_hist(1,1),Q.state_hist(2,1),Q.state_hist(3,1),'r');
    xlabel('x (m)');ylabel('y (m)');zlabel('z (m)');
    title(sprintf('time %.f sec',1/N*tend));

    hold on

    % plot center of quad
    c = plot3(Q.state_hist(1,1),Q.state_hist(2,2),Q.state_hist(3,3),'bo','MarkerFaceColor','b');
    
    % plot line that connects center to motor 1 
    m1 = plot3([Q.state_hist(1,1); M1(1,1)],[Q.state_hist(2,1); M1(2,1)],[Q.state_hist(3,1); M1(3,1)],'k','LineWidth',2);
    % plot motor 1 as a dot
    m1dot = plot3(M1(1,1),M2(2,1),M3(3,1),'bo','MarkerFaceColor','r');
    m2 = plot3([Q.state_hist(1,1); M2(1,1)],[Q.state_hist(2,1); M2(2,1)],[Q.state_hist(3,1); M2(3,1)],'k','LineWidth',2);
    m2dot = plot3(M1(1,1),M2(2,1),M3(3,1),'bo','MarkerFaceColor','b');
    m3 = plot3([Q.state_hist(1,1); M3(1,1)],[Q.state_hist(2,1); M3(2,1)],[Q.state_hist(3,1); M3(3,1)],'k','LineWidth',2);
    m3dot = plot3(M1(1,1),M2(2,1),M3(3,1),'bo','MarkerFaceColor','b');
    m4 = plot3([Q.state_hist(1,1); M4(1,1)],[Q.state_hist(2,1); M4(2,1)],[Q.state_hist(3,1); M4(3,1)],'k','LineWidth',2);
    m4dot = plot3(M1(1,1),M2(2,1),M3(3,1),'bo','MarkerFaceColor','g');
    
    for t = 1:N-1
        if mod(t,2^speed) == 0
            % update values
            set(h,'XData',Q.state_hist(1,1:t),'YData',Q.state_hist(2,1:t),'ZData',Q.state_hist(3,1:t));
            title(sprintf('t = %.f sec',t/N*tend),'FontSize',15);
            set(c,'XData',Q.state_hist(1,t),'YData',Q.state_hist(2,t),'ZData',Q.state_hist(3,t))
            set(m1,'XData',[Q.state_hist(1,t); M1(1,t)],'YData',[Q.state_hist(2,t); M1(2,t)],'ZData',[Q.state_hist(3,t); M1(3,t)])
            set(m1dot,'XData',M1(1,t),'YData',M1(2,t),'ZData',M1(3,t))
            set(m2,'XData',[Q.state_hist(1,t); M2(1,t)],'YData',[Q.state_hist(2,t); M2(2,t)],'ZData',[Q.state_hist(3,t); M2(3,t)])
            set(m2dot,'XData',M2(1,t),'YData',M2(2,t),'ZData',M2(3,t))
            set(m3,'XData',[Q.state_hist(1,t); M3(1,t)],'YData',[Q.state_hist(2,t); M3(2,t)],'ZData',[Q.state_hist(3,t); M3(3,t)])
            set(m3dot,'XData',M3(1,t),'YData',M3(2,t),'ZData',M3(3,t))
            set(m4,'XData',[Q.state_hist(1,t); M4(1,t)],'YData',[Q.state_hist(2,t); M4(2,t)],'ZData',[Q.state_hist(3,t); M4(3,t)])
            set(m4dot,'XData',M4(1,t),'YData',M4(2,t),'ZData',M4(3,t))
            drawnow
        end
    end
end