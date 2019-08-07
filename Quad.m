classdef Quad < handle
    properties (SetAccess = public)
        state;                    % current state
        measured_state;           % current measured state
        state_hist;               % state history
        measured_state_hist;      % measured state history
        des_state = zeros(12,1);  % desired state
        des_state_hist;           % desired state history
        params;                   % see InitQuadParams.m
        step = 1;                 % iterations since initialization
        last_action = [0 0 0 0];  % previous action (thrust, torque_x, torque_y, torque_z)
        sensors;                  % sensor data (sensors: accelerometer, gyroscope, GPS, compass)
    end
    
    properties (SetAccess = private)
        R = @(alpha) rotx(alpha(1)/pi*180) * roty(alpha(2)/pi*180) * rotz(alpha(3)/pi*180); % rotation matrix
        rotateToWorld = @(x,phi,theta,psi) (rotx(phi/pi*180) * roty(theta/pi*180) * rotz(psi/pi*180)) * x; % used when yaw is non-zero
    end
   
    methods
        
        % constructor
        function Q = Quad(params,state)
            Q.params = params;
            Q.state = state;   
            Q.state_hist = zeros(12,Q.params.max_iter);
            Q.des_state_hist = zeros(12,Q.params.max_iter);
            Q.sensors.accl_hist = zeros(3,Q.params.max_iter);
            Q.sensors.gyro_hist = zeros(3,Q.params.max_iter);
            Q.measured_state_hist = zeros(12,Q.params.max_iter);
            Q.UpdateMeasuredQuadState();
            Q.UpdateQuadHistory(); 
        end
        
        % takes the current state and control outputs
        % computes the next quad state
        function UpdateQuadState(Q,quadEOMhandle,controlhandle)
            sdot = quadEOMhandle(Q,controlhandle);
            Q.state = Q.state + Q.params.dt * sdot; % forward euler
        end
        
        % takes the current quad state and simulates sensors taking data 
        % to produce the current measured state
        % uncomment noise if you wish to introduce it
        % note: noise parameters are based on measured noise parameters from actual quadcopter 
        function UpdateMeasuredQuadState(Q)
            
            % accelerometer
            accl = Q.R(Q.state(7:9))'*[0 0 -Q.params.g]';
            % accl = accl + [1.647; 1.785; .517]*9.81.*randn(3,1); % accelerometer noise
            Q.sensors.accl_hist(:,Q.step) = accl;
            
            % gyroscope
            gyro = Q.state(10:12);
            % gyro = gyro + [14.74; 10.1; 10.27]/180*pi.*randn(3,1); % gyroscope noise
            Q.sensors.gyro_hist(:,Q.step) = gyro;
            
            % note: I did not include a GPS or compass state hist. 
            % gps system
            gps = Q.state(1:3);
            % gps = gps + [.1;.1;.1]/1000.*rand(3,1); % gps noise
            
            % compass
            compass = Q.state(9); 
            % compass = compass + .024/180*pi*rand(1); % compass noise
            
            if Q.step > 1  
                
                % low pass filter on accl and gyro
                accl = (1 - .04)*accl + .04*Q.sensors.accl_hist(:,Q.step-1);
                gyro = (1 - .5)*gyro + .5*Q.sensors.gyro_hist(:,Q.step-1);
                
                % predicting attitude (alpha) from accelerometer and
                % gyroscope data
                last_measured_alpha = Q.measured_state_hist(7:9,Q.step-1);
                measured_alpha_gyro = last_measured_alpha + Q.params.dt*gyro;
                measured_alpha_accl = zeros(2,1);
                measured_alpha_accl(1) = atan(accl(2)/accl(3));
                measured_alpha_accl(2) = -atan(-accl(1)/sqrt(accl(2)^2 + accl(3)^2));
                measured_alpha = (1 - .0075)*measured_alpha_gyro(1:2) + .0075*measured_alpha_accl; % weighted average of alpha from gyro and accl 
                measured_alpha(3) = compass;
                
                % predicting velocity from gps data
                last_measured_pos = Q.measured_state_hist(1:3,Q.step-1);
                measured_vel = (gps - last_measured_pos)/Q.params.dt;
                last_measured_vel = Q.measured_state_hist(4:6,Q.step-1);
                measured_vel = (1 - .65)*measured_vel + .65*last_measured_vel; % lpf on measured velocity             
                
                Q.measured_state = [gps' measured_vel' measured_alpha' gyro']';
            else
                % if this is the first simulation iteration
                Q.measured_state = zeros(12,1);
                Q.measured_state(1:3) = gps; 
                measured_alpha_accl = zeros(2,1);
                measured_alpha_accl(1) = atan(accl(2)/accl(3));
                measured_alpha_accl(2) = atan(-accl(1)/sqrt(accl(2)^2 + accl(3)^2));
                Q.measured_state(4:6) = [0;0;0];
                Q.measured_state(7:8) = measured_alpha_accl;
                Q.measured_state(9) = compass;
                Q.measured_state(10:12) = gyro;
            end
        end

        % takes the current quad position, world parameters,
        % and obstacle position+size and computes the desired
        % state of the quad
        function UpdateDesiredQuadState(Q,world_params,obstacles)
            Q.des_state = RoussosTrajectory(Q,world_params,obstacles);
        end
        
        % updating histories
        function UpdateQuadHistory(Q)
            Q.state_hist(:,Q.step) = Q.state;
            Q.measured_state_hist(:,Q.step) = Q.measured_state;
            Q.des_state_hist(:,Q.step) = Q.des_state;
            Q.step = Q.step + 1;
        end
    end
end