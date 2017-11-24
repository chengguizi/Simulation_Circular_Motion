function [anoise, pnoise, vnoise, truth] = simulate(time_span, time_step, avar, pvar, vvar)

        % ground truth parameter: c = (xc,yc); r = r ; theta = 0 ; omega = w
        % ground truth state = [t, px, py, vx, vy, theta, omega]
        % measurement state = [t, vx, vy]

        N = fix (time_span / time_step);

        param = [];
        param.c = [5;0];
        param.r = 5;
        param.barlength = 5;
        param.phi = -pi;
        param.omega = -2*pi/time_span; % rad/s, constant
    
    disp('Generating Ground Truth');
    
    %truth = zeros(N,7);
    t = 0;
    
    %%% first 1/4 circle
    for i = 1:N/4+1
        angle = param.omega*t + param.phi;
        p = param.c + param.r*[cos(angle);sin(angle)];
        v = param.r * param.omega * [-sin(angle);cos(angle)];
        a = param.r * param.omega^2 * [-cos(angle);-sin(angle)];
        truth(i,:) = [t , p' , v' , a'];
        t = t + time_step;
    end
    
    %%% horitonal bar
    v_bar = truth(end,4:5)';
    p1 = truth(end,2:3)';
    for i = 1:N/4
        p = truth(end,2:3)' + v_bar*time_step;
        v = v_bar;
        a = [0;0];
        truth(end+1,:) = [t , p' , v' , a'];
        t = t + time_step;
    end
    p_delta = truth(end,2:3)' - p1;
    
    %%% continous 1/2 circle
    for i = (N/4+2):(N*3/4+1)
        angle = param.omega*(i-1)*time_step + param.phi;
        p = p_delta + param.c + param.r*[cos(angle);sin(angle)];
        v = param.r * param.omega * [-sin(angle);cos(angle)];
        a = param.r * param.omega^2 * [-cos(angle);-sin(angle)];
        truth(end+1,:) = [t , p' , v' , a'];
        t = t + time_step;
    end
    
    %%% second horizontal bar
    v_bar = truth(end,4:5)';
    p1 = truth(end,2:3)';
    for i = 1:N/4
        p = truth(end,2:3)' + v_bar*time_step;
        v = v_bar;
        a = [0;0];
        truth(end+1,:) = [t , p' , v' , a'];
        t = t + time_step;
    end
    p_delta = truth(end,2:3)' - p1;
    
    %%% last 1/4 circle
    for i = (N*3/4+1):(N+1)
        angle = param.omega*(i)*time_step + param.phi;
        p = param.c + param.r*[cos(angle);sin(angle)];
        v = param.r * param.omega * [-sin(angle);cos(angle)];
        a = param.r * param.omega^2 * [-cos(angle);-sin(angle)];
        truth(end+1,:) = [t , p' , v' , a'];
        t = t + time_step;
    end
    
    N = size(truth,1);
    
    %assert (isequal (truth(1,2:3),truth(end,2:3)), 'ERROR: Circle is not complete'); % end point is the same as the starting 
    

    disp('Generating Simulated Measurement');
    
    rng(2);
    anoise = truth(:,6:7);
    noise = avar*randn(N,2);
    anoise = anoise + noise - 0.02;
    
    rng(3);
    pnoise = truth(:,2:3);
    noise = pvar*randn(N,2);
    pnoise = pnoise + noise;
    
    rng(4);
    vnoise = truth(:,4:5);
    noise = vvar*randn(N,2);
    vnoise = vnoise + noise;


    disp('Simulation Done');
    
    
end