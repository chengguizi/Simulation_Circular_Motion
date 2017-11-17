function [anoise, pnoise, vnoise, truth] = simulate(time_span, time_step, avar, pvar, vvar)

        % ground truth parameter: c = (xc,yc); r = r ; theta = 0 ; omega = w
        % ground truth state = [t, px, py, vx, vy, theta, omega]
        % measurement state = [t, vx, vy]

        N = fix (time_span / time_step) + 1;

        param = [];
        param.c = [5;0];
        param.r = 5;
        param.phi = -pi;
        param.omega = -2*pi/time_span; % rad/s, constant
    
    disp('Generating Ground Truth');
    
    truth = zeros(N,7);

    for i = 1:N
        t = (i-1)*time_step;
        angle = param.omega*t + param.phi;
        p = param.c + param.r*[cos(angle);sin(angle)];
        v = param.r * param.omega * [-sin(angle);cos(angle)];
        a = param.r * param.omega^2 * [-cos(angle);-sin(angle)];
        truth(i,:) = [t , p' , v' , a'];  
    end
    
    %assert (isequal (truth(1,2:3),truth(end,2:3)), 'ERROR: Circle is not complete'); % end point is the same as the starting 
    

    disp('Generating Simulated Measurement');

    anoise = truth(:,6:7);
    noise = avar*randn(N,2);
    anoise = anoise + noise - 0.02;
    
    pnoise = truth(:,2:3);
    noise = pvar*randn(N,2);
    pnoise = pnoise + noise;
    
    vnoise = truth(:,4:5);
    noise = vvar*randn(N,2);
    vnoise = vnoise + noise;


    disp('Simulation Done');
    
    
end