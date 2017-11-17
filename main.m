close all; clear; clc;
addpath('./ekf');

TIME_SPAN = 20;
TIME_STEP = 0.05;
[a1, p1, v1, truth] = simulate(TIME_SPAN,TIME_STEP,0.075,0.2,0.2);
[a2, p2 ] = simulate(TIME_SPAN,TIME_STEP,0.3,0.2,0.2);

N = size(truth,1);
N1 = fix(N*0.5);
N2 = fix(N*0.7);
a = [a1(1:N1,:);a2(N1+1:N2,:);a1(N2+1:end,:)];
p = p1;

%%% lower the update frequency of velocity
v=zeros(size(v1));
for i=1:N
    if (mod(i,5) == 0)
        v(i,:) = v1(i,:);
    else
        v(i,:) = [NaN;NaN];
    end
end

% Update routine
dt = TIME_STEP;

% F matrix
F = [   1 0 dt 0  0.5*dt^2  0 ;
        0 1 0  dt 0  0.5*dt^2 ;
        0 0 1  0  dt 0 ;
        0 0 0  1  0  dt ;
        0 0 0  0  0  0 ;
        0 0 0  0  0  0 ];
% B matrix    
B = [zeros(4,2);eye(2)];


% Input Vectors
u = a;
u_truth = truth(:,6:7); % truth for acceleration

% State Vectors
x_p_corrected = zeros(N,6);
x_p_corrected(1,:) = truth(1,2:7);
x_v_corrected = zeros(N,6);
x_v_corrected(1,:) = truth(1,2:7);
% x_truth = zeros(N,6);
% x_truth(1,:) = truth(1,2:7);



figure
subplot(411);
plot(truth(:,1),u(:,1));
hold on
plot(truth(:,1),u(:,2));
title('Generated Noisy IMU Acceleration Input (w/ Drift)');
ylim([-1.5 1.5]);
legend('a_x','a_y');

subplot(412);
plot(truth(:,1),v(:,1),'.');
hold on
plot(truth(:,1),v(:,2),'.');
plot(truth(:,1),truth(:,4),'--');
plot(truth(:,1),truth(:,5),'--');
title('Generated Noisy Velocity Measurement');
ylim([-2.5 2.5]);
legend('v_x','v_y');

subplot(4,1,[3,4]);
plot(p(:,1),p(:,2));
hold on
plot(truth(:,2),truth(:,3),'--');
title('Generated Noisy Position Measurement');
grid


%%% USE PREDICTION ONLY APPROACH %%%
x_naive = prediction_only(a,N,dt,truth,F,B);



%%% USE POSITION CORRECTION %%%

% EKF Constant
Q = eye(6)*0.001; % state noise
R = eye(2)*1;  % measurement noise

% H matrix  
H = [1, 0, 0, 0, 0, 0 ;
     0, 1, 0, 0, 0, 0];
Z = p;
P = eye(6); % initial covariance
% END of EKF Constant

knoise1 = zeros(N,6);
for i = 2:N
    [x_next, p_next, k] = EKFupdate(x_p_corrected(i-1,:)',P,Q,R,u(i-1,:)',Z(i-1,:)',F,B,H,true);
    knoise1(i,:) = k(:,1)';
    x_p_corrected(i,:) = x_next';
    P = p_next; 
end
correct_with_position_figure


%%% USE VELOCITY CORRECTION %%%
Q = diag(0.001*[1 1 20 20 1 1]); % state noise
R = eye(2)*1;  % measurement noise

% H matrix  
H = [0, 0, 1, 0, 0, 0 ;
     0, 0, 0, 1, 0, 0];
Z = v;
P = eye(6)*0.2; % initial covariance
knoise2 = zeros(N,6);
for i = 2:N
    if ( isnan(Z(i-1,1)) )
        do_update = false;
    else
        do_update = true;
    end
    [x_next, p_next, k] = EKFupdate(x_v_corrected(i-1,:)',P,Q,R,u(i-1,:)',Z(i-1,:)',F,B,H,do_update);
    
    if (do_update == true)
        knoise2(i,:) = k(:,1)';
    else
        knoise2(i,:) = knoise2(i-1,:);
    end
    x_v_corrected(i,:) = x_next';
    P = p_next; 
end

correct_with_velocity_figure

