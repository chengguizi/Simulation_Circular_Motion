close all; clear all; clc;
addpath('./ekf');

TIME_SPAN = 20;
TIME_STEP = 0.02;
[a1, p1, v1, truth] = simulate(TIME_SPAN,TIME_STEP,0.2,0.2,0.15);
[a2, p2 ] = simulate(TIME_SPAN,TIME_STEP,0.6,0.2,0.15);

N = size(truth,1);

% make acceleration more noisy at one point
gapA1 = fix(N*0.7);
gapA2 = fix(N*0.8);
a = [a1(1:gapA1,:);a2(gapA1+1:gapA2,:);a1(gapA2+1:end,:)];




p = p1;

%%% lower the update frequency of velocity
% make a gap between velocity readings
hasgapv = true;
gapV1 = fix(N*0.4);
gapV2 = fix(N*0.61);
v=zeros(size(v1));
for i=1:N
    if (mod(i,10) == 0 && (gapV1 > i || gapV2 < i  || ~hasgapv ))
        v(i,:) = v1(i,:);
    else
        v(i,:) = [NaN;NaN];
    end
end

% Update routine
dt = TIME_STEP;

% F matrix
F = [   1 0 dt 0  0  0 ;
        0 1 0  dt 0  0 ;
        0 0 1  0  0  0 ;
        0 0 0  1  0  0 ;
        0 0 0  0  0  0 ;
        0 0 0  0  0  0 ];
% B matrix    
B = [   0.5*dt^2,   0;
        0,          0.5*dt^2;
        dt,         0;
        0,          dt;
        1,          0;
        0,          1];


% Input Vectors
u = a;
u_truth = truth(:,6:7); % truth for acceleration

% State Vectors
x_p_corrected = zeros(N,6);
x_p_corrected(1,:) = truth(1,2:7);
x_v_corrected_const = zeros(N,6);
x_v_corrected_const(1,:) = truth(1,2:7);
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
Q = eye(6)*0.0005; % state noise
R = eye(2)*1;  % measurement noise

% H matrix  
H = [1, 0, 0, 0, 0, 0 ;
     0, 1, 0, 0, 0, 0];
Z = p;
P = ones(6,6); % initial covariance
% END of EKF Constant

knoise1 = zeros(N,6);
for i = 2:N
    [x_next, p_next, k] = EKFupdate(x_p_corrected(i-1,:)',P,Q,R,u(i-1,:)',Z(i,:)',F,B,H,true);
    knoise1(i,:) = k(:,1)';
    x_p_corrected(i,:) = x_next';
    P = p_next; 
end
correct_with_position_figure


%%% USE VELOCITY CORRECTION SIMPLE%%%
Q = diag(0.0001*[1 1 10 210 0 0]); % diag(0.04*[0.5*dt^2 0.5*dt^2 dt dt 1 1]); % state noise
R = eye(2);  % measurement noise

% H matrix  
H = [0, 0, 1, 0, 0, 0 ;
     0, 0, 0, 1, 0, 0];
Z = v;
P = eye(6); % initial covariance

for i = 2:N
    if ( isnan(Z(i,1)) )
        do_update = false;
    else
        do_update = true;
    end
    
    [x_next, p_next, k] = EKFupdate(x_v_corrected_const(i-1,:)',P,Q,R,u(i-1,:)',Z(i,:)',F,B,H,do_update);
    
    x_v_corrected_const(i,:) = x_next';
    if do_update
        P = p_next; 
    end
    
end


%%% USE VELOCITY CORRECTION ADVANCED%%%
%Q = diag(0.0001*[1 1 20 20 0 0]); % diag(0.04*[0.5*dt^2 0.5*dt^2 dt dt 1 1]); % state noise
R_ref = R;  % measurement noise
R_k = 1;

% H matrix  
H = [0, 0, 1, 0, 0, 0 ;
     0, 0, 0, 1, 0, 0];
Z = v;
P = eye(6); % initial covariance
knoise2 = zeros(N,6);
for i = 2:N
    if ( isnan(Z(i,1)) )
        do_update = false;
    else
        do_update = true;
    end
    
    [x_next, p_next, k] = EKFupdate(x_v_corrected(i-1,:)',P,Q,R_ref*R_k,u(i-1,:)',Z(i,:)',F,B,H,do_update);
    
    knoise2(i,:) = k(:,1)';
    x_v_corrected(i,:) = x_next';
    P = p_next; 
    
    if do_update
        R_k = max(1,R_k/1.5);
    else
        R_k = R_k*1.005;
    end
end

correct_with_velocity_figure

