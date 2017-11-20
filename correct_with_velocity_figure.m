figure
subplot(311)
plot(truth(:,1),x_v_corrected(:,3));
hold on
plot(truth(:,1),x_naive(:,3));
%plot(truth(:,1),x_v_corrected(:,4));
plot(truth(:,1),truth(:,4),'--');
%plot(truth(:,1),truth(:,5),'--');
plot(truth(:,1),v(:,1),'.');
%plot(truth(:,1),v(:,2),'.');
title('[V-Correct] Output Velocity');
legend('EKF State V_x','EKF Prediction Only V_x','Ground Truth V_x');
ylim([-2.5 2.5]);

subplot(312)
plot(truth(:,1),x_v_corrected(:,5));
hold on
plot(truth(:,1),truth(:,6),'--')
title('[V-Correct] EKF Input Acceleration');
legend('EKF Input A_x','Ground Truth A_x')
ylim([-1.5 1.5]);

subplot(313)
plot(truth(:,1),knoise2(:,1));
hold on
plot(truth(:,1),knoise2(:,3));
title('[V-Correct] K gain of EKF');
legend('K of Position','K of Velocity');
    

figure
plot(x_v_corrected(:,1),x_v_corrected(:,2));
hold on
plot(x_naive(:,1),x_naive(:,2));
plot(truth(:,2),truth(:,3),'--');
title('[V-Correct] Output Position');
legend('Full EKF','EKF Prediction Only','Ground Truth')
grid

 figure
 plot(truth(:,1),x_v_corrected(:,1));
 hold on
 plot(truth(:,1),truth(:,2));
 title('[V-Correct] Output Position P_x');
