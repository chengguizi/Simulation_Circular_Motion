figure
subplot(311)
plot(truth(:,1),x_p_corrected(:,3),'Linewidth',3);
hold on
plot(truth(:,1),x_naive(:,3),'Linewidth',2);
plot(truth(:,1),truth(:,4),'--','color',[.5 .5 .5],'Linewidth',2)
title('[P-Correct] Output Velocity');
legend('EKF State V_x','EKF Prediction Only V_x','Ground Truth V_x')
ylim([-2.5 2.5]);

subplot(312)
plot(truth(:,1),x_p_corrected(:,5));
hold on
plot(truth(:,1),truth(:,6),'--','color',[.5 .5 .5],'Linewidth',2)
title('[P-Correct] Input Acceleration');
legend('EKF Input A_x','Ground Truth A_x')
ylim([-1.5 1.5]);

subplot(313)
plot(truth(:,1),knoise1(:,1));
hold on
plot(truth(:,1),knoise1(:,3));
title('[P-Correct] K gain of EKF');
legend('K of Position','K of Velocity');
    

figure
plot(x_p_corrected(:,1),x_p_corrected(:,2),'Linewidth',3);
hold on
plot(x_naive(:,1),x_naive(:,2),':','Linewidth',2);
plot(truth(:,2),truth(:,3),'--','color',[.5 .5 .5],'Linewidth',2);
plot(p(:,1),p(:,2),'x','Color','k','MarkerSize',5);
title('[P-Correct] Output Position');
legend('Full EKF','EKF Prediction Only','Ground Truth','Position Measurment')
grid
