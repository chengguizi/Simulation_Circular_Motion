figure
subplot(311)
plot(truth(:,1),x_v_corrected(:,3),'Linewidth',2.5);
hold on
plot(truth(:,1),x_v_corrected_const(:,3),'r','Linewidth',2);
hold on
plot(truth(:,1),x_naive(:,3),'color',[1 .6 .0],'Linewidth',2);
plot(truth(:,1),truth(:,4),'--','color',[.5 .5 .5],'Linewidth',1.5);
plot(truth(:,1),v(:,1),'x','Color','k','MarkerSize',5);
title('[V-Correct] Output Velocity');
legend('EKF State V_x','EKF Const V_x','EKF Prediction Only V_x','Ground Truth V_x');
%legend('EKF Const V_x','EKF Prediction Only V_x','Ground Truth V_x');
ylim([-2.5 2.5]);

subplot(312)
plot(truth(:,1),x_v_corrected(:,5));
hold on
plot(truth(:,1),truth(:,6),'--','color',[.5 .5 .5],'Linewidth',1.5)
title('[V-Correct] EKF Input Acceleration');
legend('EKF Input A_x','Ground Truth A_x')
ylim([-1.5 1.5]);

subplot(313)
plot(truth(:,1),knoise3(:,1));
hold on
plot(truth(:,1),knoise3(:,3));
title('[V-Correct] K gain of EKF');
legend('K of Position','K of Velocity');
    

figure

plot(x_v_corrected_const(:,1),x_v_corrected_const(:,2),'r:','Linewidth',2.5);
hold on
plot(x_naive(:,1),x_naive(:,2),'color',[1 .6 .0],'Linewidth',2.5);
plot(truth(:,2),truth(:,3),'--','color',[.5 .5 .5],'Linewidth',2);
title('[V-Correct] Output Position');
legend('FULL EKF, Const P','EKF Prediction Only','Ground Truth')
grid

figure
plot(x_v_corrected(:,1),x_v_corrected(:,2),'Linewidth',3);
hold on
plot(x_v_corrected_const(:,1),x_v_corrected_const(:,2),':','Linewidth',2.5);
plot(x_naive(:,1),x_naive(:,2),'Linewidth',2.5);
plot(truth(:,2),truth(:,3),'--','color',[.5 .5 .5],'Linewidth',2);
title('[V-Correct] Output Position');
legend('Full EKF','FULL EKF, Const P','EKF Prediction Only','Ground Truth')
grid



 figure
 subplot(211)
 plot(truth(:,1),x_v_corrected_const(:,1),'Linewidth',2);
 hold on
 plot(truth(:,1),truth(:,2),'Linewidth',1.5);
 title('[V-Correct] Output Position P_x');
 subplot(212)
 plot(truth(:,1),x_v_corrected_const(:,2),'Linewidth',2);
 hold on
 plot(truth(:,1),truth(:,3),'Linewidth',1.5);
 title('[V-Correct] Output Position P_y');
