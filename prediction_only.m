function output_state = prediction_only(y,N,dt,truth,F,B)
    u = y;
    u_truth = truth(:,6:7);

%     figure
%     subplot(211);
%     plot(truth(:,1),u(:,1));
%     hold on
%     plot(truth(:,1),u(:,2));
%     title('Generated Noisy IMU');
%     legend('a_x','a_y');
%     ylim( [-0.5 0.5]);
% 
%     subplot(212);
%     plot(truth(:,1),u_truth(:,1));
%     hold on
%     plot(truth(:,1),u_truth(:,2));
%     title('Generated IMU Ground Truth');
%     legend('a_x','a_y');
%     ylim( [-0.5 0.5]);



    x = zeros(size(y,1),6);
    x_truth = zeros(size(y,1),6);

    x(1,:) = truth(1,2:7);
    x_truth(1,:) = truth(1,2:7);

    for i = 2:N
        x_truth(i,:) = F*x_truth(i-1,:)' + B*u_truth(i-1,:)';
    end

    for i = 2:N
        x(i,:) = F*x(i-1,:)' + B*u(i-1,:)';
    end
    
    output_state = x;

    % figure
    % plot(x(:,1),x(:,2));
    % hold on
    % plot(x_truth(:,1),x_truth(:,2));
    % plot(truth(:,2),truth(:,3),'--')
    % title('Output Position');
    % legend('Integrate over noisy input','Integrate over true input','Ground Truth')
    % 
    % figure
    % plot(truth(:,1),x(:,3));
    % hold on
    % plot(truth(:,1), x_truth(:,3));
    % plot(truth(:,1),truth(:,4),'--')
    % title('Output Velocity');
    % legend('Integrate over noisy input','Integrate over true input','Ground Truth')
    % 
    % figure
    % plot(truth(:,1),x(:,5));
    % hold on
    % plot(truth(:,1), x_truth(:,5));
    % plot(truth(:,1),truth(:,6),'--')
    % title('Output Acceleration');
    % legend('Integrate over noisy input','Integrate over true input','Ground Truth')
end
    