function pose_error_plot(Robots, profile)

    if flag
        legend_cell_array = 1:numel(Robots);
        figure(1)
        title("Relative translation RMSE")
        subtitle("Ground Truth vs Estimated")
        grid minor;
        hold on;

        figure(2)
        title("Relative rotation RMSE")
        subtitle("Ground Truth vs Estimated")
        grid minor;
        hold on;
    end
    
    for j = 1:numel(Robots)
        N = numel(Robots{j}.G(:,1));
        T_EST = zeros(3,3,N);
        T_GT  = zeros(3,3,N);
        cost_GT = cos(Robots{j}.G(:,4));
        sint_GT = sin(Robots{j}.G(:,4));
        cost_EST = cos(Robots{j}.Est(:,4));
        sint_EST = sin(Robots{j}.Est(:,4));
        T_GT(1,1,:) = cost_GT;
        T_GT(2,2,:) = cost_GT;
        T_GT(1,2,:) = -sint_GT;
        T_GT(2,1,:) = sint_GT;
        T_GT(1:2,3, :) = Robots{j}.G(:,2:3)';
        T_GT(3,3,:) = 1;

        T_EST(1,1,:) = cost_EST;
        T_EST(2,2,:) = cost_EST;
        T_EST(1,2,:) = -sint_EST;
        T_EST(2,1,:) = sint_EST;
        T_EST(1:2,3, :) = Robots{j}.Est(:,2:3)';
        T_EST(3,3,:) = 1;

        res = zeros(3,3,N-1);

        for i = 1 : N-1
            dij_GT =  T_GT(:,:,i+1)  / T_GT(:,:,i);
            dij_EST = T_EST(:,:,i+1) / T_EST(:,:,i);
            res(:,:,i) = dij_EST / dij_GT;
        end

        delta = zeros(2,N-1);
        delta(1,1) = norm(res(1:2,3,1))^2;
        delta(2,1) = atan2(res(2,1,1) , res(1,1,1))^2;

        for i = 2 : N - 1
            delta(1,i) = delta(1,i-1) + norm(res(1:2,3,i))^2;
            delta(2,i) = delta(2,i-1) + atan2(res(2,1,i) , res(1,1,i))^2;
        end

        mu_den = 1:N-1;
        delta(1,:) = sqrt(delta(1,:) ./ mu_den);
        delta(2,:) = sqrt(delta(2,:) ./ mu_den);

        if flag
            figure(1)
            plot(Robots{j}.G(1:end-1,1),delta(1,:), 'LineWidth', 1.5)

            figure(2)  
            plot(Robots{j}.G(1:end-1,1), delta(2,:), 'LineWidth', 1.5)
        end
        
        disp([delta(1,end), delta(2,end)]);
    end
    
    if flag
        figure(1)
        legend(strcat('Robot ', num2str(legend_cell_array')))
        xlabel('Simulation time [s]')
        ylabel('Translation RMSE [m]')
        hold off;
        set(gcf,'units','centimeters','position',[0,0,8.8,8.8]); 
        set(gca,'FontSize',10,'FontName','Times');
        if profile.sim.use_distributed
            print(strcat('images/', profile.info.dataset, '_translationRMSE'),'-depsc2');
        else
            print(strcat('images/', profile.info.dataset, '_EKFtranslationRMSE'),'-depsc2');
        end

        figure(2)
        legend(strcat('Robot ', num2str(legend_cell_array')))
        xlabel('Simulation time [s]')
        ylabel('Rotation RMSE [rad]')
        hold off;
        set(gcf,'units','centimeters','position',[0,0,8.8,8.8]); 
        set(gca,'FontSize',10,'FontName','Times');
        if profile.sim.use_distributed
            print(strcat('images/', profile.info.dataset, '_rotationRMSE'), '-depsc2');
        else
            print(strcat('images/', profile.info.dataset, '_EKFrotationRMSE'),'-depsc2');
        end
    end
end