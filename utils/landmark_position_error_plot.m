function landmark_position_error_plot(Robots, profile)
    LandmarksGT = profile.info.Landmarks(:,2:3)';
    f3 = figure;
    title("Landmark Position RMSE")
    subtitle("Ground Truth vs Estimated")
    grid minor;
    hold on;
    maxY = zeros(numel(Robots),1);
    for i = 1:numel(Robots)
        LHistory = Robots{i}.LHistory;
        [~,~,iters] = size(LHistory);
        LPositionRMSE = zeros(iters,1);
        for j = 1:iters
            mask = ~isnan(LHistory(1,:,j));
            den  = sum(mask);
            if den == 0
                continue;
            end
            x1 = LandmarksGT(:, mask);
            x2 = LHistory(:, mask, j);
            [R,t] = find_transformation(x1, x2);
            x1 = R * x1 + t;
            
            dtran = sqrt(sum((x1 - x2).^2,1));
            LPositionRMSE(j) = sqrt(sum(dtran.^2) / den);
        end
        maxY(i) = max(LPositionRMSE);
        plot(Robots{i}.G(:,1), LPositionRMSE, 'LineWidth', 1.5)
    end
    
%     yl = ceil(max(maxY) / 0.1) * 0.1;
    yl = 0.6;
    legend_cell_array = 1:numel(Robots);
    ylim([0,yl])
    figure(f3);
    legend(strcat('Robot ', num2str(legend_cell_array')))
    xlabel('Simulation time [s]')
    ylabel('Landmark Position RMSE [m]')
    hold off;
    set(gcf,'units','centimeters','position',[0,0,8.8,8.8]); 
    set(gca,'FontSize',10,'FontName','Times');
    if profile.sim.use_distributed
        print(strcat('images/', profile.info.dataset, '_landmarksRMSE'),'-depsc2');
    else
        print(strcat('images/', profile.info.dataset, '_EKFlandmarksRMSE'),'-depsc2');    
    end
    
end