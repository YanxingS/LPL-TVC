function boundedRegion(~,~)
    [xRange, yRange, radius, yaw_max, yaw_min] = config();
    % Plot the bounded region as a circle
    theta = linspace(0, 2*pi, 100);
    x = radius * cos(theta);
    y = radius * sin(theta);
    plot(x, y, 'r', 'LineWidth', 2);
    
    % Add a small crossing "+" at the origin
    line([-0.05 0.05], [0 0], 'Color', 'k', 'LineWidth', 1);
    line([0 0], [-0.05 0.05], 'Color', 'k', 'LineWidth', 1);
end