
function main()
    % Load configuration
   [xRange, yRange, radius, yaw_max, yaw_min] = config();

    % Initialize the drawing environment

    fig = figure('Name', 'Path Drawing', 'NumberTitle', 'off');
    figure(fig);
    axis equal;
    grid on;
    hold on;

    % Set up the bounded region as a circle
    boundedRegion(xRange, yRange);

    % Fix the axis limits
    xlim(xRange);
    ylim(yRange);

    % User interaction to draw the path
    pathCoordinates = drawPath(fig);

    % Print messages to the console
    disp('Drawing ended');
    disp('Start generating table');
    
    % Generate the table from the path coordinates
    generateTable(pathCoordinates, 'array.h');

    disp('Table generation finished');

    hold off;
end