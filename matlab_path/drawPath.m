
%------------------------------------------------------------------------------------
% The user interaction implementation references the following link:
%https://www.mathworks.com/help/matlab/creating_plots/capturing-mouse-clicks.html
%------------------------------------------------------------------------------------

function pathCoordinates = drawPath(fig)
    % Load configuration
    [xRange, yRange, radius, yaw_max, yaw_min] = config();

    % Ensure the figure is used
    figure(fig);
    axis equal;
    axis([xRange yRange]);
    hold on;
    xlabel('X-axis');
    ylabel('Y-axis');

    % Plot the bounded region as a circle
    boundedRegion(xRange, yRange);

    % Initialize an empty array to store the path coordinates
    pathCoordinates = [];
    h = plot(NaN, NaN, 'b'); % Initialize an empty plot

    % Set up callback functions for mouse actions
    set(fig, 'WindowButtonDownFcn', @startDrawing);
    set(fig, 'WindowButtonUpFcn', @stopDrawing);
    set(fig, 'KeyPressFcn', @keyPressCallback);

    % Nested function to handle mouse button press
    function startDrawing(~, ~)
        set(fig, 'WindowButtonMotionFcn', @drawPathMotion);
    end

    % Nested function to handle mouse button release
    function stopDrawing(~, ~)
        set(fig, 'WindowButtonMotionFcn', '');
    end

    % Nested function to handle mouse motion
    function drawPathMotion(~, ~)
        % Get the current point
        cp = get(gca, 'CurrentPoint');
        x = cp(1, 1);
        y = cp(1, 2);

        % Check if the point is within the circle
        if x^2 + y^2 <= radius^2
            % Append the new point to the path coordinates
            pathCoordinates = [pathCoordinates; x, y];

            % Update the plot
            set(h, 'XData', pathCoordinates(:, 1), 'YData', pathCoordinates(:, 2));
        end
    end

    % Nested function to handle key press
    function keyPressCallback(~, event)
        if strcmp(event.Key, 'return') % Check if the Enter key is pressed
            disp('Drawing ended');
            uiresume(fig); % Resume the UI to continue execution
        end
    end

    % Wait for the user to finish drawing
    uiwait(fig);
end