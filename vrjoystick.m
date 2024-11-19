% Initialize joystick
joystickID = 1;  % Change if using a different joystick ID
try
    joy = vrjoystick(joystickID);
catch
    error('Joystick not detected or Simulink 3D Animation toolbox not installed.');
end

% Set up a figure for visualization
figure('Name', 'Joystick Visualization', 'NumberTitle', 'off');
axis([-1 1 -1 1]);  % Axes range for X and Y (normalized to joystick range)
hold on;
h = plot(0, 0, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');  % Plot initial point at the center

title('Joystick Movement');
xlabel('X Axis');
ylabel('Y Axis');
grid on;

% Start reading the joystick
disp('Move the joystick to control the point on the plot.');
disp('Press Ctrl+C in the Command Window to stop.');

try
    while true
        % Read the joystick axes and button states
        [axes, buttons] = read(joy);

        % Extract X and Y positions for left analog stick (adjust axes based on your joystick)
        x = axes(1); % X-axis (left/right) movement
        y = axes(2); % Y-axis (up/down) movement

        % Update plot position based on joystick movement
        set(h, 'XData', x, 'YData', y);
        
        % Display axes and button data in the Command Window
        disp(['Axes: [X: ', num2str(x), ', Y: ', num2str(y), ']']);
        disp(['Buttons: ', num2str(buttons)]);
        
        % Pause briefly for smoother movement
        pause(0.05);
    end
catch ME
    % Catch Ctrl+C interruption to safely close the joystick connection
    disp('Joystick visualization stopped.');
end

% Release joystick resources
clear joy;