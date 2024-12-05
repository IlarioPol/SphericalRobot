%% ------------------------------------------------------------------------
%% newRobot_with_Control.m
% Program to simulate the motion of a spherical robot that rolls without
% slipping on a flat surface - refactored for modularity.
% Original author: Simone Fiori (DII-UnivPM)
%% Included REAL-TIME PROCESSING instead of Simulated Trajectories
% Modified by: Francisco A. Camargo, M.Phil. (CS Columbia University) /FAC
% Refactored: <Your Name>
%% ------------------------------------------------------------------------   
    %% Kick-off
    clc; 
    close all; clear
    set(0,'defaultTextInterpreter','latex')
    set(0,'defaultAxesTickLabelInterpreter','latex')
    set(0,'defaultLegendInterpreter','latex')
    set(0,'defaultAxesFontSize',16)
    set(0,'defaultTextFontSize',16)
    set(0,'DefaultLineLineWidth',2)

    %% Mathematical operators
    I3 = eye(3); 
    ez = [0 0 1]';
    SY = [0 0 0; 0 0 -1; 0 1 0]; % Lie-algebra generator for the yoke angular velocity
    SP = [0 0 1; 0 0 0; -1 0 0]; % Lie-algebra generator for the pendulum angular velocity
    SC = [0 -1 0; 1 0 0; 0 0 0]; % Complementary Lie-algebra generator
    
    %% Parameters of the spherical robot
    r   = 0.3;             % meters, radius of the spherical shell
    mS  = 2;               % kg, mass of the spherical shell
    b   = 0.2; h = 0.5;    % meters, size of the sides of the yoke
    mb  = 2; mh = 5;       % kg, mass of the sides of the yoke
    mY  = 2*(mb + mh);     % kg, mass of the yoke
    ell = 0.2;             % meters, length of the pendulum
    mP  = 1;               % kg, mass of the pendulum
    g   = 9.81;            % m/s^2, gravitational acceleration
    kbY  = 0.02;           % Breaking constant of the yoke
    kbP  = 0.05;           % Breaking constant of the pendulum
    kbS  = 0.08;           % Breaking constant of the shell
    m = mS + mY + mP;      % Mass of Shell + Yoke + Pendulum
    
    %% Derived parameters    
    jS    = (2/3)*mS*r^2; % Inertia coefficient of the spherical shell
    JShat = (1/2)*(jS)*I3; % Inertia tensor of the spherical shell
    jxx   = (mb/6 + mh/2)*b^2; 
    jyy   = (mb/2 + mh/6)*h^2; 
    JYhat = diag([jyy,jxx,0]); % Inertia tensor of the yoke
    jP    = (1/3)*mP*ell^2; 
    JPhat = diag([0,0,jP]); % Inertia tensor of the pendulum
    cPbar = -(ell/2)*ez; % Center of mass of the pendulum in the pendulum-fixed reference system
    
    %% Initial Conditions

    RS0 = I3; 
    RY0 = [1 0 0; 0 cos(0) -sin(0); 0 sin(0) cos(0)];
    RP0 = [cos(0) 0 sin(0); 0 1 0; -sin(0) 0 cos(0)];

    WS = zeros(3,3); 
    WY = zeros(3,3); 
    WP = zeros(3,3);

    % We maintain 100 seconds as the simulation maximum time limit.
    % We maintain 0.01 second as time time-step size for integration.
    N = 10000; % Total Number of time steps (determines the size of history arrays)
    T = 0.01;  % Time Step resolution (0.01 seconds)

    % History arrays of Shell, Yoke, Pendulum, and Shell Center positions
    RS = zeros(3,3,N); 
    RY = zeros(3,3,N); 
    RP = zeros(3,3,N);
    q = zeros(3,N); 

    % history of control inputs
    tauY = zeros(3,3,N); 
    tauP = zeros(3,3,N);

    % Set initial positions
    RS(:,:,1) = RS0; 
    RY(:,:,1) = RY0; 
    RP(:,:,1) = RP0;     
    q(:,1) = [0 0 r]';

    %% Comment out these lines out if you do not want a preset trajectory.
    % Piecewise constant control law - PRE-SET robot trajectory
    % for i = 1:800, tauY(:,:,i) = 0.05*SY; end
    % for i = 1600:1900, tauP(:,:,i) = -0.05*SP; end
    % for i = 2500:3000, tauY(:,:,i) = -0.05*SY; end
    % for i = 3400:4500, tauP(:,:,i) = 0.05*SP; end
    

    % Uncomment the lines belows if you want to use a joystick
    Initialize joystick (if not already initialized)
    joy = joydev(1);  % Access the first joystick device (may vary depending on your system)
    % End of joystick initialization section

    % Size of plots and animation window panes
    figsize = [460 270 1350 710]; % Dimensioni e posizione delle figure

    % Initialize RTM Graphics Renderer
    customAxisLimits = [-1, 1,-1.5, 1.5, 0, 0.8];  
    InitRenderer(q(:,1), RS(:,:, 1), RY(:,:, 1), RP(:,:, 1), r, figsize, customAxisLimits);    

    % This is just a trick to ensure the focus is on the graphics pane
    fig = getappdata(gcf, 'figure_handle');  % Get the figure handle stored during InitRenderer
    figure(fig); % Ensure focus is on the figure    

    %% Simulation Loop
    % The main loop starts here... and runs for N*T seconds.
    n = 1;
    while n < N
        i = 1;
        while i <= 10 && n < N 

              %% Uncomment the lines below if you use a joystick
              % % Read joystick input
              % [xAxis, yAxis, buttonState] = joydev(joy);  % Replace with your joystick input reading method
              % 
              % %Analog Joysticks will produce an output from -1 to 1.
              % %So, we multiply that to values for tauY and tauP accordingly.
              % %If the output is too close to zero (~0.1) we consider it zero.
              % 
              % if yAxis < -0.1 || yAxis > 0.1  % Joystick pushed up or down
              %   tauY(:,:,n) = 0.05 * yAxis* SY;  % Note The sign of yAxis
              % end
              % 
              % if xAxis < -0.1 || xAxis > 0.1  % Joystick pushed left or right
              %   tauP(:,:,n) = 0.05 * xAxis* SP;  % Note The sign of xAxis
              % end
              % 
              %% End of joystick input section

              %% This is always a nice way to interrupt the execution
              % Detect key press and update torques
              key = get(gcf, 'CurrentKey');

              %% This block simply checks an EXIT request by the user
              if strcmp(key,'x') || strcmp(key,'X')
                 disp('Exit key detected. Terminating simulation.');
                 return; %exit both loops and end the program;
              end

              %% Comment out these lines if you do not want keyboard input
              % Detect up & down keys for YOKE control and update torques
              switch key
                     case 'uparrow'
                          tauY(:,:,n) = 0.05 * SY;
                     case 'downarrow'
                          tauY(:,:,n) = -0.05 * SY;
              end
              % Detect left & right keys for PENDULUM control and update torques
              switch key
                     case 'leftarrow'
                          tauP(:,:,n) = -0.05 * SP;
                     case 'rightarrow'
                          tauP(:,:,n) = 0.05 * SP;
              end
              %% End of keyboard input section

              %% Invoke updateStep funcion for one time-step only
              [RS(:,:,n+1), RY(:,:,n+1), RP(:,:,n+1), WS, WY, WP, q(:,n+1)] = updateStep(RS(:,:,n), RY(:,:,n), RP(:,:,n), WS, WY, WP, q(:,n), tauY(:,:,n), tauP(:,:,n), r, m, JShat, JYhat, JPhat, cPbar, mP, kbS, kbY, kbP, T, g, ez, SY, SP, SC);        
              i = i+1;
              n = n+1;
        end
        %% Render the sphere robot only once every 10 time-step iterations
        StepRenderer(q(:,n), RS(:,:,n), RY(:,:,n), RP(:,:,n), T, n); 
        figure(fig); % Ensure that the focus is alway on the figure   
    end    
    %% Clean up after the graphics rendering routines
    FinalizeRenderer();
    %% End of Simulation Loop section 

    %% After the simulation completes... generates additional plots...
    % Rendering of the position plots of the robot
    figure("Position",figsize)
    subplot(2,2,[1,3]); plot(q(1,:),q(2,:),'-'); hold on; grid on; plot(q(1,1),q(2,1),'bo')
                        title('Trajectory of the robot')
                        xlabel('Coordinate along the $x$ axis (m)')
                        ylabel('Coordinate along the $y$ axis (m)')
    subplot(2,2,2); plot(T*(0:N-1),q(1,:)); grid on; xlabel('Time $t$ (sec)'); ylabel('$x$ coordinate (m)')
    subplot(2,2,4); plot(T*(0:N-1),q(2,:)); grid on; xlabel('Time $t$ (sec)'); ylabel('$y$ coordinate (m)')
    print 'trajectory' -dpng -r300

    % Rendering of the inclination of the yoke and pendulum
    inclinationY = squeeze(atan2(RY(3,2,:),RY(2,2,:)));
    inclinationP = squeeze(atan2(RP(1,3,:),RP(3,3,:)));
    figure("Position",figsize)
    subplot(2,1,1); plot(T*(0:N-1),unwrap(inclinationY)); grid on; xlabel('Time (sec)'); ylabel('Inclination (rad)')
                    title('Inclination of the yoke with respect to the shell')
    subplot(2,1,2); plot(T*(0:N-1),unwrap(inclinationP)); grid on; xlabel('Time (sec)'); ylabel('Inclination (rad)')
                    title('Inclination of the pendulum with respect to the yoke')
    print 'attitude' -dpng -r300

%% The execution ends here.
disp('All done. Terminating simulation.');
return;

%% %% Auxiliary Functions
%% Initialize Renderer
function InitRenderer(q, RS, RY, RP, rS, figsize, axis_limits)
    % Set default axis limits if not provided
    if nargin < 5
        axis_limits = [-1, 1, -1, 1, 0, 1];
    end

    % Initialize the figure
    fig=figure("Position", figsize, 'WindowKeyPressFcn', @KeyPressCallback); 
    clf(fig); hold on; axis equal; grid on; view([45, 25]); 
    colormap([0, 0, 1]); % Set a blue colormap
    xlabel('$x$ axis'); ylabel('$y$ axis'); zlabel('$z$ axis');
    ttl = title(sprintf('Type X to stop the simulation. Time into simulation $t$ = 0.00 sec'));
    ttl.Units = 'Normalize'; ttl.Position(1) = 0; ttl.HorizontalAlignment = 'left'; 

    % Apply axis limits
    axis(axis_limits);
    % Save the axis limits in the figure's app data
    setappdata(fig, 'axis_limits', axis_limits);    

    % Sphere geometry
    NS = 30; 
    [yr, zr, xr] = sphere(NS - 1);
    xS = rS * xr; yS = rS * yr; zS = rS * zr;
    shell = [reshape(xS, 1, NS^2); reshape(yS, 1, NS^2); reshape(zS, 1, NS^2)];
    updated_shell = RS * shell + q;

    % Yoke geometry
    L = 0.15; H = 0.10;
    yoke = [-L, L, L, -L, -L; -H, -H, H, H, -H; 0, 0, 0, 0, 0];
    updated_yoke = RS * RY * yoke + q;

    % Yoke axis
    yokeaxis = [-rS, rS; 0, 0; 0, 0];
    updated_yokeaxis = RS * RY * yokeaxis + q;

    % Pendulum geometry
    pendulum = [0, 0; 0, 0; 0, -L];
    updated_pendulum = RS * RY * RP * pendulum + q;

    % Render the objects
    surf(reshape(updated_shell(1,:), NS, NS), ...
         reshape(updated_shell(2,:), NS, NS), ...
         reshape(updated_shell(3,:), NS, NS), 'FaceAlpha', 0.2, 'Tag', 'dynamic_object'); % Tag to allow selective clearing
    line(updated_yoke(1,:), updated_yoke(2,:), updated_yoke(3,:), ...
         'LineWidth', 2.5, 'Color', 'blue', 'Tag', 'dynamic_object'); % Tag to allow selective clearing
    line(updated_yokeaxis(1,:), updated_yokeaxis(2,:), updated_yokeaxis(3,:), ...
         'LineWidth', 2, 'Color', 'green', 'Tag', 'dynamic_object'); % Tag to allow selective clearing
    line(updated_pendulum(1,:), updated_pendulum(2,:), updated_pendulum(3,:), ...
         'LineWidth', 2.5, 'Color', 'red', 'Tag', 'dynamic_object'); % Tag to allow selective clearing

    % Save the initialized objects for later updates in StepRenderer
    setappdata(gcf, 'shell', shell);
    setappdata(gcf, 'yoke', yoke);
    setappdata(gcf, 'yokeaxis', yokeaxis);
    setappdata(gcf, 'pendulum', pendulum);

    % Save the figure handle itself in the figure's app data
    setappdata(fig, 'figure_handle', fig);    
end


%% This tricky function ensures that the focus remains on the picture even after a key is pressed
% Examine the line below in the InitRenderer function:
%    fig=figure("Position", figsize, 'WindowKeyPressFcn', @KeyPressCallback);
% It sets up a callback function to regain control upon a key is pressed...
% .. and manages to "re-focus" the pane where the simulation is running.
function KeyPressCallback(~, event)
    global key;
    key = event.Key;
    figure(gcf); % Re-focus on the current figure to avoid switching back to IDE
end
%% End InitRenderer

%% Step Renderer
function StepRenderer(q, RS, RY, RP, T, n)

    % Fetch the objects from the previous initialization
    fig = getappdata(gcf, 'figure_handle');  % Get the figure handle stored during InitRenderer

    shell = getappdata(gcf, 'shell');
    yoke = getappdata(gcf, 'yoke');
    yokeaxis = getappdata(gcf, 'yokeaxis');
    pendulum = getappdata(gcf, 'pendulum');
    
    % Clear the previous frame
    %clf(fig); % This clears it all, erasing the recorded path.
    delete(findobj(gca, 'Tag', 'dynamic_object')); 
    hold on;
    axis equal;
    grid on;
    view([45, 25]); 
    map = [0, 0, 1];
    colormap(map);
    xlabel('$x$ axis');
    ylabel('$y$ axis');
    
    % Update title with the time of the simulation
    ttl = title(sprintf('Type X to stop the simulation. Time into simulation $t$ = %2.2f sec', n*T));
    ttl.Units = 'Normalize'; 
    ttl.Position(1) = 0; 
    ttl.HorizontalAlignment = 'left'; 

    % Retrieve the axis limits from the initialization (set in InitRenderer)
    axis_limits = getappdata(gcf, 'axis_limits');
    axis(axis_limits);

    % Retrieve or initialize the previous position (q0)
    if isappdata(gcf, 'previous_position')
        q0 = getappdata(gcf, 'previous_position');
    else
        q0 = q; % Initialize q0 with the current q on the first call
        setappdata(gcf, 'previous_position', q0); % Save it
    end
   
    % Rendering of the graphical objects at the current time step
    updated_shell = RS * shell + q;  % Update position of the shell
    surf(reshape(updated_shell(1,:), 30, 30), reshape(updated_shell(2,:), 30, 30), reshape(updated_shell(3,:), 30, 30), 'FaceAlpha', 0.2', 'Tag', 'dynamic_object'); % Tag to allow selective clearing
    
    updated_yoke = RS * RY * yoke + q;  % Update position of the yoke
    line(updated_yoke(1,:), updated_yoke(2,:), updated_yoke(3,:), 'LineWidth', 2.5, 'Color', 'blue', 'Tag', 'dynamic_object'); % Tag to allow selective clearing
    
    updated_yokeaxis = RS * RY * yokeaxis + q;  % Update position of the yoke axis
    line(updated_yokeaxis(1,:), updated_yokeaxis(2,:), updated_yokeaxis(3,:), 'LineWidth', 2, 'Color', 'green', 'Tag', 'dynamic_object'); % Tag to allow selective clearing
    
    updated_pendulum = RS * RY * RP * pendulum + q;  % Update position of the pendulum
    line(updated_pendulum(1,:), updated_pendulum(2,:), updated_pendulum(3,:), 'LineWidth', 2.5, 'Color', 'red', 'Tag', 'dynamic_object'); % Tag to allow selective clearing);
    
    % Plot the path of the sphere (if applicable)
    %plot(q(1), q(2), 'r.');  % Plot the path (without time indexing)

    % Plot the trajectory line from q0 to q (time-step segment).
    line([q0(1), q(1)], [q0(2), q(2)], 'LineWidth', 2.5, 'Color', 'red');
    
    q0 = q;
    setappdata(gcf, 'previous_position', q0); % Save it for next step

    % Pause to update the animation (for 1 ms)
    pause(0.001);

end
%% End Step Renderer

%% FinalizeRenderer
function FinalizeRenderer()
    % Delete the stored graphical objects
    rmappdata(gcf, 'shell');
    rmappdata(gcf, 'yoke');
    rmappdata(gcf, 'yokeaxis');
    rmappdata(gcf, 'pendulum');
    rmappdata(gcf, 'axis_limits');
    rmappdata(gcf, 'previous_position');
    rmappdata(gcf, 'figure_handle');
    
    % Close the figure window if desired (optional)
    % close(gcf);
    
    % Placeholder for any additional finalization tasks
    disp('Rendering Complete. All resources cleaned up.');
end

%%-----------------------------------------------------------------------------
%% This is the stepUpdate function which actually simulates the robt dynamics
%%-----------------------------------------------------------------------------
function [RSn, RYn, RPn, WSn, WYn, WPn ,qn] = updateStep(RS, RY, RP, ... 
          WS, WY, WP, q, tauY, tauP, r, m, JShat, JYhat, JPhat, cPbar, mP, kbS, kbY, kbP, T, g, ez, SY, SP, SC)
    % Variable inertia matrix-coefficients
    
    % Helper function for the computation of the robot dynamics
    adstar  = @(X,Y) -(X*Y - Y*X);    % Dual adjoint operator    
    sigma = @(X) (1/2)*(X - X'); 
    sigmaY = @(X) trace(X'*SY)*SY/2; 
    sigmaP = @(X) trace(X'*SP)*SP/2;
    diamond = @(a, b)-sigma(a*b');
   
    gamma = RS'*ez;

    JSS = m * r^2 * (gamma * gamma') + JShat + ...
          RY * JYhat * RY' + ...
          RY * RP * JPhat * RP' * RY' + ...
          mP * r * (gamma * cPbar' * RP' * RY' + RY * RP * cPbar * gamma');    

    JSY = RY * JYhat * RY' + ...
          RY * RP * JPhat * RP' * RY' + ...
          mP * r * gamma * cPbar' * RP' * RY';    

    JSP = (RY * RP * JPhat + mP * r * gamma * cPbar') * RP';
    
    JYS = JSY';    
    JYY = RY * (JYhat + RP * JPhat * RP') * RY';    
    JYP = RY * RP * JPhat * RP';
    
    JPS = JSP';    
    JPY = JYP';    
    JPP = RP * JPhat * RP';
    
    % Derivatives of the reduced-constrained Lagrangians
    dl_dgamma = -mP * g * RY * RP * cPbar;
    
    dl_dv = WS * (m * r * gamma + mP * RY * RP * cPbar) + ...
             WY * (mP * RY * RP * cPbar) + ...
             mP * RY * WP * RP * cPbar;
    
    dlc_dWS = sigma(JSS * WS + JSY * WY + JSP * WP * RY');
    
    dlc_dRY = (WS + WY)' * (WS + WY) * RY * JYhat + ...
              (WS + WY + RY * WP * RY') * RY * RP * JPhat * RP' * WP' + ...
              RY * RP * JPhat * RP' * RY' * (WS + WY + RY * WP * RY') * RY * WP + ...
              (WS + WY + RY * WP * RY')' * (WS + WY + RY * WP * RY') * RY * RP * JPhat * RP' + ...
              mP * r * WS * gamma * cPbar' * RP' * WP' + ...
              mP * r * RY * RP * cPbar * gamma' * WS' * RY * WP + ...
              mP * r * (WS + WY + RY * WP * RY')' * WS * gamma * cPbar' * RP' - ...
              mP * g * gamma * cPbar' * RP';
    
    dlc_dRP = RY' * (WS + WY + RY * WP * RY')' * (WS + WY + RY * WP * RY') * RY * RP * JPhat + ...
              mP * r * RY' * (WS + WY + RY * WP * RY')' * WS * gamma * cPbar' - ...
              mP * g * RY' * gamma * cPbar';
    
    % Time-derivatives of the attitude matrices and of the gamma array
    RSdot = RS * WS;
    RYdot = RY * WY;
    RPdot = RP * WP;
    gamma_dot = -WS * gamma;
    
    % Control law's
    % Torques on the three components
    TS = adstar(WS,dlc_dWS) - diamond(gamma,dl_dgamma) + r*diamond(gamma_dot,dl_dv) - kbS*WS;
    TY = sigmaY(RY' * dlc_dRY) + tauY - kbY * WY;
    TP = sigmaP(RP' * dlc_dRP) + tauP - kbP * WP;

    % Angular momenta
    LS = sigma( JSS*WS + JSY*WY + JSP*WP*RY');
    LY = sigmaY(JYS*WS + JYY*WY + JYP*WP*RY');
    LP = sigmaP(JPS*WS*RY + JPY*WY*RY + JPP*WP);
    LT = RS*(LS + RY*(LY + RP*LP*RP')*RY')*RS';


    % Time-derivative of the variable inertia matrices
    JSSdot = m*r^2*(gamma_dot*gamma' + gamma*gamma_dot') + ...
             RYdot*JYhat*RY' + RY*JYhat*RYdot' + ...
             RYdot*RP*JPhat*RP'*RY' + RY*RPdot*JPhat*RP'*RY' + ...
             RY*RP*JPhat*RPdot'*RY' + RY*RP*JPhat*RP'*RYdot' + ...
             mP*r*(gamma_dot*cPbar'*RP'*RY' + gamma*cPbar'*RPdot'*RY' + gamma*cPbar'*RP'*RYdot' + ...
                   RYdot*RP*cPbar*gamma' + RY*RPdot*cPbar*gamma' + RY*RP*cPbar*gamma_dot');

    JSYdot = RYdot*JYhat*RY' + RY*JYhat*RYdot' + ...
             RYdot*RP*JPhat*RP'*RY' + RY*RPdot*JPhat*RP'*RY' + ...
             RY*RP*JPhat*RPdot'*RY' + RY*RP*JPhat*RP'*RYdot' + ...
             mP*r*(gamma_dot*cPbar'*RP'*RY' + gamma*cPbar'*RPdot'*RY' + gamma*cPbar'*RP'*RYdot');

    JSPdot = RYdot*RP*JPhat*RP' + RY*RPdot*JPhat*RP' + RY*RP*JPhat*RPdot' + ...
             mP*r*(gamma_dot*cPbar'*RP' + gamma*cPbar'*RPdot');

    JYSdot = JSYdot';

    JYYdot = RYdot*JYhat*RY' + RY*JYhat*RYdot + ...
             RYdot*RP*JPhat*RP'*RY' + RY*RPdot*JPhat*RP'*RY' + ...
             RY*RP*JPhat*RPdot'*RY' + RY*RP*JPhat*RP'*RYdot';

    JYPdot = RYdot*RP*JPhat*RP' + RY*RPdot*JPhat*RP' + RY*RP*JPhat*RPdot';

    JPSdot = JSPdot';

    JPYdot = JYPdot';

    JPPdot = RPdot*JPhat*RP' + RP*JPhat*RPdot';

    % Effective torques
    ES = TS - sigma( JSSdot*WS + JSYdot*WY + JSPdot*WP*RY' + JSP*WP*RYdot');
    EY = TY - sigmaY(JYSdot*WS + JYYdot*WY + JYPdot*WP*RY' + JYP*WP*RYdot');
    EP = TP - sigmaP(JPSdot*WS*RY + JPS*WS*RYdot + JPYdot*WY*RY + JPY*WY*RYdot + JPPdot*WP);

    % Inversion of the inertia operator JI (original):
    % There is an apparent error: the matrix is not symetrical as it seems...
    % Numerically - Element (4,5) != Element (54,) ==> indicating a possible mathematical error...
    coeffMatrix = ...
        [trace(SY'*JSS*SY)  trace(SP'*JSS*SY)  trace(SC'*JSS*SY)  trace(SY'*JSY'*SY) trace(RY*SP'*JSP'*SY)
         trace(SY'*JSS*SP)  trace(SP'*JSS*SP)  trace(SC'*JSS*SP)  trace(SY'*JSY'*SP) trace(RY*SP'*JSP'*SP)
         trace(SY'*JSS*SC)  trace(SP'*JSS*SC)  trace(SC'*JSS*SC)  trace(SY'*JSY'*SC) trace(RY*SP'*JSP'*SC)
         trace(SY'*JYS'*SY) trace(SP'*JYS'*SY) trace(SC'*JYS'*SY) trace(SY'*JYY'*SY) trace(RY*SP'*JYP'*SY)
         trace(RY'*SY'*JPS'*SP) trace(RY'*SP'*JPS'*SP) trace(RY'*SC'*JPS'*SP) trace(SY'*JPY'*SP) trace(SP'*JPP'*SP)];

    % Crude attempt to fix the mathematical error: 
    % Inversion of the inertia operator JI - Attempted fix: elem (4,5)=(5,4)   
    coeffMatrix = ...
       [trace(SY'*JSS*SY)  trace(SP'*JSS*SY)  trace(SC'*JSS*SY)  trace(SY'*JSY'*SY) trace(RY*SP'*JSP'*SY)
        trace(SY'*JSS*SP)  trace(SP'*JSS*SP)  trace(SC'*JSS*SP)  trace(SY'*JSY'*SP) trace(RY*SP'*JSP'*SP)
        trace(SY'*JSS*SC)  trace(SP'*JSS*SC)  trace(SC'*JSS*SC)  trace(SY'*JSY'*SC) trace(RY*SP'*JSP'*SC)
        trace(SY'*JYS'*SY) trace(SP'*JYS'*SY) trace(SC'*JYS'*SY) trace(SY'*JYY'*SY) trace(RY*SP'*JYP'*SY)
        trace(RY'*SY'*JPS'*SP) trace(RY'*SP'*JPS'*SP) trace(RY'*SC'*JPS'*SP) trace(RY*SP'*JYP'*SY) trace(SP'*JPP'*SP)];
    % This fix did not cause signficant change in the workings of the program.
    % That's why we left it commented out by /FAC.
    

    coeffArray = ...
        [trace(ES'*SY)
         trace(ES'*SP)
         trace(ES'*SC)
         trace(EY'*SY)
         trace(EP'*SP)];
    coeffJIinv = coeffMatrix\coeffArray;

    % Angular accelerations
    WSdot = coeffJIinv(1)*SY + coeffJIinv(2)*SP + coeffJIinv(3)*SC;
    WYdot = coeffJIinv(4)*SY;
    WPdot = coeffJIinv(5)*SP;

    % fEul-Riemann method to advance the attitudes
    % Compute the next step values for RS, RY, and RP
    RSn = RS*expm(T*WS);
    RYn = RY*expm(T*WY);
    RPn = RP*expm(T*WP);

    % fEul method to advance the angular velocities
    % Compute the next step values for WS, WY, and WP
    WSn = WS + T*WSdot;
    WYn = WY + T*WYdot;
    WPn = WP + T*WPdot;

    % fEul method to advance position
    % Compute the next step values for q
    qn = q + T*r*RSdot*gamma;    
end

%% ------------------------------------------------------------------------
%% THE END  (the whole story ends here.)
%% ------------------------------------------------------------------------