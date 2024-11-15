% Program to simulate the motion of a spherical robot that rolls without
% slipping on a flat surface - by Simone Fiori (DII-UnivPM)
% Version: July 2023

%% Kick-off
clc; close all; clear 
set(0,'defaultTextInterpreter','latex')
set(0,'defaultAxesTickLabelInterpreter','latex')
set(0,'defaultLegendInterpreter','latex')
set(0,'defaultAxesFontSize',16)
set(0,'defaultTextFontSize',16)
set(0,'DefaultLineLineWidth',2)
figsize = [460 470 1550 710]; % Dimensioni e posizione delle figure

%% Parameters of the spherical robot
r   = 0.3;             % meters, radius of the spherical shell
mS  = 2;               % kg, mass of the spherical shell
b   = 0.2; h = 0.5;    % meters, size of the sides of the yoke
mb  = 2; mh = 5;       % kg, mass of the sides of the yoke
mY  = 2*(mb + mh);     % kg, mass of the yoke
ell = 0.2;             % meters, length of the pendulum
mP  = 1;               % kg, mass of the pendulum
g   = 9.81;            % m/s^2, gravitational acceleration
kbY  = 0.02;           % Braking constant of the yoke
kbP  = 0.05;           % Braking constant of the pendulum
kbS  = 0.08;            % Braking constant of the shell

%% Mathematical operators
I3      = eye(3);                 % Identity matrix
ez      = [0 0 1]';               % Asse z
SY      = [0 0 0; 0 0 -1; 0 1 0]; % Lie-algebra generator for the yoke angular velocity
SP      = [0 0 1; 0 0 0; -1 0 0]; % Lie-algebra generator for the pendulum angular velocity
SC      = [0 -1 0; 1 0 0; 0 0 0]; % Complementary Lie-algebra generator
adstar  = @(X,Y) -(X*Y - Y*X);    % Dual adjoint operator
sigma   = @(X) (1/2)*(X - X');    % Projection over the Lie algebra so(3)
sigmaY  = @(X) trace(X'*SY)*SY/2;   % Projection over span{SY}
sigmaP  = @(X) trace(X'*SP)*SP/2;   % Projection over span{SP}
diamond = @(a,b) -sigma(a*b');    % Diamond operator
smm     = @(X) X + X';            % Symmetrization

%% Derived parameters
jS    = (2/3)*mS*r^2; % Inertia coefficient of the spherical shell
JShat = (1/2)*jS*I3; % Inertia tensor of the spherical shell
jxx   = (mb/6 + mh/2)*b^2; jyy = (mb/2 + mh/6)*h^2; JYhat = diag([jyy,jxx,0]); % Inertia tensor of the yoke
jP    = (1/3)*mP*ell^2; JPhat = diag([0,0,jP]); % Inertia tensor of the pendulum
cPbar = -(ell/2)*ez; % Center of mass of the pendulum in the pendulum-fixed reference system
m     = mS + mY + mP; % Total mass of the robot

%% Definitions for the numerical simulation
RS0 = I3; % Initial attitude of the spherical shell
iaY = 0*pi/6; RY0 = [1 0 0; 0 cos(iaY) -sin(iaY); 0 sin(iaY) cos(iaY)]; % Initial attitude of the yoke
iaP = 0*pi/3; RP0 = [cos(iaP) 0 sin(iaP);0 1 0; -sin(iaP) 0 cos(iaP)]; % Initial attitude of the pendulum
WS  = zeros(3,3); % Initial angular velocity of the spherical shell (rad/sec)
WY  = zeros(3,3); % Initial angular velocity of the yoke (rad/sec)
WP  = zeros(3,3); % Initial angular velocity of the pendulum (rad/sec)
N = 10000; % Number of numerical steps
T = 0.01; % Sampling interval (sec)
RS = zeros(3,3,N); RY = zeros(3,3,N); RP = zeros(3,3,N); 
q = zeros(3,N); % Position of the center of the spherical shell
tauY = zeros(3,3,N); tauP = zeros(3,3,N);

%% Numerical simulation
% Initialization
RS(:,:,1) = RS0; RY(:,:,1) = RY0; RP(:,:,1) = RP0; 
q(:,1) = [0 0 r]';
% Piecewise constant control law
for i = 1:800, tauY(:,:,i) = 0.05*SY; end
for i = 1600:1900, tauP(:,:,i) = -0.05*SP; end
for i = 2500:3000, tauY(:,:,i) = -0.05*SY; end
for i = 3400:4500, tauP(:,:,i) = 0.05*SP; end
% Iteration
for n = 1:N-1
    % Variable inertia matrix-coefficients
    gamma = RS(:,:,n)'*ez;

    JSS = m*r^2*(gamma*gamma') + JShat + RY(:,:,n)*JYhat*RY(:,:,n)' + RY(:,:,n)*RP(:,:,n)*JPhat*RP(:,:,n)'*RY(:,:,n)' + ...
          mP*r*(gamma*cPbar'*RP(:,:,n)'*RY(:,:,n)' + RY(:,:,n)*RP(:,:,n)*cPbar*gamma');

    JSY = RY(:,:,n)*JYhat*RY(:,:,n)' + RY(:,:,n)*RP(:,:,n)*JPhat*RP(:,:,n)'*RY(:,:,n)' + ...
          mP*r*gamma*cPbar'*RP(:,:,n)'*RY(:,:,n)';
    
    JSP = (RY(:,:,n)*RP(:,:,n)*JPhat + mP*r*gamma*cPbar')*RP(:,:,n)';
    
    JYS = JSY';
    
    JYY = RY(:,:,n)*(JYhat + RP(:,:,n)*JPhat*RP(:,:,n)')*RY(:,:,n)';
    
    JYP = RY(:,:,n)*RP(:,:,n)*JPhat*RP(:,:,n)';
    
    JPS = JSP';
    
    JPY = JYP';
    
    JPP = RP(:,:,n)*JPhat*RP(:,:,n)';

    % Derivatives of the reduced-constrained Lagrangians
    dl_dgamma = -mP*g*RY(:,:,n)*RP(:,:,n)*cPbar;
    
    dl_dv     = WS*(m*r*gamma + mP*RY(:,:,n)*RP(:,:,n)*cPbar) + WY*(mP*RY(:,:,n)*RP(:,:,n)*cPbar) + ...
                mP*RY(:,:,n)*WP*RP(:,:,n)*cPbar;

    dlc_dWS   = sigma(JSS*WS + JSY*WY + JSP*WP*RY(:,:,n)');

    dlc_dRY   = (WS + WY)'*(WS + WY)*RY(:,:,n)*JYhat + ...
                (WS + WY + RY(:,:,n)*WP*RY(:,:,n)')*RY(:,:,n)*RP(:,:,n)*JPhat*RP(:,:,n)'*WP' + ...
                RY(:,:,n)*RP(:,:,n)*JPhat*RP(:,:,n)'*RY(:,:,n)'*(WS + WY + RY(:,:,n)*WP*RY(:,:,n)')*RY(:,:,n)*WP + ...
                (WS + WY + RY(:,:,n)*WP*RY(:,:,n)')'*(WS + WY + RY(:,:,n)*WP*RY(:,:,n)')*RY(:,:,n)*RP(:,:,n)*JPhat*RP(:,:,n)' + ...
                mP*r*WS*gamma*cPbar'*RP(:,:,n)'*WP' + mP*r*RY(:,:,n)*RP(:,:,n)*cPbar*gamma'*WS'*RY(:,:,n)*WP + ...
                mP*r*(WS + WY + RY(:,:,n)*WP*RY(:,:,n)')'*WS*gamma*cPbar'*RP(:,:,n)' - ...
                mP*g*gamma*cPbar'*RP(:,:,n)';
    
    dlc_dRP   = RY(:,:,n)'*(WS + WY + RY(:,:,n)*WP*RY(:,:,n)')'*(WS + WY + RY(:,:,n)*WP*RY(:,:,n)')*RY(:,:,n)*RP(:,:,n)*JPhat + ...
                mP*r*RY(:,:,n)'*(WS + WY + RY(:,:,n)*WP*RY(:,:,n)')'*WS*gamma*cPbar' - ...
                mP*g*RY(:,:,n)'*gamma*cPbar';

    % Time-derivatives of the attitude matrices and of the gamma array
    RSdot     = RS(:,:,n)*WS;
    RYdot     = RY(:,:,n)*WY;
    RPdot     = RP(:,:,n)*WP;
    gamma_dot = -WS*gamma;

    % Control law

    % Torques on the three components
    TS = adstar(WS,dlc_dWS) - diamond(gamma,dl_dgamma) + r*diamond(gamma_dot,dl_dv) - kbS*WS;
    TY = sigmaY(RY(:,:,n)'*dlc_dRY) + tauY(:,:,n) - kbY*WY;
    TP = sigmaP(RP(:,:,n)'*dlc_dRP) + tauP(:,:,n) - kbP*WP;

    % Angular momenta
    LS = sigma( JSS*WS + JSY*WY + JSP*WP*RY(:,:,n)');
    LY = sigmaY(JYS*WS + JYY*WY + JYP*WP*RY(:,:,n)');
    LP = sigmaP(JPS*WS*RY(:,:,n) + JPY*WY*RY(:,:,n) + JPP*WP);
    LT = RS(:,:,n)*(LS + RY(:,:,n)*(LY + RP(:,:,n)*LP*RP(:,:,n)')*RY(:,:,n)')*RS(:,:,n)';


    % Time-derivative of the variable inertia matrices
    JSSdot = m*r^2*(gamma_dot*gamma' + gamma*gamma_dot') + ...
             RYdot*JYhat*RY(:,:,n)' + RY(:,:,n)*JYhat*RYdot' + ...
             RYdot*RP(:,:,n)*JPhat*RP(:,:,n)'*RY(:,:,n)' + RY(:,:,n)*RPdot*JPhat*RP(:,:,n)'*RY(:,:,n)' + ...
             RY(:,:,n)*RP(:,:,n)*JPhat*RPdot'*RY(:,:,n)' + RY(:,:,n)*RP(:,:,n)*JPhat*RP(:,:,n)'*RYdot' + ...
             mP*r*(gamma_dot*cPbar'*RP(:,:,n)'*RY(:,:,n)' + gamma*cPbar'*RPdot'*RY(:,:,n)' + gamma*cPbar'*RP(:,:,n)'*RYdot' + ...
                   RYdot*RP(:,:,n)*cPbar*gamma' + RY(:,:,n)*RPdot*cPbar*gamma' + RY(:,:,n)*RP(:,:,n)*cPbar*gamma_dot');

    JSYdot = RYdot*JYhat*RY(:,:,n)' + RY(:,:,n)*JYhat*RYdot' + ...
             RYdot*RP(:,:,n)*JPhat*RP(:,:,n)'*RY(:,:,n)' + RY(:,:,n)*RPdot*JPhat*RP(:,:,n)'*RY(:,:,n)' + ...
             RY(:,:,n)*RP(:,:,n)*JPhat*RPdot'*RY(:,:,n)' + RY(:,:,n)*RP(:,:,n)*JPhat*RP(:,:,n)'*RYdot' + ...
             mP*r*(gamma_dot*cPbar'*RP(:,:,n)'*RY(:,:,n)' + gamma*cPbar'*RPdot'*RY(:,:,n)' + gamma*cPbar'*RP(:,:,n)'*RYdot');

    JSPdot = RYdot*RP(:,:,n)*JPhat*RP(:,:,n)' + RY(:,:,n)*RPdot*JPhat*RP(:,:,n)' + RY(:,:,n)*RP(:,:,n)*JPhat*RPdot' + ...
             mP*r*(gamma_dot*cPbar'*RP(:,:,n)' + gamma*cPbar'*RPdot');

    JYSdot = JSYdot';

    JYYdot = RYdot*JYhat*RY(:,:,n)' + RY(:,:,n)*JYhat*RYdot + ...
             RYdot*RP(:,:,n)*JPhat*RP(:,:,n)'*RY(:,:,n)' + RY(:,:,n)*RPdot*JPhat*RP(:,:,n)'*RY(:,:,n)' + ...
             RY(:,:,n)*RP(:,:,n)*JPhat*RPdot'*RY(:,:,n)' + RY(:,:,n)*RP(:,:,n)*JPhat*RP(:,:,n)'*RYdot';

    JYPdot = RYdot*RP(:,:,n)*JPhat*RP(:,:,n)' + RY(:,:,n)*RPdot*JPhat*RP(:,:,n)' + RY(:,:,n)*RP(:,:,n)*JPhat*RPdot';

    JPSdot = JSPdot';

    JPYdot = JYPdot';

    JPPdot = RPdot*JPhat*RP(:,:,n)' + RP(:,:,n)*JPhat*RPdot';

    % Effective torques
    ES = TS - sigma( JSSdot*WS + JSYdot*WY + JSPdot*WP*RY(:,:,n)' + JSP*WP*RYdot');
    EY = TY - sigmaY(JYSdot*WS + JYYdot*WY + JYPdot*WP*RY(:,:,n)' + JYP*WP*RYdot');
    EP = TP - sigmaP(JPSdot*WS*RY(:,:,n) + JPS*WS*RYdot + JPYdot*WY*RY(:,:,n) + JPY*WY*RYdot + JPPdot*WP);

    % Inversion of the inertia operator JI
    coeffMatrix = ...
        [trace(SY'*JSS*SY)  trace(SP'*JSS*SY)  trace(SC'*JSS*SY)  trace(SY'*JSY'*SY) trace(RY(:,:,n)*SP'*JSP'*SY)
         trace(SY'*JSS*SP)  trace(SP'*JSS*SP)  trace(SC'*JSS*SP)  trace(SY'*JSY'*SP) trace(RY(:,:,n)*SP'*JSP'*SP)
         trace(SY'*JSS*SC)  trace(SP'*JSS*SC)  trace(SC'*JSS*SC)  trace(SY'*JSY'*SC) trace(RY(:,:,n)*SP'*JSP'*SC)
         trace(SY'*JYS'*SY) trace(SP'*JYS'*SY) trace(SC'*JYS'*SY) trace(SY'*JYY'*SY) trace(RY(:,:,n)*SP'*JYP'*SY)
         trace(RY(:,:,n)'*SY'*JPS'*SP)... 
                            trace(RY(:,:,n)'*SP'*JPS'*SP)...
                                               trace(RY(:,:,n)'*SC'*JPS'*SP)...
                                                                  trace(SY'*JPY'*SP) trace(SP'*JPP'*SP)];
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
    RS(:,:,n+1) = RS(:,:,n)*expm(T*WS);
    RY(:,:,n+1) = RY(:,:,n)*expm(T*WY);
    RP(:,:,n+1) = RP(:,:,n)*expm(T*WP);

    % fEul method to advance the angular velocities
    WS = WS + T*WSdot;
    WY = WY + T*WYdot;
    WP = WP + T*WPdot;

    % fEul method to advance position
    q(:,n+1) = q(:,n) + T*r*RSdot*gamma;
end

%% Graphical rendering
% Animation of the rolling sphere
figure("Position",figsize); Renderer(q,RS,RY,RP,T,r);

% Rendering of the position of the robot
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

%% Auxiliary functions
function Renderer(q,RS,RY,RP,T,rS)
% Graphic object: shell
NS = 30; [yr,zr,xr] = sphere(NS-1);
xS = rS*xr; yS = rS*yr; zS = rS*zr;
shell = [reshape(xS,1,NS^2)
         reshape(yS,1,NS^2)
         reshape(zS,1,NS^2)];

% Graphic object: yoke
L = 0.15; H = 0.10;
yoke = [-L +L +L -L -L
        -H -H +H +H -H
         0  0  0  0  0];
yokeaxis = [-rS rS
             0  0
             0  0];

% Graphic object: pendulum
pendulum = [0  0
            0  0
            0  -L];

% Initialization of current frame
for n = 1:10:size(q,2)
clf; hold on; axis equal; grid on; view([45,25]); 
map=[0,0,1]; colormap(map); 
xlabel('$x$ axis'); ylabel('$y$ axis')
ttl = title(sprintf('Time into simulation $t$ = %2.2f sec',T*(n-1))); 
ttl.Units = 'Normalize'; ttl.Position(1) = 0; ttl.HorizontalAlignment = 'left'; 
%axis([q(1)-0.7 q(1)+0.7 q(2)-0.7 q(2)+0.7 q(3)-0.2 q(3)+0.2]);
axis([min(q(1,:))-0.5-rS max(q(1,:))+0.5+rS min(q(2,:))-0.5-rS max(q(2,:))+0.5+rS 0 0.5+rS]);

% Rendering of the graphical objects
updated_shell = RS(:,:,n)*shell + q(:,n);
surf(reshape(updated_shell(1,:),NS,NS),reshape(updated_shell(2,:),NS,NS),reshape(updated_shell(3,:),NS,NS),'FaceAlpha',0.2);
updated_yoke  = RS(:,:,n)*RY(:,:,n)*yoke + q(:,n);
line(updated_yoke(1,:),updated_yoke(2,:),updated_yoke(3,:),'LineWidth',2.5,'color','blue')
updated_yokeaxis  = RS(:,:,n)*RY(:,:,n)*yokeaxis + q(:,n);
line(updated_yokeaxis(1,:),updated_yokeaxis(2,:),updated_yokeaxis(3,:),'LineWidth',2,'color','green')
updated_pendulum = RS(:,:,n)*RY(:,:,n)*RP(:,:,n)*pendulum + q(:,n);
line(updated_pendulum(1,:),updated_pendulum(2,:),updated_pendulum(3,:),'LineWidth',2.5,'color','red')
plot(q(1,1:n),q(2,1:n),'r.')
pause(0.001)
end
end