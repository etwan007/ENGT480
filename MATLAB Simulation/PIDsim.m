clc
clear %remove variables

%create variables for the system
syms t g l I m1 mcart H V F F_drag theta(t) T_drag x(t) B_cart_drag

%assume variable information (real, always positive)
assume ([t g l m1 mcart B_cart_drag T_drag] > 0)
assume ([I H V F F_drag], "real")

%differential equations
CartH = F -H - B_cart_drag*diff(x,1) == mcart*diff(x,2);
HorizontalForce = H - F_drag == m1 * diff(x-l*sin(theta),2);

mainEq1 = eliminate([CartH HorizontalForce], H);

Normal_to_bar = F_drag*cos(theta) - V*sin(theta) - H*cos(theta) + m1*g*sin(theta) == m1*(l*diff(theta,2) - diff(x,2)*cos(theta));
Torque = V*l*sin(theta) + H*l*cos(theta) == I*diff(theta,2);

mainEq2 = eliminate([Normal_to_bar Torque],[H V]);

%convert to state space
[ssEqs, states] = odeToVectorField([mainEq1 mainEq2]);

%convert to matlab function
Ydot = matlabFunction(ssEqs, Vars=["t","Y","F","F_drag","I","mcart","B_cart_drag","g","l","m1"]);

%linearize the simulink model
io(1) = linio("cp_sim_2/Force",1,"input");
io(2) = linio("cp_sim_2/Integrator",1,"output");
%io(3) = linio("cp_sim_2/Integrator1",1,"output");
lsys = linearize("cp_sim_2", io);

Ts = 1/1000;

%calculate discrete gain matrix
Kd = lqrd(lsys.A, lsys.B, diag([10,2,1,1]), 0.5, Ts);

%calculate matrices for a discrete kalman filter
Cmod = [1 0 0 0; %modified C matrix
        0 0 1 0];
Dmod = [0; %modified D matrix
        0];
Ksys = ss(lsys.A, lsys.B, Cmod, Dmod);
[Kest,L,P,M,Z] = kalmd(Ksys,0.03,0.1,Ts);

%Actual system values - MAKE SURE THESE MATCH SIMULINK
%simulink values are the system values that will be used to calculate Kd
mweight = 0.26; %weight of pendulum weight
mrod = 0.1; %weight of pendulum arm
lval = 0.36; %in meters (center of mass)
F_dragVal = 0.01; %pendulum drag force
Ival = (mweight + (mrod/3))*lval^2; %rotational inertia
mcartVal = 1; %in kg
B_cart_dragVal = 0.5; %drag coefficient
gval = 9.81; %gravitational constant
m1val = mweight + mrod; %in kg






% --- Discrete CL simulation of control system ---
%simulation parameters
Tfinal = 15*4;            % Total simulation time
N = round(Tfinal / Ts); % Number of time steps

%initial states
Y = [0.05; %theta
     0.002; %dtheta
     0; %x
     0]; %dx

Yest = Y + randn(4,1)*0.000384; %initialize variable for kalman filter estimation
Yest(2) = 0; %cannot measure dtheta, set it to 0 by default

Yn = [0;0;0;0]; %initialize array for state noise

Ymeas = [Yest(1);Yest(3);Yest(4)]; %array for measurements

Ylast = [0;0;0;0]; %initialize array
Ylastlast = [0;0;0;0]; %initialize array

dtheta = 0; %initialize variable for dtheta estimation
u = 0; %initialize control force variable
a = 0;
v = Yest(4);

%storage arrays
Y_store = zeros(4, N+1); %create an array to store states (Y) for each time step
Y_store(:,1) = Y; %store initial state (Y) in the first column of Y_store
Yest_store = zeros(4, N+1); %create an array to store states (Y) for each time step
Yest_store(:,1) = Y; %store initial state (Y) in the first column of Y_store
U_store = zeros(1, N+1); %create an array to store the control input of each time step
f_store = zeros(1, N+1); %create an array to store the pulse frequency
dir_store = zeros(1, N+1); %create an array to store the direction
t = 0:Ts:Tfinal; %create a time vector from 0 to Tfinal with increments of Ts

kInt = 6; %integral gain
thetaInt = 0; %set initial xint value
kProp = 30; %set proportional gain
kDiff = 1; %derivative gain
thetaDiff = 0; %initial xdiff value
thetaLast = Yest(1); %initial previous x value set to Y(3) to make first calculation zero
thetaLastLast = Yest(1);
thetaTarget = 0;
error = 0;
xint = 0;
xdiff = 0;
xlast = Yest(3);
xlastlast = Yest(3);
ulast = 0;
ulastlast = 0;

%simulation/control loop
for k = 1:N %run from time step 1 to time step N

    %START CONTROL LOOP

    %find error, and adjust target theta
    thetaTarget = Yn(3)/5;
    thetaTarget = min(max(thetaTarget,-0.), 0.2); %cap targetTheta
    error = Yn(1) - thetaTarget;

    %approximate integral and derivative of theta and x
    thetaInt = thetaInt + error * Ts; %approximate integral
    thetaDiff = (error - (thetaLast + thetaLastLast)/2)/Ts; %approximate derivative
    thetaLastLast = thetaLast;
    thetaLast = error; %store last theta value
    xint = xint + Yn(3) * Ts;
    ulast = 0;

    %compute PID control input
    ulastlast = ulast;
    ulast = u;
    u = -error*kProp - thetaInt*kInt;
    % if (Yn(3)>0.15 || Yn(3)<0.15)
    u = u + 1*Yn(3) + 0.1*xint + 0.1*Yn(4);
    % end
    u = min(max(u,-10), 10);
    % u = (u + ulast)/2;

    %1.8° full step --> 200 steps per rotation
    %30mm diameter pulley --> 94.24778mm per rotation
    %0.4712389mm per step
    %v = (pulses/second)*(m/pulse) = fpulse*0.0004712389 --> fpulse = v/0.0004712389
    % a = u/mcartVal; %convert force to target cart acceleration
    % v = Ymeas(3) + a; %calculate target velocity of cart
    % fpulse = v / 0.0004712389; %calculate pulse frequency needed
    % fpulse = min(max(fpulse,-6666), 6666); %clamp fpulse to max value
    % fpulse * 0.0004712389; %calculate 'measured' v for next loop
    % if fpulse < 0
    %     fpulse = fpulse*(-1);
    %     direction = 1;
    % else
    %     direction = 0;
    % end

    %GRAPH OF ANGULAR VELOCITY AND ACCELERATION OF MOTOR, AND Nm

    % Y(4) = fpulse * 0.0004712389; %calculate 'measured' dx for next loop
    % u = 0;

    %END CONTROL LOOP


    %RK4 integration to simulate system response to control input
    k1 = Ydot(t(k), Y, u, F_dragVal, Ival, mcartVal, B_cart_dragVal, gval, lval, m1val); %slope at start of time step
    k2 = Ydot(t(k)+Ts/2, Y + 0.5*k1*Ts, u, F_dragVal, Ival, mcartVal, B_cart_dragVal, gval, lval, m1val); %slope at the midpoint
    k3 = Ydot(t(k)+Ts/2, Y + 0.5*k2*Ts, u, F_dragVal, Ival, mcartVal, B_cart_dragVal, gval, lval, m1val); %overall slope of timestep
    k4 = Ydot(t(k)+Ts, Y + k3*Ts, u, F_dragVal, Ival, mcartVal, B_cart_dragVal, gval, lval, m1val); %slope at end of timestep
    
    Y = Y + ((k1 + 2*k2 + 2*k3 +k4)/6) * Ts; %approximate new state (Y)
    
    % if k == 2*(1/Ts) %wait until the time is 2 seconds
    %     Y = Y + [0;0.2;0;0]; %add an impulse to the pendulum
    % end

    %add noise to states
    Yn = Y + randn(4,1)*0.000384; %max 0.3° error

    %store states and control input at each time step for plotting response
    Y_store(:,k+1) = Y;%min(max(Y,-10), 10);
    Yest_store(:,k+1) = Yn;%min(max(Yn,-10), 10);
    U_store(k+1) = u;%min(max(u,-10), 10);
    f_store(k+1) = thetaTarget;%fpulse/1000; %store pulse frequency in kHz
    %dir_store(k+1) = direction; %store pulse direction
end

%plot the result
figure;
hold on;

%For u, use t(1:end-1) since U_store has N points
plot(t(1,:), U_store, 'DisplayName', 'Control input u (N)');

plot(t, Y_store(1,:), 'DisplayName', '\theta (rad)');
plot(t, Y_store(2,:), 'DisplayName', 'd\theta/dt (rad/s)');
plot(t, Y_store(3,:), 'DisplayName', 'x (m)');
plot(t, Y_store(4,:), 'DisplayName', 'dx/dt (m/s)');
% plot(t, Yest_store(1,:), 'DisplayName', '\theta2(rad)');
% plot(t, Yest_store(2,:), 'DisplayName', 'd\theta/dt2 (rad/s)');
% plot(t, Yest_store(3,:), 'DisplayName', 'x2 (m)');
% plot(t, Yest_store(4,:), 'DisplayName', 'dx/dt2 (m/s)');

xlabel('Time (s)');
ylabel('States/Control Input');
title('Discrete CL Simulation');
legend('Location', 'northeast');
grid on;
hold off;

%other plot
% figure;
% hold on;
% plot(t(1,:), dir_store, 'DisplayName', 'Direction');
% plot(t(1,:), f_store, 'DisplayName', 'Pulse frequency (kHz)');
% plot(t(1,:), U_store, 'DisplayName', 'Control input u (N)');
% 
% title('Discrete CL Simulation');
% legend('Location', 'northeast');
% grid on;
% hold off;