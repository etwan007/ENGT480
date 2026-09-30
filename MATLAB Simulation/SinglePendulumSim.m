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
lsys = linearize("cp_sim_2", io);

Ts = 1/1000;
%calculate discrete gain matrix
Kd = lqrd(lsys.A, lsys.B, diag([10,10,1,1]), 0.5, Ts);

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
Tfinal = 15;            % Total simulation time
N = round(Tfinal / Ts); % Number of time steps

%initial states
Y = [0.01; %theta
    0; %dtheta
    0.1; %x
    0]; %dx

Yn = [0;0;0;0];

%storage arrays
Y_store = zeros(4, N+1); %create an array to store states (Y) for each time step
Y_store(:,1) = Y; %store initial state (Y) in the first column of Y_store
U_store = zeros(1, N); %create an array to store the control input of each time step
t = 0:Ts:Tfinal; %create a time vector from 0 to Tfinal with increments of Ts

kint = 0.5; %integral gain
xint = 0; %set initial xint value
kprop = 0.8; %set proportional gain
kdiff = 0.8; %derivative gain
xdiff = 0; %initial xdiff value
xlast = Y(3); %initial previous x value set to Y(3) to make first calculation zero

%simulation/control loop
for k = 1:N %run from time step 1 to time step N
    %compute derivative and integral of x
    xint = xint - Y(3) * Ts; %approximate integral of x
    xdiff = (xlast - Y(3))/Ts; %aproximate derivative of x

    %compute control input, and correct for x error
    u = -Kd * Y; %calculate control input
    u = u - kint * xint + kprop * Y(3) - kdiff * xdiff; %correct for drift with integral/derivative of x (Y(3))

    xlast = Y(3); %store x value for next loop

    %RK4 integration to simulate system response to control input
    k1 = Ydot(t(k), Y, u, F_dragVal, Ival, mcartVal, B_cart_dragVal, gval, lval, m1val); %slope at start of time step
    k2 = Ydot(t(k)+Ts/2, Y + 0.5*k1*Ts, u, F_dragVal, Ival, mcartVal, B_cart_dragVal, gval, lval, m1val); %slope at the midpoint
    k3 = Ydot(t(k)+Ts/2, Y + 0.5*k2*Ts, u, F_dragVal, Ival, mcartVal, B_cart_dragVal, gval, lval, m1val); %overall slope of timestep
    k4 = Ydot(t(k)+Ts, Y + k3*Ts, u, F_dragVal, Ival, mcartVal, B_cart_dragVal, gval, lval, m1val); %slope at end of timestep
    
    Y = Y + ((k1 + 2*k2 + 2*k3 +k4)/6) * Ts; %approximate new state (Y)
    
    if k == 2000 %wait until the time is 2 seconds
        Y = Y + [0;0.5;0;0]; %add an impulse to the pendulum
    end

    %store states and control input at each time step
    Y_store(:,k+1) = Y;
    U_store(k) = u;
end

%plot the result
figure;
hold on;

plot(t, Y_store(1,:), 'DisplayName', '\theta (rad)');
plot(t, Y_store(2,:), 'DisplayName', 'd\theta/dt (rad/s)');
plot(t, Y_store(3,:), 'DisplayName', 'x (m)');
plot(t, Y_store(4,:), 'DisplayName', 'dx/dt (m/s)');

%For u, use t(1:end-1) since U_store has N points
plot(t(1:end-1), U_store, 'DisplayName', 'Control input u (N)');

xlabel('Time (s)');
ylabel('States/Control Input');
title('Discrete CL Simulation');
legend('Location', 'northeast');
grid on;
hold off;