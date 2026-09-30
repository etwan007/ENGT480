clc

%State matrices
A = [0 1; 2 -1];
B = [1;0];
C = [1 0];
D = 0;

%Make a state space object
sys = ss(A, B, C, D);

%Open loop eigen values (unstble because of the +1)
E = eig(A)

%Desired closed loop eigen values (all negative for stability)
P = [-3 -1];

%Solve for K gains to stabilize
K = place(A, B, P)

%Check closed loop eigen values(make sure they were properly set)
Acl = A - B*K
Ecl = eig(Acl)

%Create a closed loop system object
syscl = ss(Acl, B, C, D);

%check step response
%step(syscl)

%Solve for Kr (scaling gain term)
Kdc = dcgain(syscl)
Kr = 1/Kdc

%scale the input of the closed loop system
syscl_scaled = ss(Acl, B*Kr, C, D);
step(syscl_scaled)