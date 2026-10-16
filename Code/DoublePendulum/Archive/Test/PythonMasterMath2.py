import numpy as np
import sympy as sp
import control as ctrl
import time
import os
import dill


print("program started")

#define symbols and symbol properties
t,g,l,lFull,l2,m1,m2,mcart,B_cart_drag = sp.symbols('t g l lFull l2 m1 m2 mcart B_cart_drag', positive = True)
theta = sp.Function('theta')(t) #define theta as a function of t
theta2 = sp.Function('theta2')(t)
x = sp.Function('x')(t) #define x as a function of t
I,I2,F,T_drag = sp.symbols('I I2 F T_drag', real = True)

print("symbols defined")

#Sample Period
Ts = 1/100

ArrayPrec = 5 #how many decimals are sent to the Text file

#0.5N rolling friction

#actual system values
#pendulum 1
length = 0.2 #total length of first pendulum
lval = 0.087 #in meters (to center of mass of first pendulum)
Ival = 0.00123324992  #rotational inertia
m1val = 0.207 #in kg
#pendulum 2
length2 = 0.25 #total length
mtotal2 = 0.206 #pendulum mass
lval2 = 0.106 #length to center of mass
Ival2 = 0.00197563667 #rotational inertia
#cart
mcartVal = 0.969 - 0.059 #mass of cart in kg
B_cart_dragVal = 0.0 #cart drag coefficient
gval = 9.81 #gravitational constant
T_dragVal = 0.0 #rotational drag force, NOT MODELED CORRECTLY, LEAVE AS 0

#substitutions
vals = {m1:m1val,m2:mtotal2,mcart:mcartVal,l:lval,lFull:length,l2:lval2,g:gval,I:Ival,I2:Ival2,T_drag:T_dragVal,B_cart_drag:B_cart_dragVal}

#Kinetic energy equation
T =	(
    (1/2)*(mcart + m1 + m2)*x.diff(t)**2
    + (1/2)*(m1*l**2 + m2*lFull**2 + I)*theta.diff(t)**2
    + (1/2)*(m2*l2**2 + I2)*theta2.diff(t)**2
    + (m1*l + m2*lFull)*x.diff(t)*theta.diff(t)*sp.cos(theta)
    + m2*l2*x.diff(t)*theta2.diff(t)*sp.cos(theta2)
    + (m2*lFull*l2)*sp.cos(theta-theta2)*theta.diff(t)*theta2.diff(t)
    )
#Potential energy equation
U = m1*g*l*sp.cos(theta) + m2*g*(lFull*sp.cos(theta) + l2*sp.cos(theta2))

#Lagrange stuff
L = T - U

#set up symbols for 2nd derivatives of states
x2div,theta2div,theta22div = sp.symbols('x2div theta2div theta22div', real = True)

# Check for abstract model
if not os.path.exists("symbolic_model2.pkl"): #if no model, derive it
    print("No .pkl, deriving equations ...")
        
    EL_x = sp.Eq(
        sp.diff(sp.diff(L, x.diff(t)), t) - sp.diff(L, x),
        F - B_cart_drag * x.diff(t)
    )

    EL_theta = sp.Eq(
        sp.diff(sp.diff(L, theta.diff(t)), t) - sp.diff(L, theta),
        -T_drag
    )

    EL_theta2 = sp.Eq(
        sp.diff(sp.diff(L, theta2.diff(t)), t) - sp.diff(L, theta2),
        -T_drag
    )

    print("Basic equations defined")

    #move everything to one side of the equation (set equal to 0)
    Eq1 = EL_x.lhs - EL_x.rhs
    Eq2 = EL_theta.lhs - EL_theta.rhs
    Eq3 = EL_theta2.lhs - EL_theta2.rhs
    print("Equations set equal to 0")
    
    #substitute in new symbols in place of accelerations
    Eq1 = Eq1.subs({sp.Derivative(x,t,2):x2div, sp.Derivative(theta,t,2):theta2div, sp.Derivative(theta2,t,2):theta22div})
    Eq2 = Eq2.subs({sp.Derivative(x,t,2):x2div, sp.Derivative(theta,t,2):theta2div, sp.Derivative(theta2,t,2):theta22div})
    Eq3 = Eq3.subs({sp.Derivative(x,t,2):x2div, sp.Derivative(theta,t,2):theta2div, sp.Derivative(theta2,t,2):theta22div})
    print("abstract symbols substituted in for 2nd derivatives")

    #solve equations for accelerations
    Subs = sp.solve([Eq1, Eq2, Eq3], [x2div, theta2div, theta22div], simplify = True)
    print("Equations solved for derivatives")
    
    # Save the solve results and the Jacobians
    with open("symbolic_model2.pkl", "wb") as f:
        dill.dump({'Subs': Subs}, f)
        
else: #Load the model if available
    print("Loading stored data...")
    with open("symbolic_model2.pkl", "rb") as f:
        data = dill.load(f)
        Subs = data['Subs']
    
#substitute in symbols for each state
Y1,Y2,Y3,Y4,Y5,Y6 = sp.symbols('Y1 Y2 Y3 Y4 Y5 Y6')
StateSubs = {theta:Y1, sp.Derivative(theta,t):Y2, theta2:Y3, sp.Derivative(theta2,t):Y4, x:Y5, sp.Derivative(x,t):Y6}
F1 = Subs[theta2div].subs(StateSubs)
F2 = Subs[theta22div].subs(StateSubs)
F3 = Subs[x2div].subs(StateSubs)
print("State symbols substituted in")

#combine into a matrix (non-linear state space model), and create state matrix (Y)
NonLinMod = sp.Matrix([Y2, F1, Y4, F2, Y6, F3])
Y = sp.Matrix([Y1, Y2, Y3, Y4, Y5, Y6])
print("Non-linear model created")

#Find A and B matrices
A_abstract = NonLinMod.jacobian(Y)
B_abstract = NonLinMod.diff(F)
print("linearization completed")

#Substitute in values and states for linearization
Equilibrium = {Y1:0,Y2:0,Y3:0,Y4:0,Y5:0,Y6:0} #all states are 0 at equilibrium
A = A_abstract.subs(vals).subs(Equilibrium)
B = B_abstract.subs(vals).subs(Equilibrium)
print("Values and equilibrium points substituted in")

C = sp.Matrix([ #manual input
    [1, 0, 0, 0, 0, 0],
    [0, 0, 1, 0, 0, 0],
    [0, 0, 0, 0, 1, 0],])
D = sp.Matrix([0, 0, 0]) #manual input

#convert to a ss system, and discretize
ssCont = ctrl.ss(A, B, C, D)
ssDisc = ctrl.c2d(ssCont, Ts)

# #Testing
# # Debugging Block
# print("Is A finite:", np.isfinite(ssDisc.A).all())
# print("Is B finite:", np.isfinite(ssDisc.B).all())
# 
# # Check for controllability
# C_mat = ctrl.ctrb(ssDisc.A, ssDisc.B)
# print("Controllability Rank:", np.linalg.matrix_rank(C_mat))
# 
# # Eigenvalue check
# print("Eigenvalues of A (continuous):", np.linalg.eigvals(ssCont.A))
    

#-----TUNING VALUES-----#
#calculate lqr and kalman filter values
#K, S, E = ctrl.lqr(A, B, sp.diag(10,2,1,1), 0.5) #continuous
start = time.time()
# Kd, Sd, Ed = ctrl.dlqr(ssDisc, sp.diag(200,5,100,5,30,0.5), 0.1) #previous tuning values
Kd, Sd, Ed = ctrl.dlqr(ssDisc, sp.diag(100,1,100,1,20,10), 0.1)
end = time.time()
print("LQG gain matrix calculated", (end-start)*1000)

#QN and RN are multiplied/divided by Ts to discretize them. MATLAB does this internally
start = time.time()
# Ld, Pd, Edkalm = ctrl.dlqe(ssDisc.A, sp.diag(1,1,1,1,1,1), ssDisc.C, Ts*sp.diag(0.2,0.001,0.2,0.001,0.2,0.001), (0.01/Ts)*sp.diag(1,1,1))
Ld, Pd, Edkalm = ctrl.dlqe(ssDisc.A, sp.diag(1,1,1,1,1,1), ssDisc.C, sp.diag(0.2,0.001,0.2,0.001,0.2,0.001), 0.01*sp.diag(1,1,1))
end = time.time()
print("Kalman matrices generated", (end-start)*1000)

# --- FINAL FILE OUTPUT SECTION ---
try:
    print("Writing to file...")
    
    # 1. Calculate the absolute path
    script_dir = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(script_dir, "control_matrices2.txt")

    # 2. Use the 'file_path' variable here, NOT the string 'control_matrices.txt'
    with open(file_path, 'w') as f:
        f.write("=== CONTROL SYSTEM DESIGN DATA ===\n\n")
        
        f.write("--- Matrix A (Linearized) ---\n")
        f.write(np.array2string(np.array(ssDisc.A.tolist(), dtype=float), precision=ArrayPrec) + "\n\n")
        
        f.write("--- Matrix B ---\n")
        f.write(np.array2string(np.array(ssDisc.B.tolist(), dtype=float), precision=ArrayPrec) + "\n\n")

        f.write("--- Matrix C ---\n")
        f.write(np.array2string(np.array(ssDisc.C.tolist(), dtype=float), precision=ArrayPrec) + "\n\n")

        f.write("--- Matrix D ---\n")
        f.write(np.array2string(np.array(ssDisc.D.tolist(), dtype=float), precision=ArrayPrec) + "\n\n")
        
        f.write("--- LQR Gain (Kd) ---\n")
        f.write(np.array2string(np.array(Kd), precision=ArrayPrec) + "\n\n")
        
        f.write("--- Kalman Gain (Ld) ---\n")
        f.write(np.array2string(np.array(Ld), precision=ArrayPrec) + "\n")
        
    # 3. Print the full path so you can click it in the terminal
    print(f"Successfully saved to: {file_path}")

except Exception as e:
    print(f"FAILED to write file. Error: {e}")

print(A)