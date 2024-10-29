import control as ct
import matplotlib.pyplot as plt

s = ct.TransferFunction.s

# System parameters
m = 0.5
l = 1
k = 0.5
J = m*l*l
g = 9.81
#The initial parameters (m, l, k, J, g) define a system that may represent its component:
# m: Mass of the object (0.5 kg).
# l: Length or distance to the center of mass (1 meter).
# k: Damping coefficient (0.5 N·s).
# J: Moment of inertia, defined as m*l*l.
# g: Acceleration due to gravity (9.81 m/s²).



# Non-linear system:
# tau = J*ddtheta + k*dtheta + m*g*l*sin(theta)

# Linearised system in theta = pi/2:
# tau = J*ddtheta + k*dtheta - m*g*l*theta

# Transfer function
P = 1/(J*s**2 + k*s - m*g*l)

# where τ is torque, θ’’ and θ’ are angular acceleration and velocity, 
# and θ is the angular displacement. 
# To simplify, the system is linearized around θ=π/2

# Plot root-locus of P
plt.figure()
ct.root_locus(P)

#The root locus plot visually represents how the poles of the closed-loop system vary with different 
#feedback gains, indicating stability. By running “ct.root_locus(P)”, 
#we analyze the stability of “P(s)” and its response to control.

# Controller transfer function - lead-lag
Tc = 0.2
z = -3
K = 2
C = K*(s-z)/(Tc*s+1)

# The control function, C(s), applies a lead-lag controller, a design that shifts poles and zeros to achieve desired stability and response:
# Tc: Time constant (0.2 s) that affects system response speed.
# z: Zero location (-3) that affects transient behavior.
# K: Gain (2) that scales the control effect.


# Close-loop transfer function
G = ct.feedback(C*P, 1)

# Plot root-locus of C*P
plt.figure()
ct.root_locus(C*P)

#The closed-loop transfer function, “G = ct.feedback(C*P, 1)”, 
#represents the overall system response when feedback control is applied. 
#“1” represents a unit feedback. This setup helps maintain the desired output and 
#reduces errors by adjusting the controller’s response.

# Plot Nyquist diagram
plt.figure()
ct.nyquist_plot(C*P)

# The Nyquist plot helps examine system stability by mapping the frequency response of” C*P”. Peaks near or 
# encircling the critical point (-1,0) may indicate instability or oscillations in the system. 
# Running “ct.nyquist_plot(C*P)” visualizes the frequency response and helps determine if “G” has an 
#appropriate stability margin.

# Ideal step response
t, yout = ct.step_response(G)
plt.figure()
plt.plot(t, yout)
plt.grid()
plt.xlabel("Time [s]")

# The step response represents how the closed-loop system reacts to a unit step input, 
# simulating a sudden disturbance or desired setpoint shift. “t, yout = ct.step_response(G)” will generate time-domain data. 
# Plotting this response shows how “G” reaches steady state, 
# which is crucial in assessing performance criteria like settling time, overshoot, and steady-state error.

# Show plots
plt.show()