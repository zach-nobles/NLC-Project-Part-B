%% G(x) in xdot = f(x) + G(x)*u
function G = g_fun(state, controls)
% state = [x y z phi theta psi xdot ydot zdot p q r]'
% controls = [T taux tauy tauz]'

u_component = nonlinear_dynamics(state, controls) - f_fun(state);
G = jacobian(u_component, controls);

end