%% f(x) in xdot = f(x) + G(x)*u
function xdot_state = f_fun(state)
% state = [x y z phi theta psi u v w p q r]'

xdot_state = nonlinear_dynamics(state, zeros(4, 1));
end

