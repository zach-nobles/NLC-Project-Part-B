%% G(x) in xdot = f(x) + G(x)*u
function G_aug = g_fun_aug(state_aug, controls)
% state = [x y z phi theta psi u v w p q r gamma sigma]'
% controls = [T taux tauy tauz]'

u_component = nonlinear_dynamics_aug(state_aug, controls) - f_fun_aug(state_aug);
G_aug = jacobian(u_component, controls);

end