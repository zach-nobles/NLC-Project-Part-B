%% f(x) in xdot = f(x) + G(x)*u
function xdot_state_aug = f_fun_aug(state_aug)
% state = [x y z phi theta psi u v w p q r gamma sigma]'

xdot_state_aug = nonlinear_dynamics_aug(state_aug, zeros(4, 1));
end

