
function state_dot_aug = nonlinear_dynamics_aug(state, controls)
% state = [x y z phi theta psi u v w p q r gamma sigma]'
% controls = [T taux tauy tauz]'

state_dot = nonlinear_dynamics(state(1:12), [state(13); controls(2:4)]);

state_dot_aug = [state_dot; state(14); controls(1)];
end