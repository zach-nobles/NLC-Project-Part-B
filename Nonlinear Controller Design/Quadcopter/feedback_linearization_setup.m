%% Take two at Lie derivatives
G = g_fun(states, u_con);
yvec = [states(1); states(2); states(3); states(6)];
yvec_dot = statesdot([1;2;3;6]);

Lf_h = jacobian(yvec, states) * f_fun(states);
Lf_g = jacobian(yvec, u_con); % zero matrix
% Lf_h + Lf_g * u = state derivatives for y

Lf2_h = jacobian(Lf_h, states) * f_fun(states); % this is b(x)
Lg_Lf_h = jacobian(Lf_h, states) * G; % this is delta, or E
% Lf2_h + Lg_Lf_h * u_con = y''

%%
% Augment state to include gamma, sigma
% Where u1  (actual input) = gamma, gammadot = sigma, sigmadot = u1_bar (original)
syms gam sig
states_aug = [states; gam; sig];

state_dot_aug = nonlinear_dynamics_aug(states_aug, u_con);
G_aug = g_fun_aug(states_aug, u_con);

%%

Lf_h_aug = jacobian(yvec, states_aug) * f_fun_aug(states_aug); % still no u
Lf_g_aug = jacobian(yvec, u_con); % still zero
% Lf_h_aug + Lf_g_aug*u_con = yvec_dot

Lf2_h_aug = jacobian(Lf_h_aug, states_aug) * f_fun_aug(states_aug); 
Lg_Lf_h_aug = jacobian(Lf_h_aug, states_aug) * G_aug; 
% 3/4 rows of Lg_Lf_h_aug are zero, keep going

Lf3_h_aug = jacobian(Lf2_h_aug, states_aug) * f_fun_aug(states_aug);
Lg_Lf2_h_aug = jacobian(Lf2_h_aug, states_aug) * G_aug;
% 1 column of all zeros, keep going

Lf4_h_aug = jacobian(Lf3_h_aug, states_aug) * f_fun_aug(states_aug);
Lg_Lf3_h_aug = jacobian(Lf3_h_aug, states_aug) * G_aug;
% Finally has rank 4, Lg_Lf3_h_aug is most of our delta

% Take r1 = r2 = r3 = 4, r4 = 2
b_vec = simplify([Lf4_h_aug(1:3); Lf2_h_aug(4)]);
delta_mat = simplify([Lg_Lf3_h_aug(1:3, :); Lg_Lf_h_aug(4, :)]);


