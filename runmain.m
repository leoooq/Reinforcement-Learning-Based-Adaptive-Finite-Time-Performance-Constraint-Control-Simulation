% Main script for simulation
clear; clc; close all;

% Simulation Time
t_final = 20; % As per figures, adjust as needed
t_span = [0, t_final];

% System Parameters (from paper's example - some may need tuning for new system)
delta_1 = 1;
delta_2 = 19/21;
delta_3 = 17/21; 
pi_const = 40/21; 

% Initial Conditions for the new system
chi0 = [0.2; -0.5]; 
U_hat1_0 = 0.1; % Since f1=0, can start small
U_hat2_0 = 1.0; % May need adjustment
num_nodes_val = 5; % Define num_nodes here for clarity


% Wa1_0 = 0.1 * ones(num_nodes_val, 1); % Initialize to small values
% Wc1_0 = 0.1 * ones(num_nodes_val, 1);
% Wa2_0 = 0.1 * ones(num_nodes_val, 1);
% Wc2_0 = 0.1 * ones(num_nodes_val, 1);
init_range = 0.4;
Wa1_0 = -init_range + (2 * init_range) * rand(num_nodes_val, 1);
Wc1_0 = -init_range + (2 * init_range) * rand(num_nodes_val, 1);
Wa2_0 = -init_range + (2 * init_range) * rand(num_nodes_val, 1);
Wc2_0 = -init_range + (2 * init_range) * rand(num_nodes_val, 1);

initial_conditions = [chi0; U_hat1_0; U_hat2_0; Wa1_0; Wc1_0; Wa2_0; Wc2_0];

% Design Parameters (These will likely need tuning for the new system)
gamma_1 = 20; gamma_2 = 25; % Adjusted, may need further tuning
sigma_1 = 25; sigma_2 = 20;  % Adjusted
gamma_a1 = 1.3; gamma_a2 = 2.25; % Adjusted
gamma_c1 = 1.2; gamma_c2 = 2.15; % Adjusted
eta_bar1 = 65; eta_bar2 = 60;   % Adjusted
zeta_1 = 1.8; zeta_2 = 1.0;    % Adjusted
tau_1 = 5; tau_2 = 5;         % Adjusted

% Performance function parameters (κ_i(t))
k01 = pi/2; k_inf1 = 0.05; l1 = 1.0; % Adjusted, make k01 reasonable for initial error
k02 = pi; k_inf2 = 0.1; l2 = 0.8;   % Adjusted

% RBF NN parameters
phi_width_sq = sqrt(2)^2; 
mu_centers = linspace(-2, 2, num_nodes_val)'; % Adjusted range for xi if it stays smaller

% Store parameters in a struct
params = struct(...
    'delta_1', delta_1, 'delta_2', delta_2, 'delta_3', delta_3, 'pi_const', pi_const, ...
    'gamma_1', gamma_1, 'gamma_2', gamma_2, 'sigma_1', sigma_1, 'sigma_2', sigma_2, ...
    'gamma_a1', gamma_a1, 'gamma_a2', gamma_a2, 'gamma_c1', gamma_c1, 'gamma_c2', gamma_c2, ...
    'eta_bar1', eta_bar1, 'eta_bar2', eta_bar2, 'zeta_1', zeta_1, 'zeta_2', zeta_2, ...
    'tau_1', tau_1, 'tau_2', tau_2, ...
    'k01', k01, 'k_inf1', k_inf1, 'l1', l1, ...
    'k02', k02, 'k_inf2', k_inf2, 'l2', l2, ...
    'num_nodes', num_nodes_val, 'phi_width_sq', phi_width_sq, 'mu_centers', mu_centers, ...
    'Im_sigma_bar_val', 0.01, ...
    'small_phi_val', 0.1 ... % For terms where phi might be zero if f is zero
);
params.Im_sigma_bar1 = params.Im_sigma_bar_val * eye(params.num_nodes);
params.Im_sigma_bar2 = params.Im_sigma_bar_val * eye(params.num_nodes);

% Solve ODE
options = odeset('RelTol',1e-4,'AbsTol',1e-6, 'Stats','off'); 
disp('Starting ODE solver...');
[T, X_ode] = ode45(@(t, x_ode) system_dynamics(t, x_ode, params), t_span, initial_conditions, options);
disp('ODE solver finished.');

% Extract results
chi1_t = X_ode(:, 1);
chi2_t = X_ode(:, 2);
U_hat1_t = X_ode(:, 3);
U_hat2_t = X_ode(:, 4);
idx_start = 5;
Wa1_t = X_ode(:, idx_start : idx_start+params.num_nodes-1); idx_start = idx_start+params.num_nodes;
Wc1_t = X_ode(:, idx_start : idx_start+params.num_nodes-1); idx_start = idx_start+params.num_nodes;
Wa2_t = X_ode(:, idx_start : idx_start+params.num_nodes-1); idx_start = idx_start+params.num_nodes;
Wc2_t = X_ode(:, idx_start : idx_start+params.num_nodes-1);

% Calculate other variables for plotting
T_vec = T(:); 
yd_t = -0.4 * exp(-0.3*T_vec) + 0.6 * sin(0.5*T_vec) + 0.5 * cos(0.7*T_vec);
dyd_dt_t = 0.12 * exp(-0.3*T_vec) + 0.3 * cos(0.5*T_vec) - 0.35 * sin(0.7*T_vec);

k1_t = (params.k01 - params.k_inf1) * exp(-params.l1 * T_vec) + params.k_inf1;
k2_t = (params.k02 - params.k_inf2) * exp(-params.l2 * T_vec) + params.k_inf2;

z1_t = chi1_t - yd_t;
xi1_plot_t = z1_t ./ k1_t; 

alpha_hat1_t = zeros(length(T), 1);
u_t = zeros(length(T), 1);
z2_t = zeros(length(T),1);
xi2_plot_t = zeros(length(T),1); 

disp('Calculating control inputs for plotting...');
for k = 1:length(T)
    t_k = T(k);
    chi_k = [chi1_t(k); chi2_t(k)];
    U_hat1_k = U_hat1_t(k);
    U_hat2_k = U_hat2_t(k);
    Wa1_k = Wa1_t(k,:)'; 
    Wc1_k = Wc1_t(k,:)'; 
    Wa2_k = Wa2_t(k,:)'; 
    Wc2_k = Wc2_t(k,:)'; 
    
    yd_k = -0.4 * exp(-0.3*t_k) + 0.6 * sin(0.5*t_k) + 0.5 * cos(0.7*t_k);
    dyd_dt_k = 0.12 * exp(-0.3*t_k) + 0.3 * cos(0.5*t_k) - 0.35 * sin(0.7*t_k);

    kappa1_k = (params.k01 - params.k_inf1) * exp(-params.l1 * t_k) + params.k_inf1;
    dkappa1_dt_k = -params.l1 * (params.k01 - params.k_inf1) * exp(-params.l1 * t_k);
    kappa2_k = (params.k02 - params.k_inf2) * exp(-params.l2 * t_k) + params.k_inf2;

    z1_k = chi_k(1) - yd_k;
    xi1_k = z1_k / kappa1_k;
    xi1_k = max(min(xi1_k, 0.99999), -0.99999);

    phi1_chi1_k = params.small_phi_val; % Since f1=0
    
    den_1_xi_term_k = (1-xi1_k);
    if abs(den_1_xi_term_k) < 1e-9 
        den_1_xi_term_k = sign(den_1_xi_term_k) * 1e-9 + (1e-9 * (den_1_xi_term_k==0)); 
    end
    Den_1_full_k = den_1_xi_term_k^3 * kappa1_k^2;

    T_1_k = (phi1_chi1_k)^params.pi_const / Den_1_full_k;
    N_1_k = 1 / Den_1_full_k;

    S1_output_k = rbf_eval(xi1_k, params.mu_centers, params.phi_width_sq, params.num_nodes);
    
    term_U1 = T_1_k * U_hat1_k + N_1_k;
    term_Wa1 = Wa1_k' * S1_output_k;
    alpha_hat1_t(k) = -signed_power(xi1_k, 2*params.delta_1) * term_U1 ...
                      - (1/(2*params.zeta_1*kappa1_k)) * term_Wa1 ...
                      + dkappa1_dt_k * xi1_k + dyd_dt_k ...
                      - (params.eta_bar1/params.zeta_1) * (1/kappa1_k) * signed_power(xi1_k, 2*params.delta_1) * signed_power(den_1_xi_term_k, params.delta_1) ...
                      - (params.tau_1 * signed_power(xi1_k, 2*params.delta_1-1)) / (params.zeta_1 * signed_power(den_1_xi_term_k,3) * kappa1_k);
    
    z2_k = signed_power(chi_k(2), 1/params.delta_2) - signed_power(alpha_hat1_t(k), 1/params.delta_2);
    z2_t(k)=z2_k;
    xi2_k = z2_k / kappa2_k;
    xi2_plot_t(k) = xi2_k; 
    xi2_k = max(min(xi2_k, 0.99999), -0.99999);

    phi2_chi_bar2_k = abs(sin(chi_k(1))) + abs(chi_k(2)) + params.small_phi_val; % Using a more direct bound for f2
    
    den_2_xi_term_k = (1-xi2_k);
    if abs(den_2_xi_term_k) < 1e-9
        den_2_xi_term_k = sign(den_2_xi_term_k) * 1e-9 + (1e-9 * (den_2_xi_term_k==0));
    end
    Den_2_full_k = den_2_xi_term_k^3 * kappa2_k^2; 

    T_2_k = (phi2_chi_bar2_k)^params.pi_const / Den_2_full_k; 
    N_2_k = 1 / Den_2_full_k; 

    S2_output_k = rbf_eval(xi2_k, params.mu_centers, params.phi_width_sq, params.num_nodes);
    
    term_U2 = T_2_k * U_hat2_k + N_2_k; 
    term_Wa2 = Wa2_k' * S2_output_k;
    u_t(k) = -signed_power(xi2_k, params.delta_3) * term_U2 ...
             - (1/(2*params.zeta_2*kappa2_k)) * term_Wa2 ...
             - (params.eta_bar2/params.zeta_2) * (1/kappa2_k) * signed_power(xi2_k, params.delta_3) * signed_power(den_2_xi_term_k, (3 - 3*params.pi_const/2) ) ...
             - (params.tau_2 * signed_power(xi2_k, 2-params.delta_2)) / (params.zeta_2 * signed_power(den_2_xi_term_k,3) * (kappa2_k^params.delta_2));
end
disp('Plotting results...');

% Plotting
figure; subplot(2,1,1); plot(T, chi1_t, 'b', T, yd_t, 'r--');
legend('\chi_1 (x_1)', 'y_d (x_r)'); xlabel('Time (s)'); ylabel('States'); title('Fig 1: x_1 and Reference x_r'); grid on;
subplot(2,1,2); plot(T, chi2_t, 'b');
xlabel('Time (s)'); ylabel('\chi_2 (x_2)'); title('Fig 2: State x_2'); grid on;

figure; subplot(2,1,1); plot(T, z1_t, 'b', T, k1_t, 'r--', T, -k1_t, 'r--');
legend('z_1 = x_1 - x_r', '\kappa_1', '-\kappa_1'); xlabel('Time (s)'); ylabel('Error z_1'); title('Fig 3: Error z_1 and Bound \kappa_1'); grid on;
ylim_val1 = max(abs(z1_t))*1.2; if ylim_val1 < params.k01*1.1, ylim_val1 = params.k01*1.1; end; if ylim_val1 == 0, ylim_val1 = 1; end
ylim([-ylim_val1, ylim_val1]);
subplot(2,1,2); plot(T, z2_t, 'b', T, k2_t, 'r--', T, -k2_t, 'r--');
legend('z_2', '\kappa_2', '-\kappa_2'); xlabel('Time (s)'); ylabel('Error z_2'); title('Fig 4: Error z_2 and Bound \kappa_2'); grid on;
ylim_val2 = max(abs(z2_t))*1.2; if ylim_val2 < params.k02*1.1, ylim_val2 = params.k02*1.1; end; if ylim_val2 == 0, ylim_val2 = 1; end
ylim([-ylim_val2, ylim_val2]);

figure; subplot(2,1,1); plot(T, u_t, 'b');
xlabel('Time (s)'); ylabel('Control Input u'); title('Fig 5: Controller u'); grid on;
subplot(2,1,2); plot(T, alpha_hat1_t, 'g');
xlabel('Time (s)'); ylabel('Virtual Control \alpha_1'); title('Virtual Control \alpha_1'); grid on;

figure; subplot(2,1,1); plot(T, U_hat1_t); ylabel('Û_1'); title('Fig 6: Û_i responses'); grid on;
subplot(2,1,2); plot(T, U_hat2_t); ylabel('Û_2'); xlabel('Time (s)'); grid on;

figure; subplot(2,2,1); plot(T, Wa1_t); title('W_{a1}'); xlabel('Time (s)'); grid on; legend_str_wa1 = cellstr(num2str((1:params.num_nodes)', 'W_{a1,%d}')); legend(legend_str_wa1);
subplot(2,2,2); plot(T, Wa2_t); title('W_{a2}'); xlabel('Time (s)'); grid on; legend_str_wa2 = cellstr(num2str((1:params.num_nodes)', 'W_{a2,%d}')); legend(legend_str_wa2);
subplot(2,2,3); plot(T, Wc1_t); title('W_{c1}'); xlabel('Time (s)'); grid on; legend_str_wc1 = cellstr(num2str((1:params.num_nodes)', 'W_{c1,%d}')); legend(legend_str_wc1);
subplot(2,2,4); plot(T, Wc2_t); title('W_{c2}'); xlabel('Time (s)'); grid on; legend_str_wc2 = cellstr(num2str((1:params.num_nodes)', 'W_{c2,%d}')); legend(legend_str_wc2);
sgtitle('Fig 7: NN Weights Responses');

disp('Simulation and plotting complete.');

% --- ODE Function ---
function dXdt = system_dynamics(t, X_ode, params)
    % Unpack states
    chi = X_ode(1:2);
    U_hat1 = X_ode(3);
    U_hat2 = X_ode(4);
    idx_start_ode = 5;
    Wa1 = X_ode(idx_start_ode : idx_start_ode+params.num_nodes-1); 
    idx_start_ode = idx_start_ode+params.num_nodes;
    Wc1 = X_ode(idx_start_ode : idx_start_ode+params.num_nodes-1); 
    idx_start_ode = idx_start_ode+params.num_nodes;
    Wa2 = X_ode(idx_start_ode : idx_start_ode+params.num_nodes-1); 
    idx_start_ode = idx_start_ode+params.num_nodes;
    Wc2 = X_ode(idx_start_ode : idx_start_ode+params.num_nodes-1);

    % Desired trajectory for the new system
    yd = -0.4 * exp(-0.3*t) + 0.6 * sin(0.5*t) + 0.5 * cos(0.7*t);
    dyd_dt = 0.12 * exp(-0.3*t) + 0.3 * cos(0.5*t) - 0.35 * sin(0.7*t);

    % Performance functions
    kappa1 = (params.k01 - params.k_inf1) * exp(-params.l1 * t) + params.k_inf1;
    dkappa1_dt = -params.l1 * (params.k01 - params.k_inf1) * exp(-params.l1 * t);
    kappa2 = (params.k02 - params.k_inf2) * exp(-params.l2 * t) + params.k_inf2;

    % Error variables
    z1 = chi(1) - yd;
    xi1 = z1 / kappa1;
    xi1 = max(min(xi1, 0.99999), -0.99999); 

    % System's nonlinear functions
    f1_chi1 = 0; 
    f2_chi_bar2 = -4.9 * sin(chi(1)) - chi(2); 

    % Bounding functions phi_i
    phi1_chi1 = params.small_phi_val; % Since f1=0
    
    den_1_xi_term = (1-xi1);
    if abs(den_1_xi_term) < 1e-9 
        den_1_xi_term = sign(den_1_xi_term) * 1e-9 + (1e-9 * (den_1_xi_term==0)); % Add small if exactly zero
    end
    Den_1_full = den_1_xi_term^3 * kappa1^2;

    T_1 = (phi1_chi1)^params.pi_const / Den_1_full;
    N_1 = 1 / Den_1_full;
    
    S1_output = rbf_eval(xi1, params.mu_centers, params.phi_width_sq, params.num_nodes);
    
    term_U1 = T_1 * U_hat1 + N_1;
    term_Wa1 = Wa1'* S1_output; 
    alpha_hat1 = -signed_power(xi1, 2*params.delta_1) * term_U1 ...
                 - (1/(2*params.zeta_1*kappa1)) * term_Wa1 ...
                 + dkappa1_dt * xi1 + dyd_dt ...
                 - (params.eta_bar1/params.zeta_1) * (1/kappa1) * signed_power(xi1, 2*params.delta_1) * signed_power(den_1_xi_term, params.delta_1) ...
                 - (params.tau_1 * signed_power(xi1, 2*params.delta_1-1)) / (params.zeta_1 * signed_power(den_1_xi_term,3) * kappa1);

    z2 = signed_power(chi(2), 1/params.delta_2) - signed_power(alpha_hat1, 1/params.delta_2);
    xi2 = z2 / kappa2;
    xi2 = max(min(xi2, 0.99999), -0.99999);

    phi2_chi_bar2 = abs(sin(chi(1))) + abs(chi(2)) + params.small_phi_val; % More direct bound for f2
    
    den_2_xi_term = (1-xi2);
    if abs(den_2_xi_term) < 1e-9
        den_2_xi_term = sign(den_2_xi_term) * 1e-9 + (1e-9 * (den_2_xi_term==0));
    end
    Den_2_full = den_2_xi_term^3 * kappa2^2;

    T_2 = (phi2_chi_bar2)^params.pi_const / Den_2_full;
    N_2 = 1 / Den_2_full;

    S2_output = rbf_eval(xi2, params.mu_centers, params.phi_width_sq, params.num_nodes);
    
    term_U2 = T_2 * U_hat2 + N_2; 
    term_Wa2 = Wa2'* S2_output;
    u = -signed_power(xi2, params.delta_3) * term_U2 ...
        - (1/(2*params.zeta_2*kappa2)) * term_Wa2 ...
        - (params.eta_bar2/params.zeta_2) * (1/kappa2) * signed_power(xi2, params.delta_3) * signed_power(den_2_xi_term, (3 - 3*params.pi_const/2) ) ...
        - (params.tau_2 * signed_power(xi2, 2-params.delta_2)) / (params.zeta_2 * signed_power(den_2_xi_term,3) * (kappa2^params.delta_2));

    dchi1_dt = chi(2) + f1_chi1; 
    dchi2_dt = u + f2_chi_bar2;
    
    % Adaptive laws
    Tau_1_for_Uhat_update = (phi1_chi1)^params.pi_const / (den_1_xi_term^3 * kappa1^2); 
    dU_hat1_dt = params.gamma_1 * signed_power(xi1, params.pi_const) * Tau_1_for_Uhat_update / (den_1_xi_term^3 * kappa1) ...
                 - params.sigma_1 * U_hat1;
    
    term_S1S1T_Im1 = S1_output * S1_output' + params.Im_sigma_bar1; 
    dWa1_dt = -params.gamma_a1 * term_S1S1T_Im1 * (Wa1 - Wc1) - params.gamma_c1 * term_S1S1T_Im1 * Wc1; 
    dWc1_dt = -params.gamma_c1 * term_S1S1T_Im1 * Wc1;

    Tau_2_for_Uhat_update = (phi2_chi_bar2)^params.pi_const / (den_2_xi_term^3 * kappa2^2); 
    dU_hat2_dt = params.gamma_2 * signed_power(xi2, params.pi_const) * Tau_2_for_Uhat_update / (den_2_xi_term^3 * (kappa2^params.delta_2) ) ...
                 - params.sigma_2 * U_hat2;

    term_S2S2T_Im2 = S2_output * S2_output' + params.Im_sigma_bar2; 
    dWa2_dt = -params.gamma_a2 * term_S2S2T_Im2 * (Wa2 - Wc2) - params.gamma_c2 * term_S2S2T_Im2 * Wc2; 
    dWc2_dt = -params.gamma_c2 * term_S2S2T_Im2 * Wc2;

    dXdt = [dchi1_dt; dchi2_dt; dU_hat1_dt; dU_hat2_dt; dWa1_dt; dWc1_dt; dWa2_dt; dWc2_dt];
end

% --- RBF Neural Network Evaluation (takes scalar input) ---
function S_output = rbf_eval(scalar_node_input, centers, width_sq, num_nodes)
    S_output = zeros(num_nodes, 1);
    for j = 1:num_nodes
        S_output(j) = exp(-(scalar_node_input - centers(j)).^2 / width_sq); 
    end
end

% --- Helper for signed power ---
function y = signed_power(x, p)
    if x == 0
        if p < 0
            y = Inf * sign(x); % Or handle as error, or return large number
                            % For robustness, one might return a very large signed number
                            % y = sign(x) * 1e18; 
                            % However, saturation of xi should prevent x=0 with p<0 in critical terms.
        elseif p == 0
            y = 1; % Convention 0^0 = 1
        else % p > 0
            y = 0;
        end
    else
        y = sign(x) * (abs(x)^p);
    end
end