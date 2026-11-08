function [Iqs, Ids, Te, omega_m, theta_e, theta_m] = pmsm(Lq_in,Ld_in,Lqd_in,Ldq_in,Lambda_d,Vds, Vqs, Iqs_prev, Ids_prev, T_L, omega_m_prev, theta_e_prev, theta_m_prev)
    
    % --- Electrical Parameters ---
    rs = 0.22;
    Lq = Lq_in*0.001;
    Ld = Ld_in*0.001;
    Lqd = Lqd_in*0.001;
    Ldq = Ldq_in*0.001; 
    Lambda_m = Lambda_d - Ld*Ids_prev - Ldq*Iqs_prev;
    P = 8;
    Ts = 5e-6; % Sampling time
    
    % --- Mechanical Parameters ---
    J = 0.003617;   % Rotor + Load Inertia (kg*m^2)
    B = 9.444e-05;  % Viscous friction (N*m*s)
    
    % 1. Calculate electrical speed from the mechanical speed
    omega_r = omega_m_prev * (P / 2);
    
    % --- NEW: Discrete Integration for Electrical Angle ---
    theta_e_raw = theta_e_prev + Ts * omega_r;
    theta_e = mod(theta_e_raw, 2*pi);

    theta_m_raw = theta_m_prev + Ts * omega_m_prev;
    theta_m = mod(theta_m_raw, 2*pi);
    
    % 1. Calculate the determinant of the inductance matrix
    Delta_L = (Ld * Lq) - (Ldq * Lqd);
    
    % 2. Calculate the 'effective' voltage for both axes using previous states (Fixed Capitalization)
    Vd_eff = Vds - (rs * Ids_prev) + omega_r * ((Lq * Iqs_prev) + (Lqd * Ids_prev));
    Vq_eff = Vqs - (rs * Iqs_prev) - omega_r * ((Ld * Ids_prev) + (Ldq * Iqs_prev) + Lambda_m);
    
    % 3. Calculate the coupled derivatives
    ids_dot = ((Lq * Vd_eff) - (Ldq * Vq_eff)) / Delta_L;
    iqs_dot = ((Ld * Vq_eff) - (Lqd * Vd_eff)) / Delta_L;
    
    % 4. Calculate the new currents using Forward Euler integration (Fixed Capitalization)
    Ids = Ids_prev + Ts * ids_dot;
    Iqs = Iqs_prev + Ts * iqs_dot;
    
    % 4. Electromagnetic Torque
    Te = (3/2) * (P/2) * (Lambda_m * Iqs + (Ld - Lq) * Iqs * Ids + Ldq*Iqs^2 - Lqd*Ids^2);
    
    % 5. Mechanical Dynamics (Forward Euler)
    omega_m_dot = (Te - T_L - B * omega_m_prev) / J;
    omega_m = omega_m_prev + Ts * omega_m_dot;
end