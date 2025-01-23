function [dxdt,p] = EVModel(t, s)

global Kp Ki
    % Constants
    Rw = 0.3;       % Ohms
    Lw = 0.015;     % Henry
    Tm = 1.718;     % Weber transduction coefficient 
    M = 2200;       % Vehicle mass (kg)
    bT = 0.05;      % Drive shaft friction (Nms/rad)
    R = 0.2;        % Wheel radius (m)
    GR = 5;         % Gear ratio
    CR = 0.006;     % Rolling Resistance Coefficient
    g = 9.81;       % Acceleration due to gravity (m/s^2)
    CD = 0.32;      % Drag Coefficient
    rho = 1.21;     % Air density (kg/m^3)
    Af = 2.05;      % Frontal area (m^2)
    n = 1e-3;       % Small number for sgn approximation





    
    % Approximation for sgn function
    sgn_approx = @(v) v ./ (abs(v) + n);

    % State variables
    pL = s(1);      % Motor flux linkage
    pM = s(2);      % Vehicle momentum
    d_actual = s(3); % Distance traveled
    d_ref = s(4);    % Reference distance

    % Motor current
    i_in = pL / Lw;

    % Vehicle velocity (flow)
    v = pM / M;

    % Drag force
    F_drag = 0.5 * rho * Af * CD * v * abs(v);

    % Rolling resistance
    F_roll = M * g * CR * sgn_approx(v);

    % Efforts
    e8 = (GR/R) * (i_in - bT * (GR/R) * v);
    e10 = F_drag;
    e11 = F_roll;

    %Dynamic v_ref input file
    v_ref = LA92Oracle(t);

    % Input Voltage (PI Control)
    u_in = Kp * (v_ref - v) + Ki * (d_ref - d_actual);

    % State equations
    pdot_L = u_in - Rw * i_in - Tm * (GR/R) * v;  % Flux linkage derivative
    pdot_M = e8 - e10 - e11;                    % Momentum derivative

    %Reference distance and actual distance
    ddot_ref = v_ref;
    ddot_actual= v;


p(1)= (u_in)*(i_in); %input energy
p(2)=v*pdot_M; %output energy  %SEE IF WE CAN figure out whats up with PdotM and if its the right var to use here.
 

    % Output derivative
    dxdt = [pdot_L; pdot_M; ddot_actual; ddot_ref];  % Return as a column vector





end 
