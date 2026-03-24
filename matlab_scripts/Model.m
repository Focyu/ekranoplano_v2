function [XDOT] = Model(X, U)
% Modelo Dinámico del Ekranoplano AirFish 8
% ESCALA: 1:21.5 (Modelo Físico para Tablero Estudiantil)
% Longitud aprox: 80 cm | Envergadura: 70 cm | Masa: 300 g

%-----------STATE AND CONTROL VECTORS-------------
x1=X(1); x2=X(2); x3=X(3); x4=X(4); x5=X(5); x6=X(6);
x7=X(7); x8=X(8); x9=X(9); x10=X(10); x11=X(11); x12=X(12);
u1=U(1); u2=U(2); u3=U(3); u4=U(4); u5=U(5);

%-----------SYSTEM PARAMETERS (ESCALA 1:21.5 - REALISTA 1.2 KG)------
m = 1.200;          % Masa real ajustada (Kg)
bw = 0.6977;        % Span Wing (m) - 69.8 cm
bh = 0.3808;        % Span HTP (m)
cbar = 0.0930;      % Mean Aerodynamic Chord (m)
lt = 0.5600;        % Distance AC tail to body (m)
Sw = 0.0649;        % Wing planform area (m^2)
Sh = 0.02164;       % Tail planform area (m^2)
ARw = 7.5; ARh = 6.7; TRw = 0.4; TRh = 0.5;

Xcg = 0; Ycg = 0; Zcg = 0;
Xac = 0.0135; Yac = 0; Zac = -0.0083;
Xapt1 = -0.0721; Yapt1 =  0.0834; Zapt1 = 0.0396;
Xapt2 = -0.0721; Yapt2 = -0.0834; Zapt2 = 0.0396;

rho = 1.225; g = 9.81; e0w = 0.9; e0h = 0.9;
alpha_0w = -3.75*pi/180; alpha_0h = -4.25*pi/180;
iw = 1.0 * pi/180; ih = 0.5 * pi/180;
zw = 0.0505; zh = 0.3475; CD_0 = 0.0306; 


%-----------CONTROL LIMITS SATURATION------------------------------
u1 = max(min(u1, 20*pi/180), -20*pi/180); % Aileron
u2 = max(min(u2, 20*pi/180), -20*pi/180); % Elevator
u3 = max(min(u3, 15*pi/180), -15*pi/180); % Rudder
u4 = max(min(u4, 1.0), 0.0);              % Throttle 1 [0,1]
u5 = max(min(u5, 1.0), 0.0);              % Throttle 2 [0,1]

%-------------INTERMEDIATE VARIABLES-----------------------------
u_r = x1; v_r = x2; w_r = x3; % Sin viento en este script
Va = sqrt(u_r^2 + v_r^2 + w_r^2);

if Va < 0.5
    alpha = 0; beta = 0;
else
    alpha = atan2(w_r, u_r);
    beta = asin(v_r / Va);
end

h = -x12;
hw = max(0.01, h - zw); % Protección división por cero
hh = max(0.01, h + zh); 

Q = 0.5*rho*Va^2;
wbe_b = [x4; x5; x6];
V_b = [x1; x2; x3];

%--------------Aerodynamics (Sincronizado con WIG form)----------
% Sustentación con efecto suelo
mu_Lw = 1 + (1 - 2.25 * (TRw^0.00273 - 0.997)*((ARw^0.717)+13.6))*(288 * abs(hw/bw)^0.787 * exp(-9.14*(abs(hw/bw)^0.327)))/(ARw^0.882);
mu_Lh = 1 + (1 - 2.25 * (TRh^0.00273 - 0.997)*((ARh^0.717)+13.6))*(288 * abs(hh/bh)^0.787 * exp(-9.14*(abs(hh/bh)^0.327)))/(ARh^0.882);

% Downwash dinámico a la cola
deps_dalpha = 0.35; 
eps = deps_dalpha * (alpha - alpha_0w) + deps_dalpha * (lt/max(Va,0.1)) * x5;

CL_w_OGE = 2*pi*(ARw/(ARw+2))*(alpha - alpha_0w + iw);
CL_h_OGE = 2*pi*(ARh/(ARh+2))*(alpha - alpha_0h + ih - eps); % Con downwash

CL_w_IGE = CL_w_OGE * mu_Lw;
CL_h_IGE = CL_h_OGE * mu_Lh;
Ltot = Q * (CL_w_IGE*Sw + CL_h_IGE*Sh);

% Arrastre inducido WIG
mu_Dw = 1 - (1 - 0.157*max(0, (TRw^0.757 - 0.373))*max(0,(ARw^0.417 - 1.27)))*exp(-4.74*max(0,abs(hw/bw)^0.814))-abs(hw/bw)^2*exp(-3.88*max(0,abs(hw/bw)^0.758));
mu_Dh = 1 - (1 - 0.157*max(0, (TRh^0.757 - 0.373))*max(0,(ARh^0.417 - 1.27)))*exp(-4.74*max(0,abs(hh/bh)^0.814))-abs(hh/bh)^2*exp(-3.88*max(0,abs(hh/bh)^0.758));

CD_iw_IGE = CL_w_IGE^2*mu_Dw/(pi*e0w*ARw);
CD_ih_IGE = CL_h_IGE^2*mu_Dh/(pi*e0h*ARh);
C_D_e = -0.0000108*u2^2+0.000715*u2;

Dtot = Q * (CD_0*Sw + CD_iw_IGE*Sw + CD_ih_IGE*Sh + C_D_e*Sh);
CQ = -0.019*beta*180/pi; 
LD_ratio = Ltot/Dtot;

FA_s = [-Dtot; CQ*Q*Sw; -Ltot];
C_bs = [cos(alpha) 0 -sin(alpha); 0 1 0; sin(alpha) 0 cos(alpha)];
FA_b = C_bs*FA_s;

%------------------ Aerodynamic Torques (Unificado) ------------------
Cm_alpha = -1.14; Cm_q = -5.0; Cm_de = -3.0; Cm_h = 0.05;
Delta_Cm_IGE = -Cm_h * exp(-4.0 * abs(hw/bw));

Cl_da = -0.5; Cl_p = -2.0; 
Cn_dr = -0.3; Cn_r = -1.5;

Cl = -0.0286*beta + Cl_p*(x4*bw/(2*max(Va,0.1))) + Cl_da*u1;
Cm = Cm_alpha*alpha + Delta_Cm_IGE + Cm_q*(x5*cbar/(2*max(Va,0.1))) + Cm_de*u2; 
Cn = -0.1146*beta + Cn_r*(x6*bw/(2*max(Va,0.1))) + Cn_dr*u3;

MAcg_b = [Cl; Cm; Cn] .* [bw*Q*Sw; cbar*Q*Sw; bw*Q*Sw];

%------------------------- Engine Model (Escalado) -------------------
km = 25.0;        % Velocidad máxima de pitch de la hélice [m/s]
Cp = 0.57;        
D = 0.0883;       % Diámetro hélice (8.83 cm)
Sp = pi*(D^2/4);    

Vd1 = Va + u4*(km-Va); Tp1 = 0.5*rho*Sp*Cp*Vd1*(Vd1-Va);
Vd2 = Va + u5*(km-Va); Tp2 = 0.5*rho*Sp*Cp*Vd2*(Vd2-Va);

FE1_b = [Tp1*cos(-5*pi/180); 0; Tp1*sin(5*pi/180)];
FE2_b = [Tp2*cos(-5*pi/180); 0; Tp2*sin(5*pi/180)];
FE_b = FE1_b + FE2_b;

MEcg1_b = cross([Xcg-Xapt1; Yapt1-Ycg; Zcg-Zapt1], FE1_b);
MEcg2_b = cross([Xcg-Xapt2; Yapt2-Ycg; Zcg-Zapt2], FE2_b);
MEcg_b = MEcg1_b + MEcg2_b;

%------------------------- Hydrodynamics (Opcional, escalada) ---------
draft = 0 - h; 
F_hydro_b = [0;0;0];
if draft > 0
    K_buoy = m * g / 0.02; % Pesa todo a los 2cm sumergido
    D_buoy = 15;           % Amortiguación re-escalada
    F_buoy_z = -(K_buoy * draft - D_buoy * x3);
    
    Cd_water = 0.8; A_wetted = 0.2 * draft;
    Drag_water = 0.5 * 1025 * u_r^2 * Cd_water * A_wetted;
    
    Rb_v = [cos(x8)*cos(x9) cos(x8)*sin(x9) -sin(x8);
            sin(x7)*sin(x8)*cos(x9)-cos(x7)*sin(x9) sin(x7)*sin(x8)*sin(x9)+cos(x7)*cos(x9) sin(x7)*cos(x8);
            cos(x7)*sin(x8)*cos(x9)+sin(x7)*sin(x9) cos(x7)*sin(x8)*sin(x9)-sin(x7)*cos(x9) cos(x7)*cos(x8)];
    F_hydro_b = Rb_v' * [-Drag_water; 0; F_buoy_z];
end

%----------------------------Dynamics----------------------------------
g_b = [-g*sin(x8); g*cos(x8)*sin(x7); g*cos(x8)*cos(x7)];
Fg_b = m*g_b;

F_b = Fg_b + FE_b + FA_b + F_hydro_b; 
x1to3dot = (1/m)*F_b - cross(wbe_b,V_b);

Ixx = 0.008217; Iyy = 0.017695; Izz = 0.023669; Ixz = 0.001856;
Gamma = Ixx*Izz - Ixz^2; 

p = x4; q = x5; r = x6;
Mcg_b = MAcg_b + MEcg_b;
L = Mcg_b(1); M = Mcg_b(2); N = Mcg_b(3);

p_dot = (Izz*L + Ixz*N - (Ixz*(Iyy - Ixx - Izz)*p + (Ixz^2 + Izz*(Izz - Iyy))*r)*q) / Gamma;
q_dot = (M - (Ixx - Izz)*p*r - Ixz*(p^2 - r^2)) / Iyy;
r_dot = (Ixz*L + Ixx*N + (Ixz*(Iyy - Ixx - Izz)*r + (Ixz^2 + Ixx*(Ixx - Iyy))*p)*q) / Gamma;

x4to6dot = [p_dot; q_dot; r_dot];
H_phi = [1 sin(x7)*tan(x8) cos(x7)*tan(x8); 0 cos(x7) -sin(x7); 0 sin(x7)/cos(x8) cos(x7)/cos(x8)];
x7to9dot = H_phi*wbe_b;

% Euler a NED
c_phi = cos(x7); s_phi = sin(x7); c_the = cos(x8); s_the = sin(x8); c_psi = cos(x9); s_psi = sin(x9);
R_ned = [c_the*c_psi, s_phi*s_the*c_psi - c_phi*s_psi, c_phi*s_the*c_psi + s_phi*s_psi;
         c_the*s_psi, s_phi*s_the*s_psi + c_phi*c_psi, c_phi*s_the*s_psi - s_phi*c_psi;
        -s_the,       s_phi*c_the,                     c_phi*c_the];
x10to12dot = R_ned * [x1; x2; x3];

XDOT = [x1to3dot; x4to6dot; x7to9dot; x10to12dot; LD_ratio; F_b; Mcg_b; CQ; Cl; Cm; Cn; alpha; beta; CL_w_OGE; CL_h_OGE; CL_w_IGE; CL_h_IGE; CD_iw_IGE; CD_ih_IGE; Fg_b; FE_b; FA_b];
end
