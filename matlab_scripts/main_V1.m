clear all; close all; clc;

%% PARAMETROS GLOBALES DEL MODELO
% Motor: 2204-2300KV | Hélice: APC 5x4.5" | 3S LiPo
Cp_motor = 0.57;
D_prop   = 0.1270;                       % CORREGIDO: 5" (era 3.5")
rho      = 1.225;
Sp_prop  = pi*(D_prop^2/4);
km_motor = 25.0;
% Thrust máximo por motor (u=1, Va=0): 0.5*rho*Sp*Cp*km*km
Cp = Cp_motor;   % alias para el bloque Simulink
D  = D_prop;     % alias para el bloque Simulink (si lo usa)
Sp = pi*(D^2/4);
km = km_motor;  
max_thrust_force_per_motor = 0.5*rho*Sp_prop*Cp_motor*km_motor^2;
fprintf('Thrust max por motor: %.3f N\n', max_thrust_force_per_motor);
fprintf('Thrust max total:     %.3f N\n', 2*max_thrust_force_per_motor);

%% CONDICIONES INICIALES BASE
tsim = 10;
step = 0.01;

Va_cruise  = 20.2;          % [m/s] velocidad de crucero
h_cruise   = 0.550;         % [m]   altura de crucero
theta_eq   = 1.5*(pi/180);  % [rad] pitch de trimado (~1.5°)

%% PUNTOS DE EQUILIBRIO — GAIN SCHEDULING
% ------------------------------------------------------------------
% Parámetros de giro calculados del modelo dinámico (escala 1:21.5)
%   bw = 0.6977 m | Cn_r = -1.5 | Q_cruise = 0.5*rho*Va^2
%   Yapt = 0.0834 m (brazo de palanca motor)
% ------------------------------------------------------------------
R_turn         = 160.0;              % [m]   radio mínimo operacional
r_eq           = Va_cruise / R_turn; % [rad/s] = 0.1263 yaw rate de equilibrio
delta_u_diff   = 0.025;              % fracción de throttle diferencial por motor

fprintf('r_eq (yaw rate giro): %.4f rad/s  (%.2f deg/s)\n', r_eq, r_eq*180/pi);
fprintf('R_turn: %.1f m\n', R_turn);

% ── Estado nominal vuelo RECTO ─────────────────────────────────────
x_nom_straight        = zeros(12,1);
x_nom_straight(1)     = Va_cruise;      % u  [m/s]
x_nom_straight(8)     = theta_eq;       % theta [rad]
x_nom_straight(12)    = -h_cruise;      % z_NED [m]

% ── Estado nominal giro a la DERECHA (r > 0 → horario en NED) ─────
x_nom_right           = x_nom_straight;
x_nom_right(6)        = +r_eq;          % r [rad/s]

% ── Estado nominal giro a la IZQUIERDA (r < 0) ────────────────────
x_nom_left            = x_nom_straight;
x_nom_left(6)         = -r_eq;          % r [rad/s]

% ── Control nominal base ───────────────────────────────────────────
u_nom_straight        = zeros(5,1);
u_nom_straight(2)     = 0.0;            % elevador neutro
u_nom_straight(4:5)   = 0.52;           % throttle de crucero

% Control nominal giro derecha: motor izquierdo (+) motor derecho (-)
u_nom_right           = u_nom_straight;
u_nom_right(4)        = 0.52 + delta_u_diff;
u_nom_right(5)        = 0.52 - delta_u_diff;

% Control nominal giro izquierda: motor izquierdo (-) motor derecho (+)
u_nom_left            = u_nom_straight;
u_nom_left(4)         = 0.52 - delta_u_diff;
u_nom_left(5)         = 0.52 + delta_u_diff;

%% SELECCIÓN DE MODO DE VUELO
% ------------------------------------------------------------------
% flight_mode: 0 = vuelo recto  |  1 = giro derecha  |  2 = giro izquierda
% Cambiar este valor antes de simular o regenerar código con Simulink Coder
% ------------------------------------------------------------------
flight_mode = 0;

switch flight_mode
    case 1   % Giro derecha
        x_nom = x_nom_right;
        u_nom = u_nom_right;
        fprintf('Modo: GIRO DERECHA  r_eq=+%.4f rad/s\n', r_eq);
    case 2   % Giro izquierda
        x_nom = x_nom_left;
        u_nom = u_nom_left;
        fprintf('Modo: GIRO IZQUIERDA  r_eq=-%.4f rad/s\n', r_eq);
    otherwise % Vuelo recto (default)
        x_nom = x_nom_straight;
        u_nom = u_nom_straight;
        fprintf('Modo: VUELO RECTO\n');
end

x0 = x_nom;

%% PARAMETROS DE CONTROL PID (Para modelo de 1.2 kg)
% ------------------------------------------------------------------
% Ganancias base — Vuelo RECTO
% ------------------------------------------------------------------

% 1. Lazo de Velocidad
u_sp   = Va_cruise;
Kp_u   = 0.08;
Ki_u   = 0.015;
Kd_u   = 0.005;

% 2. Lazo de Altura
h_sp   = h_cruise;
Kp_h   = 2.0;
Ki_h   = 0.25;
Kd_h   = 4.5;

% Límite de seguridad en pitch
theta_max =  5.0 * (pi/180);
theta_min = -3.0 * (pi/180);

% 3. Lazo Interno de Elevador (Pitch)
Kp_pitch = -0.50;
Ki_pitch = -0.03;
Kd_pitch = -0.3;

% 4. Lazo de Timón (Yaw) — ganancias base (vuelo recto)
psi_sp   = 0 * (pi/180);
Kp_yaw   = -0.40;
Ki_yaw   = -0.005;
Kd_yaw   = -0.4;

% 5. Lazo de Alerones (Roll)
phi_sp   = 0.0;
Kp_roll  = -1.2;
Ki_roll  = -0.08;
Kd_roll  = -0.5;

% ------------------------------------------------------------------
% Ganancias alternativas — Modo GIRO
% Ajustadas para rastreo de referencia tipo rampa en yaw:
%   - Kp menor: evita saturación del timón en el inicio del giro
%   - Ki mayor: compensa el error de estado estacionario ante rampa
%   - Kd menor: reduce oscilaciones al seguir la rampa de psi
% ------------------------------------------------------------------
Kp_yaw_turn = -0.35;
Ki_yaw_turn = -0.015;
Kd_yaw_turn = -0.20;

% Aplicar ganancias según modo activo
if flight_mode == 1 || flight_mode == 2
    Kp_yaw = Kp_yaw_turn;
    Ki_yaw = Ki_yaw_turn;
    Kd_yaw = Kd_yaw_turn;
    fprintf('Ganancias PID Yaw: modo GIRO aplicado\n');
else
    fprintf('Ganancias PID Yaw: modo RECTO aplicado\n');
end

%% Simulation
sim = sim('pid_control_V1.slx');

S = sim.states;
t = sim.tout;
% C = sim.control.*ones(length(t),5); %antes se transformaba
C = sim.control_out; %sim.control_out será directamente una matriz de dimensión

LD_ratio = sim.LD;
Fbx = sim.Fbx;
Fby = sim.Fby;
Fbz = sim.Fbz;
Mx = sim.Mbx;
My = sim.Mby;
Mz = sim.Mbz;

CQ = sim.CQ;
Cl = sim.Cl;
Cn = sim.Cn;
Cm = sim.Cm;
alpha = sim.alpha;
beta = sim.beta;

CL_w_OGE = sim.CL_w_OGE;
CL_h_OGE = sim.CL_h_OGE;
CL_w_IGE = sim.CL_w_IGE;
CL_h_IGE = sim.CL_h_IGE;
CD_iw_IGE = sim.CD_iw_IGE;
CD_ih_IGE = sim.CD_ih_IGE;

Fgx = sim.Fgx;
Fgy = sim.Fgy;
Fgz = sim.Fgz;
Fax = sim.Fax;
Fay = sim.Fay;
Faz = sim.Faz;
Ftx = sim.Ftx;
Fty = sim.Fty;
Ftz = sim.Ftz;

P = sim.Power;
E = sim.Energy;
LT = sim.Load_Torque;

% Creating the .csv file for numerical export of results
% T1 = table(LD_ratio,Fbx,Fby,Fbz,Mx,My,Mz,CQ,Cl,Cm,Cn,alpha,beta,CL_w_OGE,CL_h_OGE,CL_w_IGE,CL_h_IGE,CD_iw_IGE,CD_ih_IGE,'VariableNames',["LD_rat","Fbx","Fby","Fbz","Mx","My","Mz","CQ","Cl","Cm","Cn","Alpha","Beta","CL_w_OGE","CL_h_OGE","CL_w_IGE","CL_h_IGE","CD_iw_IGE","CD_ih_IGE"]);
% T2 = table(S(:,1),S(:,2),S(:,3),S(:,4),S(:,5),S(:,6),S(:,7),S(:,8),S(:,9),S(:,10),S(:,11),S(:,12),'VariableNames',["u","v","w","p","q","r","phi","theta","psi","x_NED","y_NED","z_NED"]);
% T3 = table(C(:,1),C(:,2),C(:,3),C(:,4),C(:,5),'VariableNames',["aileron","elevator","rudder","thrust_1","thrust_2"]);
% writetable([T1 T2 T3],file_name);
% 
% %% Plotting
% % LD ratio
% figure
% plot(t,LD_ratio(:,1))
% xlabel('t [s]')
% ylabel('L/D')
% title('LD ratio')
% grid on
% %-------------------Alpha and beta----------
% figure
% subplot(2,1,1)
% plot(t,alpha(:,1))
% xlabel('t [s]')
% ylabel('\alpha [rad]')
% title('Angle of attack')
% grid on
% 
% subplot(2,1,2)
% plot(t,beta(:,1))
% xlabel('t [s]')
% ylabel('\beta [rad]')
% title('Side-slip Angle')
% grid on
% %-------------------Power, Energy & Load Torque-------------------
% figure
% subplot(3,1,1)
% plot(t,P(:,1),'LineWidth',1.2)
% xlabel('t[s]')
% ylabel('P [kW]')
% grid on
% title('Power')
% 
% subplot(3,1,2)
% plot(t,E(:,1),'LineWidth',1.2)
% xlabel('t[s]')
% ylabel('E [kWh]')
% grid on
% title('Energy')
% 
% subplot(3,1,3)
% plot(t,LT(:,1),'LineWidth',1.2)
% xlabel('t[s]')
% ylabel('T_L [Nm]')
% grid on
% title('Load Torque')
% % ----------------forces and torques---------------------------------------------
% figure
% subplot(2,2,1)
% hold on
% plot(t,Fbx(:,1),'LineWidth',1.2)
% plot(t,Fby(:,1),'LineWidth',1.2)
% plot(t,Fbz(:,1),'LineWidth',1.2)
% hold off
% xlabel('t [s]')
% ylabel('F_b [N]')
% title('F_b')
% legend('Fbx','Fby','Fbz')
% grid on
% 
% subplot(2,2,2)
% hold on
% plot(t,Fax(:,1),'LineWidth',1.2)
% plot(t,Fay(:,1),'LineWidth',1.2)
% plot(t,Faz(:,1),'LineWidth',1.2)
% hold off
% xlabel('t [s]')
% ylabel('F_a [N]')
% title('F_a')
% legend('Fax','Fay','Faz')
% grid on
% 
% subplot(2,2,3)
% hold on
% plot(t,Ftx(:,1),'LineWidth',1.2)
% plot(t,Fty(:,1),'LineWidth',1.2)
% plot(t,Ftz(:,1),'LineWidth',1.2)
% hold off
% xlabel('t [s]')
% ylabel('F_t [N]')
% title('F_t')
% legend('Ftx','Fty','Ftz')
% grid on
% 
% subplot(2,2,4)
% hold on
% plot(t,Fgx(:,1),'LineWidth',1.2)
% plot(t,Fgy(:,1),'LineWidth',1.2)
% plot(t,Fgz(:,1),'LineWidth',1.2)
% hold off
% xlabel('t [s]')
% ylabel('F_g [N]')
% title('F_g')
% legend('Fgx','Fgy','Fgz')
% grid on
% 
% 
% figure
% subplot(3,1,1)
% plot(t,Mx(:,1))
% xlabel('t [s]')
% ylabel('M_b_x [N]')
% title('M_b_x')
% grid on
% 
% subplot(3,1,2)
% plot(t,My(:,1))
% xlabel('t [s]')
% ylabel('M_b_y [N]')
% title('M_b_y')
% grid on
% 
% subplot(3,1,3)
% plot(t,Mz(:,1))
% xlabel('t [s]')
% ylabel('M_b_z [N]')
% title('M_b_z')
% grid on
% sgtitle('Torque vector M_b')
% % ----------------Aerodyn Coeff---------------------------------------------
% figure
% subplot(1,2,1)
% plot(t,CL_w_OGE(:,1))
% xlabel('t [s]')
% ylabel('CL_w_{OGE}')
% title('CL_w_{OGE}')
% grid on
% 
% subplot(1,2,2)
% plot(t,CL_h_OGE(:,1))
% xlabel('t [s]')
% ylabel('CL_h_{OGE}')
% title('CL_h_{OGE}')
% grid on
% sgtitle('Aerodyn Coefficient from lift forces OGE')
% 
% figure
% subplot(1,2,1)
% plot(t,CL_w_IGE(:,1))
% xlabel('t [s]')
% ylabel('CL_w_{IGE}')
% title('CL_w_{IGE}')
% grid on
% 
% subplot(1,2,2)
% plot(t,CL_h_IGE(:,1))
% xlabel('t [s]')
% ylabel('CL_h_{IGE}')
% title('CL_h_{IGE}')
% grid on
% sgtitle('Aerodyn Coefficient from lift forces IGE')
% 
% figure
% subplot(1,2,1)
% plot(t,CD_iw_IGE(:,1))
% xlabel('t [s]')
% ylabel('CD_{iw}_{IGE}')
% title('CD_{iw}_{IGE}')
% grid on
% 
% subplot(1,2,2)
% plot(t,CD_ih_IGE(:,1))
% xlabel('t [s]')
% ylabel('CD_{ih}_{IGE}')
% title('CD_{ih}_{IGE}')
% grid on
% sgtitle('Aerodyn Coefficient from drag forces IGE')
% %--------------------------------------------------------
% figure
% plot(t,CQ(:,1))
% xlabel('t [s]')
% ylabel('C_Q')
% title('C_Q')
% grid on
% title('Aerodyn Coefficient from side forces')
% 
% figure
% subplot(3,1,1)
% plot(t,Cl(:,1))
% xlabel('t [s]')
% ylabel('C_l')
% title('C_l')
% grid on
% 
% subplot(3,1,2)
% plot(t,Cm(:,1))
% xlabel('t [s]')
% ylabel('C_m')
% title('C_m')
% grid on
% 
% subplot(3,1,3)
% plot(t,Cn(:,1))
% xlabel('t [s]')
% ylabel('C_n')
% title('C_n')
% grid on
% sgtitle('Aerodyn Coeff from torques')
% 
% % ----------------States---------------------------------------------
% % Linear velocities
% figure
% plot(t,S(:,1),t,S(:,2),t,S(:,3))
% xlabel('t [s]')
% ylabel('[m/s]')
% title('Linear velocities')
% grid on
% legend('u','v','w')
% %-----------------------------angular velocities
% figure
% plot(t,S(:,4),t,S(:,5),t,S(:,6))
% xlabel('t [s]')
% ylabel('[rad/s]')
% title('Angular velocities')
% grid on
% legend('p','q','r')
% 
% %-----------------------Euler angles
% figure
% plot(t,S(:,7),t,S(:,8),t,S(:,9))
% xlabel('t [s]')
% ylabel('[rad]')
% title('Euler angles')
% grid on
% legend('\phi','\theta','\psi')
% 
% figure
% hold all
% plot(t,-S(:,12))
% xlabel('t[s]')
% ylabel('h_{NED}[m]')
% title('Height in NED')
% grid on
% 
% 
% figure
% hold all
% plot(t,S(:,10))
% plot(t,S(:,11))
% plot(t,S(:,12))
% xlabel('t[s]')
% ylabel('[m]')
% title('Positions in NED')
% legend('x_{NED}','y_{NED}','z_{NED}')
% grid on
% 
% figure
% hold all
% plot3(S(1,10),S(1,11),-S(1,12),'rx')
% plot3(S(:,10),S(:,11),-S(:,12),'b')
% xlabel('x_{NED}')
% ylabel('y_{NED}')
% zlabel('h_{NED}')
% title('Trajectory in 3D')
% grid on
% legend('(x_0,y_0,h_0)','(x(t),y(t),h(t))')
% %--------------other-----------------
% gama = S(:,8)-atan2(S(:,3),S(:,1));
% figure
% plot(t,gama)
% xlabel('t [s]')
% ylabel('\gamma [rad]')
% title('Flight Path Angle \gamma')
% grid on
% 
% Va = sqrt(S(:,1).^2+S(:,2).^2+S(:,3).^2);
% figure
% plot(t,Va)
% xlabel('t [s]')
% ylabel('V_a [m/s]')
% title('True Airspeed V_a')
% grid on
% % ----------------Control---------------------------------------------
% figure
% subplot(5,1,1)
% plot(t,C(:,1))
% xlabel('t [s]')
% ylabel('\delta_a [deg]')
% title('Aileron Deflection')
% grid on
% 
% subplot(5,1,2)
% plot(t,C(:,2))
% xlabel('t [s]')
% ylabel('\delta_e [deg]')
% title('Elevator Deflection')
% grid on
% 
% subplot(5,1,3)
% plot(t,C(:,3))
% xlabel('t [s]')
% ylabel('\delta_r [deg]')
% title('Rudder Deflection')
% grid on
% 
% subplot(5,1,4)
% plot(t,C(:,4).*100./max_thrust_force_per_motor)
% xlabel('t [s]')
% ylabel('\delta_{t1} [%]')
% title('Throttle 1')
% grid on
% 
% subplot(5,1,5)
% plot(t,C(:,5).*100./max_thrust_force_per_motor)
% xlabel('t [s]')
% ylabel('\delta_{t2} [%]')
% title('Throttle 2')
% grid on

% %% Save all figures automatically
% 
% output_folder = '/home/facyu/Descargas';
% 
% % Crear la carpeta si no existe
% if ~exist(output_folder, 'dir')
%     mkdir(output_folder);
% end
% 
% % Obtener todas las figuras abiertas
% figs = findall(0,'Type','figure');
% 
% for i = 1:length(figs)
%     filename = fullfile(output_folder, sprintf('Figura_%02d.png', i));
%     exportgraphics(figs(i), filename, 'Resolution', 300);
% end
% 
% disp('Todas las figuras fueron guardadas correctamente.')
