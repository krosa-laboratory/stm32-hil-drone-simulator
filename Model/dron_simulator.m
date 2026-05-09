% ==========================================
% SIMULADOR FÍSICO DE DRON 6-DoF
% ==========================================
clear all; clc; close all;

% PARÁMETROS FÍSICOS DEL DRON
masa_dron  = 1.2;   % Masa del dron en Kg
a_gravedad = 9.81;  % Aceleración de la Gravedad en m/s^2
inercia_xx = 0.01;  % Inercia en X, Roll
inercia_yy = 0.01;  % Inercia en Y, Pitch
inercia_zz = 0.01;  % Inercia en Z, Yaw
f_aero_lin = 0.1;   % Fricción aerodinámica lineal
tau_motor  = 0.05;  % Tiempo que tardan los motores en reaccionar

% CONFIGURACIÓN DE LA SIMULACIÓN
dt = 0.01;        % Paso de tiempo entre iteraciones
t_final = 20;     % Segundos totales a simular (ampliado para viaje en XY)
N = t_final / dt; % Número total de iteraciones
alpha = dt / (tau_motor + dt); % Constante de filtro para los motores
beta = 0.1;       % Peso para el filtro de sensores MME

% VARIABLES DE ESTADO INICIALES
x = 0; y = 0; z = 10;             % Posición inicial
vx = 0; vy = 0; vz = 0;           % Velocidades lineales
phi = 0; theta = 0; psi = 0;      % Ángulos de Euler (Roll, Pitch, Yaw)
p = 0; q = 0; r = 0;              % Velocidades angulares

z_filtrada = z;                   % Inicializamos el filtro
U1_real = masa_dron * a_gravedad; % Empuje inicial

% CONSTANTES DE LOS CONTROLADORES PID
% PID Altitud (Z)
z_ref  = 5;
kp_z = 3; ki_z = 0; kd_z = 4;
err_z_acumulado = 0; err_z_previo = 0;

% PIDs de Navegación (X e Y)
x_ref = 5;
y_ref = 5;
kp_xy = 0.1; ki_xy = 0; kd_xy = 0.25;
err_x_acumulado = 0; err_x_previo = 0;
err_y_acumulado = 0; err_y_previo = 0;

angulo_max = 0.5; % Saturación: Límite de inclinación en radianes (~28 grados)

% VECTORES DE TELEMETRÍA
hist_t          = zeros(1, N);
hist_x          = zeros(1, N);
hist_y          = zeros(1, N);
hist_z_real     = zeros(1, N);
hist_z_medida   = zeros(1, N);
hist_z_filtrada = zeros(1, N);

% ==========================================
% BUCLE PRINCIPAL DE VUELO
% ==========================================
for i = 1 : N

  % LECTURA DE SENSORES Y FILTRADO
  z_medida = z + randn() * 0.2; % Ruido blanco
  z_filtrada = (1 - beta) * z_filtrada + beta * z_medida; % Filtro MME

  % PILOTO DE NAVEGACIÓN

  % PID de Altitud (Z) -> Decide la fuerza hacia arriba
  err_z = z_ref - z_filtrada;
  err_z_acumulado = err_z_acumulado + err_z * dt;
  vel_err_z = (err_z - err_z_previo) / dt;
  fuerza_pid = kp_z * err_z + err_z_acumulado * ki_z + vel_err_z * kd_z;
  err_z_previo = err_z;

  % PID de Eje X -> Decide cuánto cabecear (Pitch / Theta)
  err_x = x_ref - x;
  err_x_acumulado = err_x_acumulado + err_x * dt;
  vel_err_x = (err_x - err_x_previo) / dt;
  theta_deseado = kp_xy * err_x + err_x_acumulado * ki_xy + vel_err_x * kd_xy;
  err_x_previo = err_x;

  % PID de Eje Y -> Decide cuánto alabear (Roll / Phi)
  err_y = y_ref - y;
  err_y_acumulado = err_y_acumulado + err_y * dt;
  vel_err_y = (err_y - err_y_previo) / dt;
  phi_deseado = -(kp_xy * err_y + err_y_acumulado * ki_xy + vel_err_y * kd_xy);
  err_y_previo = err_y;

  % SATURACIÓN DE SEGURIDAD
  if theta_deseado > angulo_max; theta_deseado = angulo_max;
  elseif theta_deseado < -angulo_max; theta_deseado = -angulo_max; end

  if phi_deseado > angulo_max; phi_deseado = angulo_max;
  elseif phi_deseado < -angulo_max; phi_deseado = -angulo_max; end

  % Asumimos que el dron rota mágicamente e instantáneamente a los ángulos deseados.
  theta = theta_deseado;
  phi = phi_deseado;
  psi = 0;

  % ACTUADORES (Fuerza Z)
  U1_deseado = (masa_dron * a_gravedad) + fuerza_pid;

  if U1_deseado <= 0; U1_deseado = 0;
  elseif U1_deseado > 2 * masa_dron * a_gravedad; U1_deseado = 2 * masa_dron * a_gravedad; end

  U1_real = U1_real + alpha * (U1_deseado - U1_real);

  % DINÁMICA FÍSICA Y CINEMÁTICA
  ax = (U1_real / masa_dron) * (cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi)) - (f_aero_lin / masa_dron) * vx;
  ay = (U1_real / masa_dron) * (cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi)) - (f_aero_lin / masa_dron) * vy;
  az = (U1_real / masa_dron) * (cos(phi) * cos(theta)) - a_gravedad - (f_aero_lin / masa_dron) * vz;

  vx = vx + ax * dt;
  vy = vy + ay * dt;
  vz = vz + az * dt;

  x = x + vx * dt;
  y = y + vy * dt;
  z = z + vz * dt;

  if z <= 0; z = 0; vz = 0; end

  % GUARDAR TELEMETRÍA
  hist_t(i)          = i * dt;
  hist_x(i)          = x;
  hist_y(i)          = y;
  hist_z_real(i)     = z;
  hist_z_medida(i)   = z_medida;
  hist_z_filtrada(i) = z_filtrada;

end

% ==========================================
% VISUALIZACIÓN DE RESULTADOS
% ==========================================
% Figura 1: Telemetría 2D (Altitud y Navegación)
figure(1);
subplot(2,1,1);
hold on;
plot(hist_t, hist_z_real, 'b', 'LineWidth', 2);
plot(hist_t, hist_z_filtrada, 'g', 'LineWidth', 2);
title('Control PID: Altitud (Z)');
ylabel('Altitud (m)');
legend('Z Real', 'Z Filtrada');
grid on;

subplot(2,1,2);
hold on;
plot(hist_t, hist_x, 'r', 'LineWidth', 2);
plot(hist_t, hist_y, 'm', 'LineWidth', 2);
title('Control PID: Navegación (X e Y)');
xlabel('Tiempo (s)');
ylabel('Posición (m)');
legend('Posición X', 'Posición Y');
grid on;

% Figura 2: Vista de la trayectoria en 3D
figure(2);
plot3(hist_x, hist_y, hist_z_real, 'b', 'LineWidth', 2);
grid on;
title('Trayectoria de Vuelo 3D hacia [5, 5, 5]');
xlabel('Eje X (m)');
ylabel('Eje Y (m)');
zlabel('Eje Z (Altitud en m)');

