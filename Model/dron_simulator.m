% ==========================================
% SIMULADOR FÍSICO DE DRON 6-DoF
% ==========================================
clear all; clc; close all;

% Parámetros físicos del Dron
masa_dron  = 1.2;   % Masa del dron en Kg
a_gravedad = 9.81;  % Aceleración de la Gravedad en m / s^2
inercia_xx = 0.01;  % Inercia en X, Roll
inercia_yy = 0.01;  % Inercia en Y, Pitch
inercia_zz = 0.01;  % Inercia en Z, Yaw
f_aero_lin = 0.1;   % Fricción aerodinámica lineal

% Variables de estado
x = 0; y = 0; z = 10;         % Variables de posición
vx = 0; vy = 0; vz = 0;       % Variables de velocidad lineal
phi = 0; theta = 0; psi = 0;  % Variables de ángulos de Euler
p = 0; q = 0; r = 0;          % Variables de velocidad angular

% Configuración de la Simulación
dt = 0.01;        % Paso de tiempo entre iteraciones
t_final = 10;      % Segundos totales a simular
N = t_final / dt; % Número total de iteraciones

% Vectores para guardar la telemetría y graficar después
hist_t  = zeros(1, N);
hist_z  = zeros(1, N);
hist_vz = zeros(1, N);

% Constantes del Controlador PID
z_ref  = 5;
err_acumulado = 0;
err_previo = 0;
kp = 5;
ki = 0;
kd = 2;

% Bucle principal para simular el Dron
for i = 1 : N

  % Controlador PID
  err_actual= z_ref - z;
  err_acumulado = err_acumulado + err_actual * dt;
  vel_err = (err_actual - err_previo) / dt;
  fuerza_pid = kp * err_actual + err_acumulado * ki + vel_err * kd;
  err_previo = err_actual;

  % Cáculo de los Empujes del Dron
  U1 = (masa_dron * a_gravedad) + fuerza_pid;
  U2 = 0; U3 = 0; U4 = 0;

  if U1 <= 0
    U1 = 0;
  elseif U1 > 2 * masa_dron * a_gravedad
    U1 = 2 * masa_dron;
  endif

  % Cálculo de las aceleraciones
  ax = (U1 / masa_dron) * (cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi)) - (f_aero_lin / masa_dron) * vx;
  ay = (U1 / masa_dron) * (cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi)) - (f_aero_lin / masa_dron) * vy;
  az = (U1 / masa_dron) * (cos(phi) * cos(theta)) - a_gravedad - (f_aero_lin / masa_dron) * vz;

  % Actualizar velocidades lineales
  vx = vx + ax * dt;
  vy = vy + ay * dt;
  vz = vz + az * dt;

  % Actualizar posiciones espaciales
  x = x + vx * dt;
  y = y + vy * dt;
  z = z + vz * dt;

  % Detectamos colisión contra el suelo
  if z <= 0
    z = 0;
    vz = 0;
  endif

  % Guardamos la telemetría
  tiempo_actual = i * dt;
  hist_t(i)  = tiempo_actual;
  hist_z(i)  = z;
  hist_vz(i) = vz;

end

% Graficamos la altitud
figure(1);
plot(hist_t, hist_z, 'b', 'LineWidth', 2);
title('Caída libre del Dron (Z)');
xlabel('Tiempo (s)');
ylabel('Altitud (m)');
grid on;

