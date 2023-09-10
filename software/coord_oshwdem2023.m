% Coordenadas para el circuito de la OSHWDem 2023 de Coruña

function [dim_cto origen_cto tramos_cto marca_salida] = coord_oshwdem2023()
%% Dimensiones del circuito
X_cto = 5000; % mm
Y_cto = 2500; % mm
dim_cto = [X_cto Y_cto];

%% Punto y direccion en origen
x0_pos = 1000;
y0_pos = 280;
x0_dir = -1;
y0_dir = 0;
origen_cto = [x0_pos y0_pos x0_dir y0_dir];

%% Trazado [tipo, longitud]
    % tipo:
        % 0 = recta
        % ang = angulo de curva a izquierda en grados
        % -ang = angulo de curva a derecha en grados
    % longitud:
        % distancia en mm para recta
        % radio en mm para curva

numero_de_tramos = 28;
tramos_cto = zeros(numero_de_tramos,2);

tramos_cto(1,:) = [-90 700];
tramos_cto(2,:) = [0 750];
tramos_cto(3,:) = [-190 500];
tramos_cto(4,:) = [0 310];
tramos_cto(5,:) = [100 500];
tramos_cto(6,:) = [0 20];
tramos_cto(7,:) = [6 700];
tramos_cto(8,:) = [-17 700];
tramos_cto(9,:) = [22 700];
tramos_cto(10,:) = [-22 700];
tramos_cto(11,:) = [22 700];
tramos_cto(12,:) = [-22 700];
tramos_cto(13,:) = [22 700];
tramos_cto(14,:) = [-17 700];
tramos_cto(15,:) = [6 700];
tramos_cto(16,:) = [0 20];
tramos_cto(17,:) = [207 500];
tramos_cto(18,:) = [0 950];
tramos_cto(19,:) = [-207 520];
tramos_cto(20,:) = [0 1250];
tramos_cto(21,:) = [-40 1300];
tramos_cto(22,:) = [-20 720];
tramos_cto(23,:) = [-10 480];
tramos_cto(24,:) = [-10 1000];
tramos_cto(25,:) = [-10 1000];
tramos_cto(26,:) = [-25 1300];
tramos_cto(27,:) = [-1 0];
tramos_cto(28,:) = [0 0];
% Los ultimos dos tramos se dejan con la longitud/radio a 0 y con el angulo
% de curva a 1 o -1 para que el programa los calcule de manera optima para
% cerrar el circuito.

marca_salida = 1800; % distancia entre el origen y la primera marca de salida en mm