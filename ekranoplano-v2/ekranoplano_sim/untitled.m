% 1. Cargar la grabación (ajusta la ruta a donde se guardó la carpeta 'vuelo_ekranoplano')
bagFolder = '/home/facyu/vuelo_ekranoplano_2';
bagReader = ros2bagreader(bagFolder);

% 2. Leer los mensajes del tópico model_states
msgs = readMessages(select(bagReader, 'Topic', '/gazebo/model_states'));

% 3. Extraer la posición X e Y del ekranoplano
% En tu log, 'ocean' es el índice 1 (en MATLAB) y 'ekranoplano' es el índice 2.
num_msgs = length(msgs);
x_pos = zeros(num_msgs, 1);
y_pos = zeros(num_msgs, 1);

for i = 1:num_msgs
    % Extraemos la posición del segundo elemento (índice 2) de la lista de poses
    x_pos(i) = msgs{i}.pose(2).position.x;
    y_pos(i) = msgs{i}.pose(2).position.y;
end

% 4. Graficar la trayectoria Vista en Planta (Izquierda / Derecha)
figure('Name', 'Trayectoria desde ROS 2 (Gazebo)', 'Color', 'w');
hold on; grid on;

% Dibujar la línea de trayectoria
plot(y_pos, x_pos, 'b-', 'LineWidth', 2);

% Marcar inicio y fin
plot(y_pos(1), x_pos(1), 'ko', 'MarkerFaceColor', 'g', 'MarkerSize', 8); % Inicio verde
plot(y_pos(end), x_pos(end), 'ks', 'MarkerFaceColor', 'r', 'MarkerSize', 8); % Fin rojo

% Detalles de la gráfica
xlabel('Desplazamiento Lateral - Este (Y) [m]', 'FontWeight', 'bold');
ylabel('Avance Frontal - Norte (X) [m]', 'FontWeight', 'bold');
title('Vista Superior de la Trayectoria del Ekranoplano', 'FontSize', 14);
legend('Trayectoria', 'Inicio', 'Fin', 'Location', 'best');

% Mantener proporción 1:1 para que la curva no se vea deformada
axis equal; 
