clc;
clear;

% ========================== Robot Parameters ==========================
l = 0.5; % Distance between wheel pairs (m)
d = 0.5; % Distance between wheels (m)
L_plus_D = l + d; % Combined geometric parameter

% Inverse kinematics matrix
J_inv = [1, -1, -L_plus_D;
         1,  1, -L_plus_D;
         1, -1,  L_plus_D;
         1,  1,  L_plus_D];

% ========================== Simulation Parameters ==========================
dt = 0.1; % Time step (s)
radius = 2; % Radius of the circle (m)
angular_velocity = 1; % Angular velocity for circular motion (rad/s)
T = 2 * pi / angular_velocity+0.1; % Time to complete one circle
steps = T / dt; % Number of steps for one circle
wait_steps = 0.5 / dt; % 0.5 seconds wait before starting

% Initialize position, orientation, and history
x_pos = 0; y_pos = -radius; phi = 0; % Start at bottom of the circle
x_history = []; y_history = []; phi_history = [];
x_dot_history = []; y_dot_history = []; phi_dot_history = [];

% ========================== Video Writer Setup ==========================
video = VideoWriter('Mecanum_Motion_Circle.avi'); % Create video file
video.FrameRate = 20; % Set frame rate
open(video); % Open video for writing

% Set fixed figure size for consistent frames
figure('Position', [100, 100, 1200, 900]);

% ========================= Simulation Loop ============================= 
for t = 1:steps + wait_steps
    if t <= wait_steps
        % Wait for 0.5 seconds (no movement)
        v1 = 0; v2 = 0; v3 = 0; v4 = 0;
    else
        % Set velocities for a circular trajectory
        % The robot moves in a circle by combining forward and rotational velocities
        v_forward = angular_velocity * radius; % Tangential velocity
        v_rotation = angular_velocity; % Rotational velocity

        % Wheel velocities for circular motion
        v1 = v_forward - v_rotation * L_plus_D;
        v2 = v_forward - v_rotation * L_plus_D;
        v3 = v_forward + v_rotation * L_plus_D;
        v4 = v_forward + v_rotation * L_plus_D;

        % Compute local velocities using the inverse kinematics matrix
        v = [v1; v2; v3; v4];
        velocities = J_inv \ v;
        x_dot_m = velocities(1);
        y_dot_m = velocities(2);
        phi_dot = velocities(3);

        % Store velocities
        x_dot_history = [x_dot_history, x_dot_m];
        y_dot_history = [y_dot_history, y_dot_m];
        phi_dot_history = [phi_dot_history, phi_dot];

        % Update positions
        x_pos = x_pos + (x_dot_m * cos(phi) - y_dot_m * sin(phi)) * dt;
        y_pos = y_pos + (x_dot_m * sin(phi) + y_dot_m * cos(phi)) * dt;
        phi = phi + phi_dot * dt;
    end

    % Store history
    x_history = [x_history, x_pos];
    y_history = [y_history, y_pos];
    phi_history = [phi_history, phi];

    % ===================== Visualization ======================
    % Clear current figure
    clf;

    % Plot 3D trajectory
    plot3(x_history, y_history, zeros(size(x_history)), 'b-', 'LineWidth', 2);
    hold on;

    % Draw the robot as a 3D box
    car_length = 0.4; % Robot length (m)
    car_width = 0.2;  % Robot width (m)
    car_height = 0.1; % Robot height (m)

    % Define corners of the robot box
    corners_bottom = [-car_length/2, -car_width/2, 0;
                       car_length/2, -car_width/2, 0;
                       car_length/2,  car_width/2, 0;
                      -car_length/2,  car_width/2, 0];

    corners_top = corners_bottom + [0, 0, car_height]; % Add height to top corners

    % Combine top and bottom faces
    corners = [corners_bottom; corners_top];

    % Apply rotation and translation
    R = [cos(phi), -sin(phi), 0;
         sin(phi),  cos(phi), 0;
         0,        0,        1]; % Rotation matrix
    rotated_corners = (R * corners')';
    translated_corners = rotated_corners + [x_pos, y_pos, 0];

    % Draw bottom face
    fill3(translated_corners(1:4, 1), translated_corners(1:4, 2), ...
          translated_corners(1:4, 3), 'g', 'FaceAlpha', 0.5);

    % Draw top face
    fill3(translated_corners(5:8, 1), translated_corners(5:8, 2), ...
          translated_corners(5:8, 3), 'g', 'FaceAlpha', 0.5);

    % Draw sides
    for i = 1:4
        fill3([translated_corners(i, 1), translated_corners(mod(i, 4)+1, 1), ...
               translated_corners(mod(i, 4)+5, 1), translated_corners(i+4, 1)], ...
              [translated_corners(i, 2), translated_corners(mod(i, 4)+1, 2), ...
               translated_corners(mod(i, 4)+5, 2), translated_corners(i+4, 2)], ...
              [translated_corners(i, 3), translated_corners(mod(i, 4)+1, 3), ...
               translated_corners(mod(i, 4)+5, 3), translated_corners(i+4, 3)], ...
              'g', 'FaceAlpha', 0.5);
    end

    % Draw 3D grid
    grid on;
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    zlabel('Z Position (m)');
    title('Mecanum Robot 3D Motion - Circular Trajectory');
    axis equal;
    zlim([-0.1, 0.5]); % Adjust Z-axis limits

    % Capture frame for the video
    frame = getframe(gcf);
    writeVideo(video, frame);

    hold off;
end

% ========================= Finalize Video ==========================
close(video);
disp('Video created: Mecanum_Motion_Circle.avi');

% ========================= Velocity Graphs ==========================
time_array = 0:dt:(steps - 1) * dt;

% Plot X velocity
figure;
plot(time_array, x_dot_history, 'b', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('X Velocity (m/s)');
title('X Velocity over Time');
grid on;

% Plot Y velocity
figure;
plot(time_array, y_dot_history, 'r', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Y Velocity (m/s)');
title('Y Velocity over Time');
grid on;

% Plot Rotational velocity
figure;
plot(time_array, phi_dot_history, 'g', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Rotational Velocity (rad/s)');
title('Rotational Velocity over Time');
grid on;

