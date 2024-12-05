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
distance = 4; % Side length of the square (m)
speed = 2; % Linear speed (m/s)
T_per_side = distance / speed; % Time to traverse one side
steps_per_side = T_per_side / dt;
total_steps = 4 * steps_per_side;
wait_steps = 0.5 / dt; % Wait for 0.5 seconds before starting

% Initialize position, orientation, and history
x_pos = 0; y_pos = 0; phi = 0; % Initial position and orientation
x_history = []; y_history = []; phi_history = [];
x_dot_history = []; y_dot_history = []; phi_dot_history = [];

% ========================== Video Writer Setup ==========================
video = VideoWriter('Mecanum_Motion_Square.avi'); % Create video file
video.FrameRate = 20; % Set frame rate
open(video); % Open video for writing

% Set fixed figure size for consistent frames
figure('Position', [100, 100, 1200, 900]);

% ========================= Simulation Loop =============================
for t = 1:(total_steps + wait_steps)
    if t <= wait_steps
        % Wait for 0.5 seconds (no movement)
        v1 = 0; v2 = 0; v3 = 0; v4 = 0;
    elseif t <= wait_steps + steps_per_side
        % Move forward
        v1 = 2; v2 = 2; v3 = 2; v4 = 2;
    elseif t <= wait_steps + 2 * steps_per_side
        % Move right (sideways)
        v1 = -2; v2 = 2; v3 = -2; v4 = 2;
    elseif t <= wait_steps + 3 * steps_per_side
        % Move backward
        v1 = -2; v2 = -2; v3 = -2; v4 = -2;
    else
        % Move left (sideways)
        v1 = 2; v2 = -2; v3 = 2; v4 = -2;
    end

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
    x_pos = x_pos + x_dot_m * dt;
    y_pos = y_pos + y_dot_m * dt;
    phi = phi + phi_dot * dt;

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

    % Draw wheels
    [zc, yc, xc] = cylinder(0.05, 20); % Wheel radius = 0.05, 20 points for smoothness
    xc = xc * 0.02; % Wheel thickness = 0.02

    % Apply 90° rotation around Z-axis to wheels
    temp = xc;
    xc = -yc; % Swap X and Y for 90° rotation
    yc = temp;

    % Define exact wheel positions relative to car body
    wheel_positions = [
        -car_length/2, -car_width/2-0.023, 0;  % Bottom-left corner of the robot
        car_length/2, -car_width/2-0.023, 0;  % Bottom-right corner of the robot
        car_length/2,  car_width/2, 0;  % Top-right corner of the robot
        -car_length/2,  car_width/2, 0]; % Top-left corner of the robot

    % Draw each wheel
    for i = 1:4
        % Transform wheel position based on robot's current pose
        wheel_center = (R * wheel_positions(i, :)')' + [x_pos, y_pos, 0];
        wheel_geometry = R * [xc(:), yc(:), zc(:)]'; % Rotate wheel geometry
        wheel_x = reshape(wheel_geometry(1, :), size(xc)) + wheel_center(1);
        wheel_y = reshape(wheel_geometry(2, :), size(yc)) + wheel_center(2);
        wheel_z = reshape(wheel_geometry(3, :), size(zc)) + wheel_center(3);

        % Draw cylindrical side of the wheel
        surf(wheel_x, wheel_y, wheel_z, 'FaceColor', [1, 0.5, 0], 'EdgeColor', 'none');

        % Draw circular ends
        fill3(wheel_x(1, :), wheel_y(1, :), wheel_z(1, :), [1, 0.5, 0]);
        fill3(wheel_x(2, :), wheel_y(2, :), wheel_z(2, :), [1, 0.5, 0]);
    end

    % Draw 3D grid
    grid on;
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    zlabel('Z Position (m)');
    title('Mecanum Robot 3D Motion - Square Trajectory');
    axis equal;
    zlim([-0.1, 0.5]); % Adjust Z-axis limits

    % Capture frame for the video
    frame = getframe(gcf);
    writeVideo(video, frame);

    hold off;
end

% ========================= Finalize Video ==========================
close(video);
% disp('Video created: Mecanum_Motion_Square.avi');

% ========================= Velocity Graphs ==========================
time_array = 0:dt:(total_steps + wait_steps - 1) * dt;

% % Plot X velocity
% figure;
% plot(time_array, x_dot_history, 'b', 'LineWidth', 1.5);
% xlabel('Time (s)');
% ylabel('X Velocity (m/s)');
% title('X Velocity over Time');
% grid on;

% % Plot Y velocity
% figure;
% plot(time_array, y_dot_history, 'r', 'LineWidth', 1.5);
% xlabel('Time (s)');
% ylabel('Y Velocity (m/s)');
% title('Y Velocity over Time');
% grid on;

% % Plot Rotational velocity
% figure;
% plot(time_array, phi_dot_history, 'g', 'LineWidth', 1.5);
% xlabel('Time (s)');
% ylabel('Rotational Velocity (rad/s)');
% title('Rotational Velocity over Time');
% grid on;

% Function to plot velocity data
function plot_velocity(time_array, velocity_data, color, ylabel_text, title_text)
    figure;
    plot(time_array, velocity_data, color, 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel(ylabel_text);
    title(title_text);
    grid on;
end

% Plot X velocity
plot_velocity(time_array, x_dot_history, 'b', 'X Velocity (m/s)', 'X Velocity over Time');

% Plot Y velocity
plot_velocity(time_array, y_dot_history, 'r', 'Y Velocity (m/s)', 'Y Velocity over Time');

% Plot Rotational velocity
plot_velocity(time_array, phi_dot_history, 'g', 'Rotational Velocity (rad/s)', 'Rotational Velocity over Time');

