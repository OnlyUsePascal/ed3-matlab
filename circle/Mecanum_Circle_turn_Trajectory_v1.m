% Mecanum Wheel Robot Simulation with Circular Trajectory and Proper Wheels
clc;
clear;

% ========================== Forward Kinematics ==========================
% Robot parameters
l = 0.5; % Distance between wheel pairs (m)
d = 0.5; % Distance between wheels along the axis (m)

% Forward kinematics matrix (pseudo-inverse of the kinematic model)
J_inv = (1 / 4) * [1, -1, -(l + d);
                   1,  1, -(l + d);
                   1, -1,  (l + d);
                   1,  1,  (l + d)];

% ========================== Simulation Setup ============================
% Time parameters
dt = 0.1; % Time step (s)
t_total = 3; % Total time for completing the circle (s)
t_steps = t_total / dt; % Total steps
time = linspace(0, t_total, t_steps); % Time array

% Circle parameters
radius = 1; % Radius of the circle (m)
angular_velocity = 2 * pi / t_total; % Constant angular velocity (rad/s)
linear_velocity = angular_velocity * radius; % Linear velocity (m/s)

% Initialize position, orientation, and velocity history
x_pos = 0; % Start on the circle at x = radius
y_pos = 0; % Start at y = 0
phi = 0; % Robot initially facing forward along the x-axis

x_history = [];
y_history = [];
phi_history = [];
v1_history = [];
v2_history = [];
v3_history = [];
v4_history = [];

% ========================= Video Writer Setup ===========================
video = VideoWriter('Circular_Trajectory_with_Turning_Robot.avi'); % Create video file
video.FrameRate = 20; % Set frame rate
open(video); % Open video for writing

% Set fixed figure size for consistent frames
figure('Position', [100, 100, 1200, 900]); % Increase figure size for buffer

% ========================= Robot Motion Simulation ======================
for t = 1:(t_steps + 5) % Additional 5 steps for 0.5-second delay
    % During the first 0.5 second, keep the robot stationary
    if t <= 5 % 1 second of delay (5 steps with dt = 0.1)
        x_vel = 0;
        y_vel = 0;
        phi_vel = 0;
    else
        % Compute linear and angular velocities for the robot
        x_vel = linear_velocity * cos(phi); % X velocity depends on orientation
        y_vel = linear_velocity * sin(phi); % Y velocity depends on orientation
        phi_vel = angular_velocity; % Robot turns left at constant angular velocity
    end

    % Compute wheel velocities
    wheel_velocities = J_inv * [x_vel; y_vel; phi_vel];

    % Save wheel velocities for plotting
    v1_history = [v1_history, wheel_velocities(1)];
    v2_history = [v2_history, wheel_velocities(2)];
    v3_history = [v3_history, wheel_velocities(3)];
    v4_history = [v4_history, wheel_velocities(4)];

    % Update robot pose
    x_pos = x_pos + (x_vel * dt);
    y_pos = y_pos + (y_vel * dt);
    phi = phi + phi_vel * dt;

    % Normalize phi to stay between -pi and pi
    phi = mod(phi + pi, 2 * pi) - pi;

    % Store history for plotting
    x_history = [x_history, x_pos];
    y_history = [y_history, y_pos];
    phi_history = [phi_history, phi];

    % ===================== Visualization Per Frame ======================
    % Plot trajectory up to current step
    plot3(x_history, y_history, zeros(size(x_history)), 'b-', 'LineWidth', 2);
    hold on;

    % Draw car model (fully enclosed 3D box)
    car_length = 0.4; % Length of the car
    car_width = 0.2; % Width of the car
    car_height = 0.1; % Height of the car

    % Define the 8 corners of the box
    corners_bottom = [-car_length/2, -car_width/2, 0;
                       car_length/2, -car_width/2, 0;
                       car_length/2,  car_width/2, 0;
                      -car_length/2,  car_width/2, 0];

    corners_top = corners_bottom + [0, 0, car_height]; % Add height to top corners

    % Combine top and bottom faces
    corners = [corners_bottom; corners_top];

    % Rotate and translate car model
    R = [cos(phi), -sin(phi), 0;
         sin(phi),  cos(phi), 0;
         0,        0,        1]; % 3D rotation matrix

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

    % Formatting
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    zlabel('Z Position (m)');
    title('Circular Trajectory with Turning Robot');
    grid on;
    axis equal;
    zlim([-0.1 0.5]); % Adjust Z-axis limits

    % Capture frame and write to video
    frame = getframe(gcf);
    writeVideo(video, frame);

    hold off;
end

% ========================= Finalize Video ============================
close(video); % Close and save video file

% ========================= Velocity Graphs ============================
time_array = (0:(length(v1_history)-1)) * dt; % Create a time array in seconds

% Individual wheel angular velocities
figure;
plot(time_array, v1_history, 'r-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
title('Wheel 1 Angular Velocity');

figure;
plot(time_array, v2_history, 'g-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
title('Wheel 2 Angular Velocity');

figure;
plot(time_array, v3_history, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
title('Wheel 3 Angular Velocity');

figure;
plot(time_array, v4_history, 'k-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
title('Wheel 4 Angular Velocity');

% Combined wheel angular velocities
figure;
hold on;
plot(time_array, v1_history, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Wheel 1');
plot(time_array, v2_history, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Wheel 2');
plot(time_array, v3_history, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Wheel 3');
plot(time_array, v4_history, 'k-', 'LineWidth', 1.5, 'DisplayName', 'Wheel 4');
hold off;
grid on;
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
title('Combined Wheel Angular Velocities');
legend('show');

