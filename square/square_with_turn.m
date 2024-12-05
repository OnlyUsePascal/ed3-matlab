% Mecanum Wheel Square Trajectory Simulation with Proper Wheel Alignment and Velocity Graphs
clc;
clear;
global wheel_positions xc yc zc corners; % Declare global variables for wheel positions and wheel geometry

% ========================== Inverse Kinematics ==========================
% Robot parameters
l = 0.5; % Distance between wheel pairs (m)
d = 0.5; % Distance between wheels along the axis (m)

% Inverse kinematics matrix 
J_inv = [1, -1, -(l + d);
         1,  1, -(l + d);
         1, -1,  (l + d);
         1,  1,  (l + d)];

% ========================== Simulation Setup ============================
% Time parameters
dt = 0.1; % Time step (s)
distance_per_side = 4; % Distance per side (m)
speed = 5.0; % Linear speed (m/s)
steps_per_side = distance_per_side / (speed * dt); % Time steps to move straight
steps_per_turn = 10; % Time steps to turn 90°

% Initialize position, orientation, and velocity history
x_pos = 0; % X position (m)
y_pos = 0; % Y position (m)
phi = 0; % Orientation (rad)

% Initialize history arrays
x_history = [];
y_history = [];
phi_history = [];

v1_history = [];
v2_history = [];
v3_history = [];
v4_history = [];
vx_history = [];
vy_history = [];
vphi_history = [];

% ========================== Drawing Setup ============================
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

% ========================= Video Writer Setup ===========================
video = VideoWriter('Square With Turn.avi'); % Create video file
video.FrameRate = 20; % Set frame rate
open(video); % Open video for writing

% Set fixed figure size for consistent frames
figure('Position', [100, 100, 1200, 900]); % Increase figure size for buffer

% ========================= Robot Motion Simulation ======================
function draw_car_body(R, x_pos, y_pos)
    global corners; % Declare global variable for car corners
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
end

function draw_wheels(R, x_pos, y_pos)
    global wheel_positions xc yc zc; % Declare global variables for wheel positions and wheel geometry
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
    
end

for t = 1:(4 * steps_per_side + 4 * steps_per_turn + 10) % Added 10 steps for the 1-second wait
    % During the first 1 second, set all velocities to zero
    if t <= 10 % Assuming dt = 0.1, 10 steps correspond to 1 second
        vx = 0;
        vy = 0;
        vphi = 0;
    elseif mod(t - 10, steps_per_side + steps_per_turn) <= steps_per_side && mod(t - 10, steps_per_side + steps_per_turn) > 0 % Straight motion
        vx = speed; % Move forward at constant speed
        vy = 0;
        vphi = 0;
    else % Turning motion
        vx = 0;
        vy = 0;
        vphi = pi / 2 / (steps_per_turn * dt); % Angular velocity for 90° turn
    end
    
    % Compute wheel angular velocities
    wheel_angular_velocities = J_inv * [vx; vy; vphi];

    % Update robot pose
    x_pos = x_pos + (vx * cos(phi) - vy * sin(phi)) * dt;
    y_pos = y_pos + (vx * sin(phi) + vy * cos(phi)) * dt;
    phi = phi + vphi * dt;

    % Save wheel angular velocities for plotting
    v1_history = [v1_history, wheel_angular_velocities(1)];
    v2_history = [v2_history, wheel_angular_velocities(2)];
    v3_history = [v3_history, wheel_angular_velocities(3)];
    v4_history = [v4_history, wheel_angular_velocities(4)];
    vx_history = [vx_history, vx];
    vy_history = [vy_history, vy];
    vphi_history = [vphi_history, vphi];

    % Store history for plotting
    x_history = [x_history, x_pos];
    y_history = [y_history, y_pos];
    phi_history = [phi_history, phi];

    % ===================== Visualization Per Frame ======================
    % Plot trajectory up to current step
    plot3(x_history, y_history, zeros(size(x_history)), 'b-', 'LineWidth', 2);
    hold on;

    % Rotate and translate car model
    R = [cos(phi), -sin(phi), 0;
         sin(phi),  cos(phi), 0;
         0,        0,        1]; % 3D rotation matrix

    draw_car_body(R, x_pos, y_pos); 

    % Draw each wheel
    draw_wheels(R, x_pos, y_pos);
    
    % Formatting
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    zlabel('Z Position (m)');
    title('Square Trajectory with Correctly Rotated Wheels');
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

function plot_velocity(time_array, velocity_data, color, y_label_text, title_text)
    figure;
    plot(time_array, velocity_data, color, 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel(y_label_text);
    title(title_text);
    grid on;
end

plot_velocity(time_array, v1_history, 'r-', 'Angular Velocity (rad/s)', 'Wheel 1 Angular Velocity');
plot_velocity(time_array, v2_history, 'g-', 'Angular Velocity (rad/s)', 'Wheel 2 Angular Velocity');
plot_velocity(time_array, v3_history, 'b-', 'Angular Velocity (rad/s)', 'Wheel 3 Angular Velocity');
plot_velocity(time_array, v4_history, 'k-', 'Angular Velocity (rad/s)', 'Wheel 4 Angular Velocity');

% % Combined wheel angular velocities
% figure;
% hold on;
% plot(time_array, v1_history, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Wheel 1');
% plot(time_array, v2_history, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Wheel 2');
% plot(time_array, v3_history, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Wheel 3');
% plot(time_array, v4_history, 'k-', 'LineWidth', 1.5, 'DisplayName', 'Wheel 4');
% hold off;
% grid on;
% xlabel('Time (s)');
% ylabel('Angular Velocity (rad/s)');
% title('Combined Wheel Angular Velocities');
% legend('show');
