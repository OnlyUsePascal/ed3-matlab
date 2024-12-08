% Mecanum Wheel Square Trajectory Simulation with Proper Wheel Alignment and Velocity Graphs
clc;
clear;

global wheel_positions xc yc zc corners time_array J_inv v1_history v2_history v3_history v4_history phi x_pos y_pos; % Declare global variables for wheel positions and wheel geometry
global vCarX_history vCarY_history vCarPhi_history; % Declare global variables for car velocities
global video; % Declare global variable for video writer
global x_history y_history phi_history; % Declare global variables for position and orientation history
global phi1_history phi2_history phi3_history phi4_history; % Declare global variables for wheel orientation history
global phi1_pos phi2_pos phi3_pos phi4_pos; % Declare global variables for wheel orientation


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
phi1_pos = 0; % Wheel 1 orientation (rad)
phi2_pos = 0; % Wheel 2 orientation (rad)
phi3_pos = 0; % Wheel 3 orientation (rad)
phi4_pos = 0; % Wheel 4 orientation (rad)

% Initialize history arrays
x_history = [0];
y_history = [0];
phi_history = [0];

v1_history = [0];
v2_history = [0];
v3_history = [0];
v4_history = [0];
vCarX_history = [0];
vCarY_history = [0];
vCarPhi_history = [0];

phi1_history = [0];
phi2_history = [0];
phi3_history = [0];
phi4_history = [0];

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

function move(distance, degree, totalTime)
    global time_array phi x_pos y_pos J_inv v1_history v2_history v3_history v4_history x_history y_history phi_history; % Declare global variables for time, position, orientation, and velocity history
    global vCarX_history vCarY_history vCarPhi_history; % Declare global variables for car velocities
    global video; % Declare global variable for video writer
    global phi1_pos phi2_pos phi3_pos phi4_pos; % Declare global variables for wheel orientation
    global phi1_history phi2_history phi3_history phi4_history; % Declare global variables for wheel orientation history

    % constant for now 
    step = 20;
    dt = totalTime / step;
    dts = (0:(step)) * dt;
    time_array = dts;
    
    % prepare vCarX, vCary, vCarPhi
    rad = deg2rad(degree); % s
    vCarX = distance * cos(rad) / totalTime;
    vCarY = distance * sin(rad) / totalTime;
    vCarPhi = rad / totalTime;
    
    fprintf('vx: %d, vy: %d, vphi: %d\n', vCarX, vCarY, vCarPhi);
    % start at step 1, end at last step
    for i = 2:length(dts)
        % Update robot pose
        dPhi = vCarPhi * dt;
        phi = phi + dPhi;
        x_pos = x_pos + (vCarX * cos(phi) - vCarY * sin(phi)) * dt;
        y_pos = y_pos + (vCarX * sin(phi) + vCarY * cos(phi)) * dt;
        
        fprintf('x: %d, y: %d, phi: %d\n', x_pos, y_pos, phi);
        
        % Compute wheel angular velocities
        wheelAngularVelocities = J_inv * [vCarX; vCarY; vCarPhi];
        phi1_pos = phi1_pos + wheelAngularVelocities(1) * dt;
        phi2_pos = phi2_pos + wheelAngularVelocities(2) * dt;
        phi3_pos = phi3_pos + wheelAngularVelocities(3) * dt;
        phi4_pos = phi4_pos + wheelAngularVelocities(4) * dt;

        % Save wheel angular velocities for plotting
        v1_history = [v1_history, wheelAngularVelocities(1)];
        v2_history = [v2_history, wheelAngularVelocities(2)];
        v3_history = [v3_history, wheelAngularVelocities(3)];
        v4_history = [v4_history, wheelAngularVelocities(4)];
        vCarX_history = [vCarX_history, vCarX];
        vCarY_history = [vCarY_history, vCarY];
        vCarPhi_history = [vCarPhi_history, vCarPhi];

        % Store history for plotting
        x_history = [x_history, x_pos];
        y_history = [y_history, y_pos];
        phi_history = [phi_history, phi];        

        % wheel angular position
        phi1_history = [phi1_history, phi1_pos];       
        phi2_history = [phi2_history, phi2_pos];
        phi3_history = [phi3_history, phi3_pos];
        phi4_history = [phi4_history, phi4_pos];

        % ===================== Visualization Per Frame ======================
        % Plot trajectory up to current step
        plot3(x_history, y_history, zeros(size(x_history)), 'b-', 'LineWidth', 2);
        hold on;
        
        % disp(x_pos);
        % disp(y_pos);
        % disp(phi);
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
end

move(10, 360, 10);

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

function plot_velocities(time_array, velocities, colors, yLabel, titleLabel, legends)
    figure;
    hold on;
    for i = 1:length(velocities)
        % disp(i); 
        % disp(velocities{i});
        % disp(colors(i));
        % disp(legends(i));
        plot(time_array, velocities{i}, colors(i), 'LineWidth', 1.5, 'DisplayName', legends(i));
    end
    hold off;
    grid on;
    xlabel('Time (s)');
    ylabel(yLabel);
    title(titleLabel);
    legend('show');
end

% plot_velocity(time_array, v1_history, 'r-', 'Angular Velocity (rad/s)', 'Wheel 1 Angular Velocity');
% plot_velocity(time_array, v2_history, 'g-', 'Angular Velocity (rad/s)', 'Wheel 2 Angular Velocity');
% plot_velocity(time_array, v3_history, 'b-', 'Angular Velocity (rad/s)', 'Wheel 3 Angular Velocity');
% plot_velocity(time_array, v4_history, 'k-', 'Angular Velocity (rad/s)', 'Wheel 4 Angular Velocity');
plot_velocities(time_array, {v1_history; v2_history; v3_history; v4_history}, ["r-"; "g-"; "b-"; "k-"], "Angular Velocity (rad/s)", "Wheel Angular Velocities", ["Wheel 1"; "Wheel 2"; "Wheel 3"; "Wheel 4"]);

% convert wheel angular position to degree
% phi1_history = rad2deg(phi1_history);
% phi2_history = rad2deg(phi2_history);
% phi3_history = rad2deg(phi3_history);
% phi4_history = rad2deg(phi4_history);

% TODO: divide y-axis by pi

% plot_velocity(time_array, phi1_history, 'r-', 'Angular Position (rad)', 'Wheel 1 Angular Position');
% plot_velocity(time_array, phi2_history, 'g-', 'Angular Position (rad)', 'Wheel 2 Angular Position');
% plot_velocity(time_array, phi3_history, 'b-', 'Angular Position (rad)', 'Wheel 3 Angular Position');
% plot_velocity(time_array, phi4_history, 'k-', 'Angular Position (rad)', 'Wheel 4 Angular Position');

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
