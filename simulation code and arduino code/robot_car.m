% BB-8 Style Differential Drive Robot Simulation (over even terrain and frictionless zone)

clear all;
close all;
clc;

% Robot parameters
r = 0.05;           % Wheel radius (m)
L = 0.2;            % Distance between wheels (m)
robotRadius = 0.15; % Main body radius
headRadius = 0.08;  % BB-8-style head radius
eyeRadius = 0.02;   % BB-8 eye radius

% Simulation time
dt = 0.05;
simTime = 30;
t = 0:dt:simTime;
N = length(t);

% Initial pose
x = zeros(1, N);
y = zeros(1, N);
z = zeros(1, N); % flat even terrain
theta = zeros(1, N);

% Wheel angular velocities
omega_R = zeros(1, N);
omega_L = zeros(1, N);

% Motion pattern
for i = 1:N
    if i < N/4
        omega_R(i) = 5; omega_L(i) = 3;
    elseif i < N/2
        omega_R(i) = 3; omega_L(i) = 5;
    elseif i < 3*N/4
        omega_R(i) = 6; omega_L(i) = 2;
    else
        omega_R(i) = 4; omega_L(i) = 4;
    end
end

% Simulate motion
for i = 1:N-1
    v_R = r * omega_R(i);
    v_L = r * omega_L(i);

    v = (v_R + v_L) / 2;
    omega = (v_R - v_L) / L;

    x(i+1) = x(i) + v * cos(theta(i)) * dt;
    y(i+1) = y(i) + v * sin(theta(i)) * dt;
    theta(i+1) = theta(i) + omega * dt;
end

% Plot setup
figure('Name', 'BB-8 Inspired Robot', 'Color', 'white', 'Position', [100 100 800 600]);
ax = axes('XLim', [min(x)-1 max(x)+1], ...
          'YLim', [min(y)-1 max(y)+1], ...
          'ZLim', [0 0.5]);
view(3); grid on; hold on;
axis equal;
camlight headlight; lighting gouraud;

% Ground
[Xg, Yg] = meshgrid(min(x)-1:0.2:max(x)+1, min(y)-1:0.2:max(y)+1);
Zg = zeros(size(Xg));
surf(Xg, Yg, Zg, 'FaceColor', [0.95 0.95 0.95], 'EdgeColor', 'none', 'FaceAlpha', 0.5);

% Trajectory
plot3(x, y, robotRadius*ones(1,N), 'b-', 'LineWidth', 1);

% Create BB-8 styled body
[sx, sy, sz] = sphere(50);
bodyC = ones(size(sz));
bodyC(abs(sz) < 0.2) = 0.4; % colored belt
bodySurf = surf(robotRadius*sx, robotRadius*sy, robotRadius*sz);
set(bodySurf, 'FaceColor', 'interp', 'CData', bodyC, 'EdgeColor', 'none');

% Head (half sphere)
[hsx, hsy, hsz] = sphere(50);
hsz(hsz < 0) = 0;
headSurf = surf(headRadius*hsx, headRadius*hsy, headRadius*hsz);
set(headSurf, 'FaceColor', [0.2 0.6 0.8], 'EdgeColor', 'none');

% Eye (black dot)
[ex, ey, ez] = sphere(10);
eyeSurf = surf(eyeRadius*ex, eyeRadius*ey, eyeRadius*ez);
set(eyeSurf, 'FaceColor', [0 0 0], 'EdgeColor', 'none');

% Heading arrow
axisLength = 1.5 * robotRadius;
headingArrow = plot3([0 axisLength], [0 0], [0 0], 'r-', 'LineWidth', 2);

xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('BB-8 Robot on Even Terrain (Frictionless Zone)');

fprintf('Simulating BB-8 over even frictionless terrain...\n');

% Animation
frameSkip = max(round(N / (simTime * 20)), 1);
for i = 1:frameSkip:N
    ax.XLim = [x(i)-1, x(i)+1];
    ax.YLim = [y(i)-1, y(i)+1];

    % Body
    set(bodySurf, 'XData', robotRadius*sx + x(i), ...
                  'YData', robotRadius*sy + y(i), ...
                  'ZData', robotRadius*sz + z(i) + robotRadius);

    % Head
    headZ = z(i) + 2 * robotRadius;
    set(headSurf, 'XData', headRadius*hsx + x(i), ...
                  'YData', headRadius*hsy + y(i), ...
                  'ZData', headRadius*hsz + headZ);

    % Eye
    eyeX = x(i) + 0.9 * headRadius * cos(theta(i));
    eyeY = y(i) + 0.9 * headRadius * sin(theta(i));
    eyeZ = headZ + 0.02;
    set(eyeSurf, 'XData', eyeRadius*ex + eyeX, ...
                 'YData', eyeRadius*ey + eyeY, ...
                 'ZData', eyeRadius*ez + eyeZ);

    % Heading
    set(headingArrow, ...
        'XData', [x(i), x(i) + axisLength*cos(theta(i))], ...
        'YData', [y(i), y(i) + axisLength*sin(theta(i))], ...
        'ZData', [z(i)+robotRadius, z(i)+robotRadius]);

    drawnow;
    pause(0.01);
end

fprintf('Simulation complete!\n');
