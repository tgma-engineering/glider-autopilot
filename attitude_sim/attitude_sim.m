% Written partially by ChatGPT
% Set up serial communication
s = serialport("COM4", 115200);
flush(s);

% Set up figure and axes for plottings
figure;
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
xlim([-.2, .2]);
ylim([-.2, .2]);
zlim([-.2, .2]);
hold on;

view(0, 20)

% Load STL mesh
mesh = stlread('drone.stl');
% Rotate model such that it is looking into positive y-direction
mesh_points = ([cos(pi/2) -sin(pi/2) 0; sin(pi/2) cos(pi/2) 0; 0 0 1] * mesh.Points')';
mesh_clist = mesh.ConnectivityList;
mesh = trimesh(mesh_clist, mesh_points(:, 1), mesh_points(:, 2), mesh_points(:, 3));

% Create patch object for plotting
%p = patch('Vertices', vertices, 'Faces', faces, 'FaceColor', 'blue', 'EdgeColor', 'none');

% Set up quaternion variables
quat = [1 0 0 0]; % initialize quaternion
R = eye(3); % initialize rotation matrix

while true % loop indefinitely
    % Read in quaternion data from serial
    start = read(s, 1, "char");
    if (start == 's')
        quat_data = read(s, 4, "single");
        if length(quat_data) == 4 % check if we received valid data
            quat = quat_data; % update quaternion
        end
        
        % Convert quaternion to rotation matrix
        R = quat2rotm(quat);
        
        % Rotate vertices using rotation matrix
        mesh_points_rotated = (R * mesh_points')';
        
        % Update triangulation object with rotated vertices
        set(mesh, 'Vertices', mesh_points_rotated)
        drawnow;
    end
end

% Close serial communication when done
%clear s;