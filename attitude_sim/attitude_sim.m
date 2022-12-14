% Written partially by ChatGPT
% Set up serial communication
clear s;
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
target_mesh = trimesh(mesh_clist, mesh_points(:, 1), mesh_points(:, 2), mesh_points(:, 3), 'EdgeColor', 'blue');
actual_mesh = trimesh(mesh_clist, mesh_points(:, 1), mesh_points(:, 2), mesh_points(:, 3), 'EdgeColor', 'red');
diff_mesh = trimesh(mesh_clist, mesh_points(:, 1), mesh_points(:, 2), mesh_points(:, 3), 'EdgeColor', 'magenta');

% Set up quaternion variables
target_quat = [1 0 0 0]; % initialize quaternion
actual_quat = [1 0 0 0]; % initialize quaternion
diff_quat = [1 0 0 0]; % initialize quaternion
R = eye(3); % initialize rotation matrix
drawnow;

while true % loop indefinitely
    % Read in quaternion data from serial
    start = read(s, 1, "char");
    if (start == 'a')
        quat_data = read(s, 4, "single");
        if length(quat_data) == 4  % check if we received valid data
            actual_quat = quat_data;  % update quaternion
        end
        
        % Convert quaternion to rotation matrix
        R = quat2rotm(actual_quat);
        
        % Rotate vertices using rotation matrix
        mesh_points_rotated = (R * mesh_points')';
        
        % Update triangulation object with rotated vertices
        set(actual_mesh, 'Vertices', mesh_points_rotated)
        drawnow;
    end

    if (start == 't')
        quat_data = read(s, 4, "single");
        if length(quat_data) == 4  % check if we received valid data
            target_quat = quat_data;  % update quaternion
        end
        
        % Convert quaternion to rotation matrix
        R = quat2rotm(target_quat);
        
        % Rotate vertices using rotation matrix
        mesh_points_rotated = (R * mesh_points')';
        
        % Update triangulation object with rotated vertices
        set(target_mesh, 'Vertices', mesh_points_rotated)
        drawnow;
    end

    if (start == 'd')
        quat_data = read(s, 4, "single");
        if length(quat_data) == 4  % check if we received valid data
            diff_quat = quat_data;  % update quaternion
        end
        disp(diff_quat);
        % Convert quaternion to rotation matrix
        R = quat2rotm(diff_quat);
        
        % Rotate vertices using rotation matrix
        mesh_points_rotated = (R * mesh_points')';
        
        % Update triangulation object with rotated vertices
        set(diff_mesh, 'Vertices', mesh_points_rotated)
        drawnow;
    end
end

% Close serial communication when done
%clear s;