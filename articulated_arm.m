% Radius of Cylinder
R_b = 10; % Radius of base
R_a = 7; % Radius of arm
R_l = 3; % Radius of link

% Height of Cylinder
H_b = 10; % Height of base
H_a = 10; % Height of arm
H_l = 20; % Height of link

% Init angle
theta1 = deg2rad(90);
theta2 = deg2rad(-45);
theta3 = deg2rad(60);

% Create base, arm and link
[x_b, y_b, z_b] = cylinder(R_b);
[x_a1, y_a1, z_a1] = cylinder(R_a);
[x_a2, y_a2, z_a2] = cylinder(R_a);
[x_l1, y_l1, z_l1] = cylinder(R_l);
[x_l2, y_l2, z_l2] = cylinder(R_l);
[x_l3, y_l3, z_l3] = cylinder(R_l);

% Scale the cylinders to the desired heights
z_b = z_b * H_b;
z_a1 = z_a1 * H_a;
z_a2 = z_a2 * H_a;
z_l1 = z_l1 * H_l;
z_l2 = z_l2 * H_l;
z_l3 = z_l3 * H_l;

% Link base and link 1 maxtrix
Matrix_Link1 = [cos(theta1), 0, sin(theta1), 0;
    sin(theta1), 0, -cos(theta1), 0;
    0, 1, 0, H_l + H_b;
    0, 0, 0, 1];

% Link link 1 and arm 1 maxtrix
Matrix_Link2 = [cos(theta2), -sin(theta2), 0, H_l*cos(theta2);
    sin(theta2), cos(theta2), 0, H_l*sin(theta2);
    0, 0, 1, 0;
    0, 0, 0, 1];

% Link link 2 and arm 2 maxtrix
Matrix_Link3 = [cos(theta3), -sin(theta3), 0, H_l*cos(theta3);
    sin(theta3), cos(theta3), 0, H_l*sin(theta3);
    0, 0, 1, 0;
    0, 0, 0, 1];
% Rotation Y matrix
rotY = [cos(pi/2),  0, sin(pi/2), 0;
    0,          1, 0,         0;
    -sin(pi/2), 0, cos(pi/2), 0;
    0,          0, 0,         1];


% Visualization
figure;
hold on;
grid on;
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('HoangLee Robotics');

% Link base and link 1
for j = 1:length(x_l1)
    for i = 1:2
        Pos1 = [x_l1(i, j); y_l1(i, j); z_l1(i, j) + H_b; 1];
        Pos2 = Pos1;
        x_l1(i, j) = Pos2(1);
        y_l1(i, j) = Pos2(2);
        z_l1(i, j) = Pos2(3);
    end
end

% Link link 1 and arm 1
for j = 1:length(x_a1)
    for i = 1:2
        Pos1 = [x_a1(i, j); y_a1(i, j); z_a1(i, j) - H_a/2; 1];
        Pos2 = Matrix_Link1 * Pos1;
        x_a1(i, j) = Pos2(1);
        y_a1(i, j) = Pos2(2);
        z_a1(i, j) = Pos2(3);
    end
end

% Link arm 1 and link 2
for j = 1:length(x_l2)
    for i = 1:2
        Pos1 = [x_l2(i, j); y_l2(i, j); z_l2(i, j) - R_a- H_l/2; 1];
        Pos2 = Matrix_Link1 * Matrix_Link2 * rotY * Pos1;
        x_l2(i, j) = Pos2(1);
        y_l2(i, j) = Pos2(2);
        z_l2(i, j) = Pos2(3);
    end
end

% Link link 2 and arm 2
for j = 1:length(x_a2)
    for i = 1:2
        Pos1 = [x_a2(i, j); y_a2(i, j); z_a2(i, j) - H_a/2; 1];
        Pos2 = Matrix_Link1 * Matrix_Link2  * Pos1;
        x_a2(i, j) = Pos2(1);
        y_a2(i, j) = Pos2(2);
        z_a2(i, j) = Pos2(3);
    end
end

% Link arm 2 and link 3
for j = 1:length(x_l3)
    for i = 1:2
        Pos1 = [x_l3(i, j); y_l3(i, j); z_l3(i, j) - R_a- H_l/2; 1];
        Pos2 = Matrix_Link1 * Matrix_Link2 * Matrix_Link3 * rotY * Pos1;
        x_l3(i, j) = Pos2(1);
        y_l3(i, j) = Pos2(2);
        z_l3(i, j) = Pos2(3);
    end
end


% Base
surf(x_b, y_b, z_b, 'FaceColor', 'red', 'EdgeColor', 'none');
fill3(x_b(1,:), y_b(1,:), z_b(1,:), 'r');
fill3(x_b(2,:), y_b(2,:), z_b(2,:), 'r');

% Link 1
surf(x_l1, y_l1, z_l1, 'FaceColor', 'blue', 'EdgeColor', 'none');
fill3(x_l1(1,:), y_l1(1,:), z_l1(1,:), 'b');
fill3(x_l1(2,:), y_l1(2,:), z_l1(2,:), 'b');

% Arm 1
surf(x_a1, y_a1, z_a1, 'FaceColor', 'green', 'EdgeColor', 'none');
fill3(x_a1(1,:), y_a1(1,:), z_a1(1,:), 'g');
fill3(x_a1(2,:), y_a1(2,:), z_a1(2,:), 'g');

% Link 2
surf(x_l2, y_l2, z_l2, 'FaceColor', 'yellow', 'EdgeColor', 'none');
fill3(x_l2(1,:), y_l2(1,:), z_l2(1,:), 'y');
fill3(x_l2(2,:), y_l2(2,:), z_l2(2,:), 'y');

% Arm 2
surf(x_a2, y_a2, z_a2, 'FaceColor', 'cyan', 'EdgeColor', 'none');
fill3(x_a2(1,:), y_a2(1,:), z_a2(1,:), 'c');
fill3(x_a2(2,:), y_a2(2,:), z_a2(2,:), 'c');

% Link 3
surf(x_l3, y_l3, z_l3, 'FaceColor', 'magenta', 'EdgeColor', 'none');
fill3(x_l3(1,:), y_l3(1,:), z_l3(1,:), 'm');
fill3(x_l3(2,:), y_l3(2,:), z_l3(2,:), 'm');

% Adjust the coodinate space
xlim([-80 80]);
ylim([-80 80]);
zlim([0 120]);

%  Turn on 3D rotation
rotate3d on;