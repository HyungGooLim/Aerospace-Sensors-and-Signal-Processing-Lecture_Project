% data = readtable('square.xlsx');
% data = table2array(square);

%Accel is measured in body frame
accel_x = data(:,1);
accel_y = data(:,3);
accel_z = data(:,5);
accel_bodyf = [accel_x accel_y accel_z];
dt = 0.01;

%Knowing timestep : t
size_accel = size(accel_x);
t = size_accel(1);

%Pose is measured in navigation frame
phi = data(:,7);
theta = data(:,9);
psi = data(:,11);

%Declare the variable
v_x = zeros(1,t);
v_y = zeros(1,t);
v_z = zeros(1,t);
p_x = zeros(1,t);
p_y = zeros(1,t);
p_z = zeros(1,t);

%We make accel{b} to accel{n}
for n=1:t
    % We can make Direction Cosine Matrix with DMP value%
    %(Using 3-2-1 Euler Angle)%
    Rotation_x = [1 0 0 ; 0 cos(phi(n)) -1*sin(phi(n)) ; 0 sin(phi(n)) cos(phi(n))];
    Rotation_y = [cos(theta(n)) 0 sin(theta(n)) ; 0 1 0 ; -1*sin(theta(n)) 0 cos(theta(n))];
    Rotation_z = [cos(psi(n)) -1*sin(psi(n)) 0 ; sin(psi(n)) cos(psi(n)) 0; 0 0 1];

    DCM = Rotation_z*Rotation_y*Rotation_x;    
    accel_navf = DCM*accel_bodyf(n,:)';
    
    %We have to gravity correlation in Navframe
    accel(n,:) = accel_navf' - [0; 0; -9.8]';
end


for k=2:t
% we wil get navigation velocity
v_x(k) = v_x(k-1) + accel(k,1) * dt;
v_y(k) = v_y(k-1) + accel(k,2) * dt;
v_z(k) = v_z(k-1) + accel(k,3) * dt;

% we will integral the velocity to position
p_x(k) = p_x(k-1) + v_x(k)* dt;
p_y(k) = p_y(k-1) + v_y(k)* dt;
p_z(k) = p_z(k-1) + v_z(k)* dt;
end

% p_x = -1*p_x;
% p_y = -1*p_y;
% p_z = -1*p_z;

velocity = [v_x ;v_y ;v_z];
position = [p_x ;p_y ;p_z];
scatter3(p_y,p_x,p_z);
xlabel('Position X');
ylabel('Position Y');
zlabel('Position Z');