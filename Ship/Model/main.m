% Load parameter for the Simulink model 

dim_x = 0.61;
dim_y = 0.206;
dim_z = 0.063;
roh_boat =  192.22156; % kg.m^ -3
roh_water = 977; % kg.m^ -3
m_boat = roh_boat * dim_x * dim_y * dim_z;
m_propeller = 0.344;    % kg
m = m_boat + m_propeller;
g = 9.81;   % m/ s^2
Fg = [0; 0; -2 * m_boat * g];
V_subm = m_boat / roh_water; % 0.5 * dim_x * dim_y * dim_z * 2; % m / roh_water; % in equilibrium
Fb = [0; 0; 2 * roh_water * V_subm * g];
Ly = 0.206;
Lz = 0.154;
k = 0.2;
b = 0.001;
D = 100*[1,0,0;
    0,1,0;
    0,0,1];
% R = [1, 0, 0; 
%     0, cos(phi), -sin(phi); 
%     0, sin(phi), cos(phi);];
Ixx_boat = (m_boat/12) * (dim_y^2 + dim_z^2) + m_boat * Ly^2;
Iyy_boat = (m_boat/12) * (dim_x^2 + dim_z^2);
Izz_boat = (m_boat/12) * (dim_x^2 + dim_y^2) + m_boat * Ly^2;
I_boat = [2 * Ixx_boat,0,0;
        0, 2 * Iyy_boat,0;
        0,0, 2 * Izz_boat];

R_propeller = 0.1 / 2;  % m
l_propeller = 0.113;    % m
z_propeller = 0.154;    % m

% I propeller around its own axis
Ixx_propeller = (1/2) * m_propeller * R_propeller^2; 
Iyy_propeller = (1/12) * m_propeller * (3 * R_propeller^2 + l_propeller^2);
Izz_propeller = Iyy_propeller;

% I propeller around cuboid axis
Ixx_propeller_cuboid = Ixx_propeller + m_propeller * z_propeller^2; 
Iyy_propeller_cuboid = Iyy_propeller + m_propeller * z_propeller^2;
Izz_propeller_cuboid = Izz_propeller;

Ixx_propeller_ship = Ixx_propeller_cuboid + m_propeller * Ly^2; 
Iyy_propeller_ship = Iyy_propeller_cuboid;
Izz_propeller_ship = Izz_propeller_cuboid + m_propeller * Ly^2;

I_propeller = [2 * Ixx_propeller_ship, 0, 0;
        0, 2 * Iyy_propeller_ship, 0;
        0, 0, 2 * Izz_propeller_ship];

% get total moment of inertia
I = I_boat + I_propeller;

x_init = [0;0;0];
yaw_init = 0;

%% 
data = out.x_state1.Data;
sz = size(data)
data_res = reshape(data, [3,sz(3)]);
figure()
plot(data_res(1,:), data_res(2,:))
xlabel('x')
ylabel('y')
grid on

