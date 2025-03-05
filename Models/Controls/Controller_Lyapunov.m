function [canardInput, L] = Controller_Lyapunov(x, cmd, P, D, kins, inds, AeroModel, dt)
% Controller_Lyapunov - Lyapunov attitude controller
% Roll, Pitch, and Yaw controller using a P and D gains
% Inputs:
%   x - Current State
%   cmd - [rad] Commanded Roll, Pitch, and Yaw
%   P - Proportional gain parameter
%   I - Integral Gain Parameter
%   D - Derivative Gain Parameter
% Outputs:
%   Torques - Three Canard Torques to achieve commanded attitude command

%% States
q = [x(inds.qx); x(inds.qy); x(inds.qz); x(inds.qw)];

v = x(inds.vel);

w = x(inds.w_ib);


%% Lyapunov
qc = euler2quaternion(cmd);

qc_inv = [-qc(1); -qc(2); -qc(3); qc(4)];

dq = [qc_inv(4) * q(1:3) + q(4) * qc_inv(1:3) - cross(q(1:3), qc_inv(1:3));
    q(4)*qc_inv(4) - dot(q(1:3), qc_inv(1:3));
];

% L = -P * sign(dq(4))*dq(1:3)-D*x(inds.w_ib);

% dq_2 = Qmult(q,Qinv(qc))

L = -P*sign(dq(4))*dq(1:3)-D*(1+dq(1:3)'*dq(1:3))*w;
% L_2 = -P*sign(dq(4))*dq(1:3)-D*(1-dq(1:3)'*dq(1:3))*w

T_x = L(1);
T_y = L(2);
T_z = L(3);

%% Torque to Canard Actuations
    lla = ecef2lla(x(inds.pos),"wgs84");
    alt = lla(3);
    AtmosphericModel(alt);
    rho_inf = AtmosphericModel.rho_sl;
    v_inf = norm(x(inds.vel, 2));

    q_inf = 0.5 * rho_inf * v_inf^2;

    % A = [d -d  d -d; 
    %      r  0 -r  0;
    %      0 -r  0  r];
    % 
    % b = [T_x/H; T_y/H; T_z/H];

    % A = [kins.canard.z_cp_13, kins.canard.y_cp_24, kins.canard.z_cp_13, kins.canard.y_cp_24;
    %  kins.canard.x_cp,   -kins.canard.x_cp,    0,  0;
    %  0,  0,   kins.canard.x_cp,  -kins.canard.x_cp];

    C_p = (kins.diameter/2) + (kins.canard.height/2);

    A = [
        C_p -C_p C_p -C_p;
        0 kins.x_cp 0 -kins.x_cp;
        -kins.x_cp 0 kins.x_cp 0;
    ];

    % Compute b vector
    % b = [T_x; T_y; T_z];
    b = (1 / (q_inf * kins.canard.S * AeroModel.canard.CL_delta)) * ...
        [T_x; 
         T_y; 
         T_z];

    % cmd = b / A';
    cmd = pinv(A) * b;

    % canardInput.d1 = T_x;
    % canardInput.d2 = T_x;
    % canardInput.d3 = T_x;
    % canardInput.d4 = T_x;

    canardInput.d1 = cmd(1);
    canardInput.d2 = cmd(2);
    canardInput.d3 = cmd(3);
    canardInput.d4 = cmd(4);

%% Function
    function quat = euler2quaternion(eul)
        phi = eul(1)/2; %roll
        theta = eul(2)/2; %pitch
        psi = eul(3)/2; %yaw

        quat = [sin(phi)*cos(theta)*cos(psi)+cos(phi)*sin(theta)*sin(psi);
                cos(phi)*sin(theta)*cos(psi)-sin(phi)*cos(theta)*sin(psi);
                cos(phi)*cos(theta)*sin(psi)+sin(phi)*sin(theta)*cos(psi);
                cos(phi)*cos(theta)*cos(psi)-sin(phi)*sin(theta)*sin(psi)];
    end 


end