function canardInput = Controller_Lyapunov(x, cmd, P, I, D, kins, inds, AeroModel, dt)
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
% not sure if this function works for quaternion set up
qc = euler2quaternion(cmd);

dq = Qmult(q,Qinv(qc));

L = -P*sign(dq(4))*dq(1:3)-D*(1-dq(1:3)'*dq(1:3))*w;

%% Torque to Canard Actuations
% tranpose could be wrong here
lla = ecef2lla(x(inds.pos)',"wgs84");
alt = lla(3);
AtmosphericModel(alt);

d = kins.diameter;
r = kins.x_cp;
rho_inf = AtmosphericModel.rho_sl;
v_inf = norm(v);
S = kins.S;

% Not sure if this is right, check notebook
CL_delta = AeroModel.canard.CL_delta;
% CL = CL_delta*qc;

q_inf = 0.5*rho_inf*v_inf^2;

H = CL_delta*q_inf*S;

A = [d -d d -d;
     r 0 -r 0;
     0 -r 0 r];

B = [L(1)/H; L(2)/H; L(3)/H];

cmd = A\B;

canardInput.d1 = cmd(1);
canardInput.d2 = cmd(2);
canardInput.d3 = cmd(3);
canardInput.d4 = cmd(4);

%% Functions
    function quadprod = Qmult(p, q)
        q13 = q(1:3);
        p13 = p(1:3);
        q4 = q(4);
        p4 = p(4);

        quadprod1 = q4*p13 + p4*q13 - cross(p13, q13);
        quadprod2 = p4*q4 - dot(p13, q13);

        quadprod = [quadprod1; quadprod2];
    end

    function p = Qinv(q)
        q1 = q(1);
        q2 = q(2);
        q3 = q(3);
        q4 = q(4);

        q_mag = norm(q);
        q_star = [-q1; -q2; -q3; q4];

        p = q_star/(q_mag^2);
    end

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