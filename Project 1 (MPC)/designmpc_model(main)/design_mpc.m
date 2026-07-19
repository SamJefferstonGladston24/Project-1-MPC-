% Vehicle parameters
m  = 1500;     % mass (kg)
Iz = 3000;     % yaw inertia (kg*m^2)

lf = 1.2;      % front axle distance (m)
lr = 1.6;      % rear axle distance (m)

Cf = 19000;    % front cornering stiffness (N/rad)
Cr = 33000;    % rear cornering stiffness (N/rad)

Vx = 15;       % constant forward speed (m/s)

A = [ 0, 1, 0, 0;
      0, 0, 1, Vx;
      0, 0, -(Cf+Cr)/(m*Vx), -(Cf*lf-Cr*lr)/(m*Vx) - Vx;
      0, 0, -(Cf*lf-Cr*lr)/(Iz*Vx), -(Cf*lf^2+Cr*lr^2)/(Iz*Vx) ];

B = [0;
     0;
     Cf/m;
     Cf*lf/Iz];

C = eye(4);        % output all states
D = zeros(4,1);    % no direct input-output effect

sys = ss(A,B,C,D);

t = 0:0.01:5;
u = 0.05 * ones(size(t));   % constant steering input

lsim(sys, u, t)

mpcDesigner(sys)

Ts = 0.05;   % sample time (seconds)

sys_d = c2d(sys, Ts);

Ad = sys_d.A;
Bd = sys_d.B;
Cd = sys_d.C;
Dd = sys_d.D;

t = 0:Ts:5;
u = 0.05 * ones(size(t));

lsim(sys_d, u, t)

mpcobj = mpc(sys_d, Ts);

% Discretization
Ts = 0.05;
sys_d = c2d(sys, Ts);

% Check matrices
Ad = sys_d.A;
Bd = sys_d.B;

% Test
step(sys_d);

% MPC
mpcDesigner(sys_d);

mpcobj = mpc(sys_d, Ts);

C = eye(4);

mpcobj.Model.Nominal.Y = [0; 0; 0; 0];

mpcobj.Weights.OutputVariables = [10 5 0 0];
mpcobj.Weights.ManipulatedVariables = 0.1;
mpcobj.Weights.ManipulatedVariablesRate = 1;

mpcobj.MV.Min = -0.5;   % radians (~ -28 deg)
mpcobj.MV.Max = 0.5;

mpcobj.MV.RateMin = -0.1;
mpcobj.MV.RateMax = 0.1;

mpcobj.PredictionHorizon = 20;
mpcobj.ControlHorizon = 5;

mpcDesigner(sys_d)

T = 50;
r = [0 0 0 0];   % reference (stay at zero error)

sim(mpcobj, T, r)

% Create MPC
mpcobj = mpc(sys_d, Ts);

% Weights
mpcobj.Weights.OutputVariables = [10 5 0 0];
mpcobj.Weights.ManipulatedVariables = 0.1;
mpcobj.Weights.ManipulatedVariablesRate = 1;

% Constraints
mpcobj.MV.Min = -0.5;
mpcobj.MV.Max = 0.5;

% Horizons
mpcobj.PredictionHorizon = 20;
mpcobj.ControlHorizon = 5;

% Launch GUI
mpcDesigner(sys_d);