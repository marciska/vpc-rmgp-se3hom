function dpwo = duffing_mod(p, t, v, alpha, beta, gamma, delta, omega, center, scale)
%%DUFFING_MOD modified Duffing oscillator velocity field
%
% Editor:
%   OMAINSKA Marco - Doctoral Student, Cybernetics
%       <marcoomainska@g.ecc.u-tokyo.ac.jp>
% Property of: Fujita-Yamauchi Lab, The University of Tokyo, 2022
% Website: https://www.scl.ipc.i.u-tokyo.ac.jp

%------------- BEGIN CODE --------------
dpwo = zeros(1,3);

% default parameters
if nargin < 10
    scale = 1;
end
if nargin < 9
    center = zeros(1,3);
end
if nargin < 8
    omega = 1;
end
if nargin < 7
    delta = 1;
end
if nargin < 6
    gamma = 1;
end
if nargin < 5
    beta = 1;
end
if nargin < 4
    alpha = 1;
end
if nargin < 3
    v = 1;
end

% scale & center position around velocity field center
p = (p(:)-center(:)) ./ scale;

% velocity field equations below
dpwo(1) = v * p(2);
dpwo(2) = v * (-delta*p(2) -alpha*p(1) -beta*(p(1)^3) +gamma*cos(omega*t));

%------------- END CODE --------------
end
