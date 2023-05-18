function Vb = quartic(p, v, k, center, scale)
%%QUARTIC oscillator velocity field
%
% Editor:
%   OMAINSKA Marco - Doctoral Student, Cybernetics
%       <marcoomainska@g.ecc.u-tokyo.ac.jp>
% Property of: Fujita-Yamauchi Lab, The University of Tokyo, 2022
% Website: https://www.scl.ipc.i.u-tokyo.ac.jp

%------------- BEGIN CODE --------------
Vb = zeros(1,6);

% default parameters
if nargin < 5
    scale = 1;
end
if nargin < 4
    center = zeros(1,3);
end
if nargin < 3
    k = 1;
end
if nargin < 2
    v = 1;
end

% scale & center position around velocity field center
p = (p(:)-center(:)) ./ scale;

theta = atan2(p(2),p(1)) - pi/4;

% velocity field equations below
Vb(1) = v * p(2);
Vb(2) = v * k*(-p(1)^3 + p(1));
Vb(3) = v * cos(theta);
Vb(4) = 0; % TODO: fill-in
Vb(5) = 0; % TODO: fill-in
Vb(6) = 0; % TODO: fill-in

%------------- END CODE --------------
end
