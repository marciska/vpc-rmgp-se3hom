function Vb = vanderpol(p, v, epsilon, center, scale)
%%VANDERPOL velocity field
%
% Editor:
%   OMAINSKA Marco - Doctoral Student, Cybernetics
%       <marcoomainska@g.ecc.u-tokyo.ac.jp>
% Property of: Fujita-Yamauchi Lab, The University of Tokyo, 2022
% Website: https://www.scl.ipc.i.u-tokyo.ac.jp

%------------- BEGIN CODE --------------

% default parameters
if nargin < 5
    scale = 1;
end
if nargin < 4
    center = zeros(1,length(p));
end
if nargin < 3
    epsilon = 1;
end
if nargin < 2
    v = 1;
end

% process I/O
Vb = zeros(1,6);

% scale & center position around velocity field center
p = (p(:)-center(:)) ./ scale;

% velocity field equations below
dx = v * p(2);
dy = -v * p(1) + v * epsilon * (1 - p(1)^2) * p(2);
ddx = v*dy;
ddy = -v*dx + v*epsilon*(1-p(1)^2)*dy - 2*v*epsilon*p(1)*dx*p(2);
if dx^2+dy^2 < 10*eps
    wz = 0; %nan
else
    wz = (dx*ddy-ddx*dy)/(dx^2+dy^2);
end

% set output
Vb(1) = dx;
Vb(2) = dy;
Vb(6) = wz;

%------------- END CODE --------------
end
