function dpwo = lorenz(p, v, sigma, rho, beta, center, scale)
%%LORENZ attractor velocity field
%
% Editor:
%   OMAINSKA Marco - Doctoral Student, Cybernetics
%       <marcoomainska@g.ecc.u-tokyo.ac.jp>
% Property of: Fujita-Yamauchi Lab, The University of Tokyo, 2022
% Website: https://www.scl.ipc.i.u-tokyo.ac.jp

%------------- BEGIN CODE --------------
dpwo = zeros(1,3);

% default parameters
if nargin < 7
    scale = 1;
end
if nargin < 6
    center = zeros(1,3);
end
if nargin < 5
    beta = 1;
end
if nargin < 4
    rho = 1;
end
if nargin < 3
    sigma = 1;
end
if nargin < 2
    v = 1;
end

% scale & center position around velocity field center
p = (p(:)-center(:)) ./ scale;

% velocity field equations below
dpwo(1) = v * sigma* (p(3) - p(1));
dpwo(2) = v * (p(1) * (rho - p(2)) - p(3));
dpwo(3) = v * (p(1) * p(3) - beta * p(2));

%------------- END CODE --------------
end
