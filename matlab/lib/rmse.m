function err = rmse(data)
%%RMSE calculates root mean square error.
%
% -----------
%
% Inputs:
%   - data: matrix [size MxN]
%
% Outputs:
%   - err: root mean square error [scalar]
%
% Example commands:
%   err = rmse(rand(100,2))
%
%
% Editor:
%   OMAINSKA Marco - Doctoral Student, Cybernetics
%       <marcoomainska@g.ecc.u-tokyo.ac.jp>
% Property of: Fujita-Yamauchi Lab, The University of Tokyo, 2022
% Website: https://www.scl.ipc.i.u-tokyo.ac.jp

%------------- BEGIN CODE --------------

err = sqrt(mean(vecnorm(data,2,2)));

end

