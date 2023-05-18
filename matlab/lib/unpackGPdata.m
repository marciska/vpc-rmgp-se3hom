function [X,Y,kernel,hyp,sn] = unpackGPdata(filestr)
%%UNPACKGPDATA returns the GP parameters of given mat-file.
%
% -----------
%
% Inputs:
%   - filestr: file path string
%
% Outputs:
%   - X: GP input data [size Mx6]
%   - Y: GP output data [size Mx6]
%   - kernel: kernel function [function handler]
%   - hyp: GP kernel hyperparameters [size depends on kernel]
%   - sn: noise standard deviation [scalar]
%
% Example commands:
%   [X,Y,kernel,hyp,sn] = unpackGPdata('data/GP/VF_vdp_eps05v1_SE3_M30.mat')
%
%
% Editor:
%   OMAINSKA Marco - Doctoral Student, Cybernetics
%       <marcoomainska@g.ecc.u-tokyo.ac.jp>
% Property of: Fujita-Yamauchi Lab, The University of Tokyo, 2022
% Website: https://www.scl.ipc.i.u-tokyo.ac.jp

%------------- BEGIN CODE --------------

% load
S = load(filestr);

% unpack
X = S.X;
Y = S.Y;
try
    kernel = str2func(S.kernel);
catch ME
    kernel = S.kernel;
end
hyp = S.hyp;
sn = S.sn;

end

