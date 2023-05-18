function hex = rgb2hex(rgb)
%%RGB2HEX converts a rgb-color to corresponding hex string.
%
% Detailed Explanation:
%   none
%
% -----------
%
% Inputs:
%   - rgb: color vector [size 3]
%
% Outputs:
%   - hex: hexcode string
%
% Example commands:
%   none
%
%
% Editor:
%   OMAINSKA Marco - Doctoral Student, Cybernetics
%       <marcoomainska@g.ecc.u-tokyo.ac.jp>
% Property of: Fujita-Yamauchi Lab, The University of Tokyo, 2022
% Website: https://www.scl.ipc.i.u-tokyo.ac.jp

%------------- BEGIN CODE --------------

% check input 
assert(isnumeric(rgb)==1,'not a valid rgb value') 
nc = length(rgb);
assert(max(rgb(:)) <= 255 & min(rgb(:)) >= 0,'rgb values not properly scaled')
if max(rgb(:)) <= 1
    rgb = round(rgb * 255); 
else
    rgb = round(rgb); 
end

% conversion below
hex(:,2:(nc*2+1)) = reshape(sprintf('%02X',rgb.'),nc*2,[]).';
hex(:,1) = '#';

%-------------- END CODE ---------------
end