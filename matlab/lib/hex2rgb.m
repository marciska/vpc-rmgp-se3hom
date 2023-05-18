function rgb = hex2rgb(hex)
%%HEX2RGB converts an hexcode-string to corresponding rgb color.
%
% Detailed Explanation:
%   none
%
% -----------
%
% Inputs:
%   - hex: hexcode string
%
% Outputs:
%   - rgb: color vector [size 3]
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
if isstring(hex); hex = char(hex); end
if ~ischar(hex); error('did not provide a string hex code'); end
hex = strip(hex,'left','#');
if isempty(regexpi(hex,'^[0-9A-F]{6}$','once')); error('invalid hex code.'); end

rgb = [double(hex2dec(hex(1:2))),...
       double(hex2dec(hex(3:4))),...
       double(hex2dec(hex(5:6)))]./255;

%-------------- END CODE ---------------
end