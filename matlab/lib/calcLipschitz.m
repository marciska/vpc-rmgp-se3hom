%% calcLipschitz.m
% *Summary:* Plots figures regarding highest acceleration magnitudes
%
% -----------
%
% Editor:
%   OMAINSKA Marco - Doctoral Student, Cybernetics
%       <marcoomainska@g.ecc.u-tokyo.ac.jp>
% Property of: Fujita-Yamauchi Lab, The University of Tokyo, 2022
% Website: https://www.scl.ipc.i.u-tokyo.ac.jp

%------------- BEGIN CODE --------------
% set(groot,'defaultAxesTickLabelInterpreter','latex');
% set(groot,'defaultColorbarTickLabelInterpreter','latex');

% trajectory settings
center = [0 0 0];
scale = 1;


%% plot acceleration magnitude

% area to calculate in
xbnd = [-2.9+center(1) 2.9+center(1)]./scale;
ybnd = [-2.9+center(2) 2.9+center(2)]./scale;
step = 0.2;

% plot
figure('Name','Acceleration magnitudes','NumberTitle','off',...
    'Units','normalized','Position',[.1 .1 .8 .4]);
t = tiledlayout(1,3,'TileSpacing','Compact','Padding','Compact');
ax(1) = nexttile;
[c_min1,c_max1] = plotVF(ax(1),xbnd,ybnd,step,v=1,epsilon=0.5,center=center,scale=scale);
ax(2) = nexttile;
[c_min2,c_max2] = plotVF(ax(2),xbnd,ybnd,step,v=0.5,epsilon=1,center=center,scale=scale);
ax(3) = nexttile;
[c_min3,c_max3] = plotVF(ax(3),xbnd,ybnd,step,v=0.7,epsilon=0.3,center=center,scale=scale);
c_min = min([c_min1,c_min2,c_min3]);
c_max = max([c_max1,c_max2,c_max3]);
for i = 1:length(ax)
    clim(ax(i),[c_min c_max]);
end
cb = colorbar;
cb.Layout.Tile = 'east';
xlabel(ax(1),'')
xlabel(ax(3),'')
ylabel(ax(2),'')
ylabel(ax(3),'')


%% plot trajectories

tend = 20;
v_ = [1 0.5 0.7];
epsilon_ = [0.5 1 0.3];

% observer gain
Ke = 30*eye(6);

% focal length
lambda = 20;

% feature points
fp = [   0,  0,  0.5;
       0.5,  0,    0;
       0,    0, -0.5;
      -0.5,  0,    0];

% initial conditions
gco_init = mergepose(eye(3),[0 1 0]);
gwc_init = mergepose(eye(3),[0 -5 0]);
pwo_init = [-2 0 0];

for i = 1:length(ax)
    epsilon = epsilon_(i);
    v = v_(i);
    simout = sim('generateData');
    [~,pwo] = splitpose(simout.gwo.signals.values);
    plot(ax(i),pwo(:,1),pwo(:,2),'Color','#f4a261','LineWidth',5);
end


%% Local functions

function [c_min,c_max] = plotVF(ax,xbnd,ybnd,step,options)
    arguments
        ax
        xbnd
        ybnd
        step
        options.v double = 1
        options.epsilon double = 1
        options.center (1,3) double = [0 0 0]
        options.scale double = 1
    end
    % setup meshgrid
    xm = xbnd(1):step:xbnd(2);
    ym = ybnd(1):step:ybnd(2);
    [X,Y] = meshgrid(xm,ym);
    vel = zeros(size(X));

    % evaluate function over grid
    for i = 1:numel(X)
        p = [X(i) Y(i) 0];
        vel(i) = norm(vanderpol(p, options.v, options.epsilon, ...
            options.center, options.scale));
    end
    
    % calculate slopes
    [ddx,ddy] = gradient(vel,step);
    ddxy = sqrt(ddx.^2 + ddy.^2);
    c_min = min(ddxy(:));
    c_max = max(ddxy(:));
    
    % plot
%     contourf(ax,X,Y,ddxy);
    contourf(ax,X,Y,ddxy,'ShowText','on');
    hold(ax,'on')
    xlim(ax,xbnd);
    ylim(ax,ybnd);
    xlabel(ax,'x [m]', 'interpreter', 'latex')
    ylabel(ax,'y [m]', 'interpreter', 'latex')
    axis(ax,'equal')
%     cb = colorbar(ax);
%     cb.Layout.Tile = 'east';
    ax.FontSize = 25;
end

