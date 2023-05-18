%% kernel_computation_times.m
% *Summary:* Plot-script to compare kernel computation times
%
% -----------
%
% Editor:
%   OMAINSKA Marco - Doctoral Student, Cybernetics
%       <marcoomainska@g.ecc.u-tokyo.ac.jp>
% Review:
%   YAMAUCHI Junya - Assistant Professor
%       <junya_yamauchi@ipc.i.u-tokyo.ac.jp>
%
% Property of: Fujita-Yamauchi Lab, University of Tokyo, 2023
% Website: https://www.scl.ipc.i.u-tokyo.ac.jp

%------------- BEGIN CODE --------------

rng(0);
randiv = @(a,b,s) a + (b-a).*rand(s);


%% generate some pose data
ntraining = 1000;
ntest = 100;
p = randiv(-2,2,[ntest+ntraining,3]);
q = randrot(ntest+ntraining,1);
R = quat2rotm(q);
xitheta = quat2axang(q);

% create overall data
ghom = zeros(4,4,ntest+ntraining);
gvec = zeros(ntest+ntraining,6);
ghomvec = zeros(ntest+ntraining,12);
for i = 1:ntest+ntraining
    ghom(:,:,i) = mergepose(R(:,:,i),p(i,:));
    gvec(i,:) = [p(i,:) xitheta(i,1:3).*xitheta(i,4)];
    R_ = R(:,:,i);
    ghomvec(i,:) = [p(i,:) R_(:)'];
end

% create training data
% idx = sort(randperm(ntest,ntraining));
idx = 1:ntraining;
ghomdata = ghom(:,:,idx);
gvecdata = gvec(idx,:);
ghomvecdata = ghomvec(idx,:);
Vdata = ones(ntraining,6);

% create test data
idx = (1:ntest)+ntraining;
ghomtest = ghom(:,:,idx);
gvectest = gvec(idx,:);
ghomvectest = ghomvec(idx,:);


%% computation measurement

trials = 100;
dtime_se_gvec = zeros(trials,2);
dtime_se3_gvec = zeros(trials,2);
dtime_se3_ghom = zeros(trials,2);
for trial=1:trials
    fprintf("Trial %i of %i\n",trial,trials);

    % se-kernel @ gvec
    [dt_kernel, dt_pred] = timed_SE(gvecdata,Vdata,gvectest);
    dtime_se_gvec(trial,1) = dt_kernel;
    dtime_se_gvec(trial,2) = dt_pred;

    % se3-kernel @ gvec
    [dt_kernel, dt_pred] = timed_SE3Axang(gvecdata(:,1:3),gvecdata(:,4:6),Vdata,gvectest(:,1:3),gvectest(:,4:6));
    dtime_se3_gvec(trial,1) = dt_kernel;
    dtime_se3_gvec(trial,2) = dt_pred;

    % se3-kernel @ ghom
    [dt_kernel, dt_pred] = timed_SE3Hom(ghomvecdata(:,1:3),ghomvecdata(:,4:12),Vdata,ghomvectest(:,1:3),ghomvectest(:,4:12));
    dtime_se3_ghom(trial,1) = dt_kernel;
    dtime_se3_ghom(trial,2) = dt_pred;
end
dtime_stdv_se_gvec = std(sum(dtime_se_gvec,2));
dtime_stdv_se3_gvec = std(sum(dtime_se3_gvec,2));
dtime_stdv_se3_ghom = std(sum(dtime_se3_ghom,2));
dtime_se_gvec = mean(dtime_se_gvec,1);
dtime_se3_gvec = mean(dtime_se3_gvec,1);
dtime_se3_ghom = mean(dtime_se3_ghom,1);

fprintf("Kernel Computation times:\n")
fprintf("   se  @ gvec:  %.2f ms\n", sum(dtime_se_gvec)*1000)
fprintf("   se3 @ gvec:  %.2f ms\n", sum(dtime_se3_gvec)*1000)
fprintf("   se3 @ ghom:  %.2f ms\n", sum(dtime_se3_ghom)*1000)
fprintf("Performance-Gain: %.2f%%\n", (1-sum(dtime_se3_ghom)/sum(dtime_se3_gvec))*100)


%% plot

y = [dtime_se_gvec; dtime_se3_gvec; dtime_se3_ghom]*1000;
y_stdv = [dtime_stdv_se_gvec dtime_stdv_se3_gvec dtime_stdv_se3_ghom]*1000;

fig = figure('Name','Kernel Computation Comparison','NumberTitle','off',...
    'Units','normalized','Position',[0.1 .2 .5 .3]);
tiledlayout(1,1);
ax = nexttile;
b = bar(ax,y,'stacked','EdgeColor','none','FaceColor','flat');
grid(ax,'on');
hold(ax,'on');
xlim(ax,[0.2 3.8]);
xticklabels(ax,{...
    '$$\begin{array}{c}\textrm{Squared Exponential}\\\textrm{Kernel}\end{array}$$',...
    '$$\begin{array}{c}\textrm{Rigid Body Kernel}\\\textrm{(Axis-Angle)}\end{array}$$',...
    '$$\begin{array}{c}\textrm{Rigid Body Kernel}\\\textrm{(Homogeneous)}\end{array}$$'...
    });
% b.FaceColor = 'flat';
b(1).CData(1,:) = hex2rgb('457B9D');
b(2).CData(1,:) = hex2rgb('00B4D8');
b(1).CData(2,:) = hex2rgb('457B9D');
b(2).CData(2,:) = hex2rgb('00B4D8');
b(1).CData(3,:) = hex2rgb('E63946');
b(2).CData(3,:) = hex2rgb('FF758F');
errorbar(ax,sum(y,2),y_stdv,'LineStyle','none','LineWidth',5,'CapSize',20,'Color',hex2rgb('023047'))
ylabel(ax,'Time [ms]')
ax.FontSize = 30;


%% Functions

function [dt_kernel, dt_pred] = timed_SE(gdata,Vdata,gtest)
    sigmafsq = 1^2;
    lsq = 1^2;
    sn = 0.1;
    [m, ~] = size(gdata);
    kernel = coder.nullcopy(zeros(m,m));
    
    % start timer
    tic;

    % compute kernel
    for i = 1:m
        for j = i:m
            if i == j
                kernel(i,j) = sigmafsq;
            end
    
            % compute distance
            d = gdata(i,:) - gdata(j,:);
            distsq = dot(d,d);
            
            % compute kernel
            kernel(i,j) = sigmafsq * exp(-0.5*distsq/lsq);
            kernel(j,i) = kernel(i,j);
        end
    end

    % stop timer
    dt_kernel = toc;

    % start timer
    tic;

    % compute output
    N = size(gtest,1);
    mu = coder.nullcopy(zeros(N,6));
    for q = 1:6
        K = kernel + sn^2*eye(m);
        A = K\Vdata(:,q);

        for i=1:N
            Kst = coder.nullcopy(zeros(1,m));
            for j=1:m
                d = gtest(i,:) - gdata(j,:);
                distsq = dot(d,d);
                kernel(1,j) = sigmafsq * exp(-0.5*distsq/lsq);
            end
            mu(i,q) = Kst*A;
        end
    end
    
    % stop timer
    dt_pred = toc;
end

function [dt_kernel, dt_pred] = timed_SE3Axang(pdata,xithetadata,Vdata,ptest,xithetatest)
    sigmafsq = 1^2;
    lsq = 1^2;
    rho1 = 0.5;
    rho2 = 0.5;
    sn = 0.1;
    [m, ~] = size(pdata);
    kernel = coder.nullcopy(zeros(m,m));
    
    % start timer
    tic;

    % compute kernel
    for i = 1:m
        for j = i:m
            if i == j
                kernel(i,j) = sigmafsq;
            end

            % split pose
            p1 = pdata(i,:);
            p2 = pdata(j,:);
            xitheta1 = xithetadata(i,:);
            xitheta2 = xithetadata(j,:);

            % compute distance
            dp = p1-p2;
            distsq = rho1*dot(dp,dp) + rho2*darc(xitheta1,xitheta2)^2;
            
            % compute kernel
            kernel(i,j) = sigmafsq * exp(-0.5*distsq/lsq);
            kernel(j,i) = kernel(i,j);
        end
    end

    % stop timer
    dt_kernel = toc;

    % start timer
    tic;

    % compute output
    N = size(ptest,1);
    mu = coder.nullcopy(zeros(N,6));
    for q = 1:6
        K = kernel + sn^2*eye(m);
        A = K\Vdata(:,q);

        for i=1:N
            p1 = ptest(i,:);
            xitheta1 = xithetatest(i,:);
            Kst = coder.nullcopy(zeros(1,m));
            for j=1:m
                p2 = pdata(j,:);
                xitheta2 = xithetadata(j,:);
                dp = p1-p2;
                distsq = rho1*dot(dp,dp) + rho2*darc(xitheta1,xitheta2)^2;
                kernel(1,j) = sigmafsq * exp(-0.5*distsq/lsq);
            end
            mu(i,q) = Kst*A;
        end
    end
    
    % stop timer
    dt_pred = toc;
end

function d = darc(xitheta1,xitheta2)
%%DARC distance function metric for axis-angle rotations 
%
% Syntax:   D = DARC(XITHETA1,XITHETA2)
%
% Inputs:
%   - XITHETA1: axis-angle rotation [size 3]
%   - XITHETA2: axis-angle rotation [size 3]
%
% Outputs:
%   - D: distance [scalar]

%------------- BEGIN CODE --------------

% pre-process inputs
[xi1, theta1] = splitAxisAngle(xitheta1);
[xi2, theta2] = splitAxisAngle(xitheta2);

% calculate distance
% Note: Sometimes x is larger than 1, which is a problem for acos().
% This error comes just from numerics, so we saturate x as: -1 <= x <= 1
x = abs( cos(theta1/2)*cos(theta2/2) ...
        + sin(theta1/2)*sin(theta2/2)*dot(xi1,xi2) );
x = max(-1, min(x,1));
d = 2*acos(x);

%-------------- END CODE ---------------
end

function [xi, theta] = splitAxisAngle(xitheta)
%%SPLITAXISANGLE splits the axis-angle composed vector in axis and angle
%
% Syntax:   [XI,THETA] = SPLITAXISANGLE(XITHETA)
%
% Inputs:
%   - XITHETA: axis-angle rotation [size 3]
%
% Outputs:
%   - XI: rotation axis [size 3]
%   - THETA: rotation angle [scalar]

%------------- BEGIN CODE --------------

theta = norm(xitheta,2);
if theta == 0 % corner case
    xi = [0 0 1]; % default axis
else
    xi = xitheta./theta;
end

%-------------- END CODE ---------------
end


function [dt_kernel, dt_pred] = timed_SE3Hom(pdata,Rdata,Vdata,ptest,Rtest)
    sigmafsq = 1^2;
    lsq = 1^2;
    rho1 = 0.5;
    rho2 = 0.5;
    sn = 0.1;
    [m,  ~] = size(pdata);
    kernel = coder.nullcopy(zeros(m,m));
    
    % start timer
    tic;

    % compute kernel
    for i = 1:m
        for j = i:m
            if i == j
                kernel(i,j) = sigmafsq;
            end

            % split pose
            p1 = pdata(i,:);
            p2 = pdata(j,:);
            R1 = Rdata(i,:);
            R2 = Rdata(j,:);

            % compute distance
            dp = p1-p2;
            distsq = rho1*dot(dp,dp) + rho2*(3 - dot(R1,R2))/2;
            
            % compute kernel
            kernel(i,j) = sigmafsq * exp(-0.5*distsq/lsq);
            kernel(j,i) = kernel(i,j);
        end
    end

    % stop timer
    dt_kernel = toc;

    % start timer
    tic;

    % compute output
    N = size(ptest,1);
    mu = coder.nullcopy(zeros(N,6));
    for q = 1:6
        K = kernel + sn^2*eye(m);
        A = K\Vdata(:,q);

        for i=1:N
            p1 = ptest(i,:);
            R1 = Rtest(i,:);
            Kst = coder.nullcopy(zeros(1,m));
            for j=1:m
                p2 = pdata(j,:);
                R2 = Rdata(j,:);
                dp = p1-p2;
                distsq = rho1*dot(dp,dp) + rho2*(3 - dot(R1,R2))/2;
                kernel(1,j) = sigmafsq * exp(-0.5*distsq/lsq);
            end
            mu(i,q) = Kst*A;
        end
    end
    
    % stop timer
    dt_pred = toc;
end
