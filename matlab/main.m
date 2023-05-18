%% main_matlab.m
% *Summary:* Runs a simulation within matlab / simulink.
%
% -----------
%
% Editor:
%   OMAINSKA Marco - Doctoral Student, Cybernetics
%       <marcoomainska@g.ecc.u-tokyo.ac.jp>
% Property of: Fujita-Yamauchi Lab, The University of Tokyo, 2022
% Website: https://www.scl.ipc.i.u-tokyo.ac.jp

% ------------- BEGIN CODE -------------


%% set simulation parameters

% load general simulation parameters
init;

% trajectory settings
center = [0 0 0];
scale = 1;
epsilon = 0.25; v = 1.5;

%% load data

% load GP models
M = 6;
% datatype = 'VF';
datatype = 'TJ';
% datatype = 'bestTJ';
% trajectory = 'vdp';
trajectory = 'quartic';
[X_GP_se,Y_GP_se,kernel_GP_se,hyp_GP_se,sn_GP_se] = unpackGPdata(['data/GP/' datatype '_' trajectory '_eps' erase(sprintf('%g',epsilon),'.') 'v' erase(sprintf('%g',v),'.') '_SE_M' sprintf('%g',M)]);
[X_GP_axang,Y_GP_axang,kernel_GP_axang,hyp_GP_axang,sn_GP_axang] = unpackGPdata(['data/GP/' datatype '_' trajectory '_eps' erase(sprintf('%g',epsilon),'.') 'v' erase(sprintf('%g',v),'.') '_SE3Axang_M' sprintf('%g',M)]);
[X_GP_hom,Y_GP_hom,kernel_GP_hom,hyp_GP_hom,sn_GP_hom] = unpackGPdata(['data/GP/' datatype '_' trajectory '_eps' erase(sprintf('%g',epsilon),'.') 'v' erase(sprintf('%g',v),'.') '_SE3Hom_M' sprintf('%g',M)]);


%% run simulation

% now run the simulation within the simulink file and stop it at any given
% time. Then proceed with the sections below.
% You also may want to save the simulation output with:
% save('data/unityResults_00.mat')

tend = 15;
simout = sim('vfs_matlab');

% unpack data
gwo = simout.gwo.signals.values;
gwc1 = simout.gwc1.signals.values;
gwc2 = simout.gwc2.signals.values;
gwc3 = simout.gwc3.signals.values;
gwc4 = simout.gwc4.signals.values;
gcobar1 = simout.gcobar1.signals.values;
gcobar2 = simout.gcobar2.signals.values;
gcobar3 = simout.gcobar3.signals.values;
gcobar4 = simout.gcobar4.signals.values;
tout = simout.tout;
[~, pwo] = splitpose(gwo);


%% animations

% plot animation
figure('Name','Animation','NumberTitle','off',...
    'Units','normalized','Position',[.05 0.05 .8 .8]);
ax = gca;
ax.FontSize = 35;
hold(ax,'on')
animate(ax,tout,...
      {gwo,agentdesign('target','show_trajectory','on','blocksize',0.7*ones(1,3))},...
      {gwc1,agentdesign('agent','color',[0 0 0],'blocksize',0.7*ones(1,3)),gcobar1,agentdesign('estimate','color',[0 0 0],'blocksize',0.7*ones(1,3))},...
      {gwc2,agentdesign('agent','color',hex2rgb('#fb8500'),'blocksize',0.7*ones(1,3)),gcobar2,agentdesign('estimate','color',hex2rgb('#fb8500'),'blocksize',0.7*ones(1,3))},...
      {gwc3,agentdesign('agent','color',hex2rgb('#0077b6'),'blocksize',0.7*ones(1,3)),gcobar3,agentdesign('estimate','color',hex2rgb('#0077b6'),'blocksize',0.7*ones(1,3))},...
      {gwc4,agentdesign('agent','color',hex2rgb('#e63946'),'blocksize',0.7*ones(1,3)),gcobar4,agentdesign('estimate','color',hex2rgb('#e63946'),'blocksize',0.7*ones(1,3))}...
      ...%'bound', 2*[-5 5; -5 5; -5 5]...
)


%% error plot
% set(groot,'defaultAxesTickLabelInterpreter','latex');

delta = 1e-3;
% delta = 1e-32;
% delta = 0.9999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999;
tau = 0.001;

% compute covering number
max_extension = [4 4 4];
covering_number = ceil( (1+sqrt(0.5*2)*2/tau)^2  * prod(1+sqrt(0.5)/tau*max_extension) );

% lipschitz constant of real function f
lip_f = [0.062;0.74;0.074;1.5;1.4;2.4];
kappa = min([eig(Kc)' eig(Ke)']) - norm(lip_f,2);

% compute kernel lipschitz constant
lip_kernel = (hyp_GP_hom(:,1)./hyp_GP_hom(:,2)).^2;

% compute gp lipschitz constants
lip_mu = zeros(6,1);
lip_cov = zeros(6,1);
gramKinv = zeros(M,M,6);
gramA = zeros(M,6);
for q=1:6
    L = chol(SE3Hom(X_GP_hom,X_GP_hom,hyp_GP_hom(q,:)) + sn_GP_hom(q)^2*eye(M),"lower"); % A = L*L'
    gramKinv(:,:,q) = (L\eye(M))'*(L\eye(M));
    gramA(:,q) = gramKinv(:,:,q)*Y_GP_hom(:,q);
end
for q = 1:6
    lip_mu(q) = lip_kernel(q)*sqrt(M)*norm(gramA(:,q),2);
    lip_cov(q) = 2*tau*lip_kernel(q)*(1+M*norm(gramKinv(:,:,q),2)*(hyp_GP_hom(q,1)^2));
end

% compute beta and gamma
beta = sqrt(2*log(covering_number/delta));
gamma = zeros(6,1);
for i = 1:6
    gamma(i) = (lip_mu(i)+lip_f(i))*tau + beta*sqrt(lip_cov(i)*tau);
end
gamma = norm(gamma,2);
disp(['beta  = ' num2str(beta,'%.2f')])
disp(['gamma = ' num2str(gamma,'%.2f')])


% simulate model error bound
modelerror = zeros(1,length(tout));
b = zeros(1,length(tout));
gwobar4 = pagemtimes(gwc4,gcobar4);
cov_i = zeros(1,length(tout));
for i = 1:length(tout)
    [~,cov] = gp_calc(gwobar4(:,:,i),X_GP_hom,Y_GP_hom,@SE3Hom,hyp_GP_hom,sn_GP_hom,gramKinv,gramA);
    cov_i(i) = norm(cov,2);
    modelerror(i) = beta*sqrt(sum(cov)) + gamma;
    b(i) = modelerror(i)/kappa;
end
[~,cov_max] = gp_calc(mergepose(roty(180),[100 100 100]),X_GP_hom,Y_GP_hom,@SE3Hom,hyp_GP_hom,sn_GP_hom,gramKinv,gramA);
bmax = (beta*sqrt(sum(cov_max)) + gamma)/kappa;
disp(['bmin = ' num2str(min(b),'%.4f')])
disp(['(1-\delta)^6 = ' num2str((1-delta)^6,'%.4f')])


% unpack some data
e1 = simout.e1.signals.values;
e2 = simout.e2.signals.values;
e3 = simout.e3.signals.values;
e4 = simout.e4.signals.values;
tout = simout.tout;

% obtain RMSE
RMSE1 = rmse(e1);
RMSE2 = rmse(e2);
RMSE3 = rmse(e3);
RMSE4 = rmse(e4);
disp(['RMSE (no GP)                  : ' num2str(RMSE1)])
disp(['RMSE (RBF-Kernel)             : ' num2str(RMSE2)])
disp(['RMSE (SE3-Kernel Axis-Angle)  : ' num2str(RMSE3)])
disp(['RMSE (SE3-Kernel Homogeneous) : ' num2str(RMSE4)])

y = [RMSE1 RMSE2 RMSE3 RMSE4];

fig = figure('Name','Error RMSE','NumberTitle','off',...
    'Units','normalized','Position',[0.1 .2 .8 .4]);
tiledlayout(1,1);
ax = nexttile;
rmse_bar = bar(ax,y,'EdgeColor','none');
grid(ax,'on');
hold(ax,'on');
xlim(ax,[0.2 4.8]);
xticklabels(ax,{...
    '\textrm{No GP}',...
    '$$\begin{array}{c}\textrm{Squared Exponential}\\\textrm{Kernel}\end{array}$$',...
    '$$\begin{array}{c}\textrm{Rigid Body Kernel}\\\textrm{(Axis-Angle)}\end{array}$$',...
    '$$\begin{array}{c}\textrm{Rigid Body Kernel}\\\textrm{(Homogeneous)}\end{array}$$'...
    });
rmse_bar.FaceColor = 'flat';
rmse_bar.CData(1,:) = hex2rgb('457B9D');
rmse_bar.CData(2,:) = hex2rgb('457B9D');
rmse_bar.CData(3,:) = hex2rgb('457B9D');
rmse_bar.CData(4,:) = hex2rgb('E63946');
ylabel(ax,'Time [ms]')
ax.FontSize = 30;


fig = figure('Name','Error','NumberTitle','off',...
    'Units','normalized','Position',[0.1 .2 .8 .6]);
t = tiledlayout(1,1,'TileSpacing','Compact','Padding','Compact');

% error plot
lw = 8;
ax = nexttile;
hold(ax,'on')
grid(ax,'on')
p1=plot(tout,vecnorm(e1,2,2),'LineWidth',1*lw,'color',[0 0 0]);
p2=plot(tout,vecnorm(e2,2,2),'LineWidth',1*lw,'color','#fb8500');
p3=plot(tout,vecnorm(e3,2,2),'LineWidth',2*lw,'color','#0077b6');
p4=plot(tout,vecnorm(e4,2,2),'LineWidth',2*lw,'color','#e63946');
plot(tout(1:1:end),b(1:1:end),':','LineWidth',2*lw,'MarkerSize',10,'color',0.9.*hex2rgb('#e63946'));
% plot(tout,bmax,'--','LineWidth',0.5*lw,'color','#e63946');
xlim(ax,[0 tout(end)])
ylim(ax,[0 0.8])
% ylim(ax,[0 1.2])
xlabel(ax,'Time $t$ [s]','interpreter', 'latex')
ylabel(ax,'$\|${\boldmath${e}$}$\|$','interpreter', 'latex')
% lg=legend(ax,[p1 p2 p3 p4],...
%     sprintf('\\textrm{no GP} (MSE: %.3f)',RMSE1),...
%     sprintf('\\textrm{Squared Exponential} (MSE: %.3f)',RMSE2),...
%     sprintf('\\textrm{Rigid Body (Axis-Angle)} (MSE: %.3f)',RMSE3),...
%     sprintf('\\textrm{Rigid Body (Homogeneous)} (MSE: %.3f)',RMSE4),...
%     'interpreter', 'latex', ...
%     'NumColumns',2);
ax.FontSize = 40;


% save plots
% exportgraphics(t,'error.eps','BackgroundColor','none','ContentType','vector')
