clear
close all
clc 

work_path = '...';
cd(work_path);


%% Basic parameter settings 

rng(0);

% sampling time
tau = 0.01; 
parameters.tau = tau;

% Reaction time
r = 0.01;
parameters.r = r;

% ALL sampling time
T_array = 20/0.01;
parameters.T_array = T_array;

% Vehicle length
length = 2;
parameters.length = length;

% Desired distance
Dis = 10;
parameters.Dis = Dis;

% Projection time
P_array = 0.05; 
prediction_step_num = P_array/tau;
parameters.prediction_step_num = prediction_step_num;

% The number vehicle in platoon
N = 5;
parameters.N = N;

% The accleration of leading vehicle
Ul = zeros(2005, 1);
Ul(800:1000) = linspace(0, 1, 201);
Ul(1001:1200) = 1;
Ul(1201:1400) = linspace(1, 0, 200);
parameters.Ul = Ul;

%Random perturbations obey Gaussian distribution
U_zeta_all = normrnd(0.001,0.03,2005,1); 
%U_zeta_all = -0.15 + (0.15-(-0.15))*rand(2005,1);  uniform distribution
parameters.U_zeta = U_zeta_all;

Omege1=2.44;
Omege2=2.44;
parameters.Omege1 = Omege1;
parameters.Omege2 = Omege2;

std_U_zeta = std(U_zeta_all);  
parameters.std_U_zeta = std_U_zeta;

% Constructive coefficient matrix
A = [1 tau; 0 1];
parameters.A = A;

B = [-(tau^2/2+r*tau);-tau];
parameters.B = B;

C = [tau^2/2; tau];
parameters.C = C;

% initial solution
x0= [0; 0];
parameters.x0 = x0;

% Weighting coefficients
Q =0.0001.*eye(2); 
parameters.Q = Q;

R =1;
parameters.R = R;

Q_x=[]; R_u=[];

for i=1:prediction_step_num      
        Q_x = [Q_x zeros(size(Q_x,1),size(Q,1));
            zeros(size(Q,1),size(Q_x,1)) Q];
        R_u = [R_u zeros(size(R_u,1),size(R,1));
            zeros(size(R,1),size(R_u,1)) R];
end

parameters.Q_x = Q_x;

parameters.R_u = R_u;

I1 = eye(prediction_step_num, prediction_step_num);
parameters.I1 = I1;

I2 = eye(prediction_step_num, prediction_step_num);
parameters.I2 = I2;

% initial solution
init_X = zeros(prediction_step_num,1);
parameters.init_X = init_X;


%% Optimization
case_num = T_array;  
veh_num = 5; 

opt_Solutions = cell(case_num, veh_num); 
opt_Values = zeros(case_num, veh_num); 
opt_acc = zeros(case_num,veh_num);

x = zeros(2,veh_num);
xmeasure = x0;
parameters.xmeasure =xmeasure;

T = zeros(case_num,veh_num);
vehall_K = zeros(case_num+prediction_step_num,2);


for j = 1 : veh_num
    
    % ---------- 更新反馈增益K，对应的M_phy,M_B,M_C也会发生变化----------
  
    U_found = Ul(:,j);
    parameters.U_found = U_found;
    K = Feekback_K_solution(parameters);
  
   
for k = 1 : case_num
    
    U_refer = Ul(k:k+prediction_step_num-1,j);
    parameters.U_refer = U_refer;
    
    U_zeta = U_zeta_all(k:k+prediction_step_num-1,1);
    parameters.U_zeta =  U_zeta;
    
    % M_phy,M_B,M_C,  K_bar
    K_feekback = K(k,:);
    
    phy = A + B*K_feekback; 

    M_phy=[]; M_B=[]; M_C = [];   K_bar = []; temp1_N=[];temp2_N=[]; 
    
    t_start = tic;

for i=1:prediction_step_num
    
        M_phy = [M_phy; phy^i];
        
        M_B = [M_B zeros(size(M_B,1), size(B,2));
                 phy^(i-1)*B temp1_N];
             
        temp1_N = [phy^(i-1)*B temp1_N];
        
        M_C = [M_C zeros(size(M_C,1), size(C,2));
                 phy^(i-1)*C temp2_N];
             
        temp2_N = [phy^(i-1)*C temp2_N];
        
        K_bar = [K_bar zeros(size(K_bar,1),size(K_feekback,2));
            zeros(size(K_feekback,1),size(K_bar,2)) K_feekback];
end
    parameters.M_phy = M_phy;
    parameters.M_B = M_B;
    parameters.M_C = M_C;
    parameters.K_bar = K_bar;
      
    % ---------- Solve model ----------
    tic;
    [opt_X, opt_f,fval_out,firstorderopt_out] = Work4_OptEngine(parameters);
    T(k,j) = toc;
    
    a(k,j) = K_feekback*xmeasure + opt_X(1);

    % Store the results obtained 
    opt_Solutions{k,j} = opt_X;
    opt_Values(k,j) = opt_f;
    opt_acc(k,j) = a(k);
    
    % Update state
    x(2*k+1:2*k+2, j) = phy*x(2*k-1:2*k, j) + B*opt_X(1)+ C*Ul(k,j)+C*U_zeta_all(k,1); % x(:, k+1)的意思是将第k+1列的数值赋予x
    parameters.xmeasure = x(2*k-1:2*k, j);
    
    % initial solution
    parameters.init_X = opt_X;
end
    a1 = zeros(prediction_step_num, 1);
    Ul(:,j+1) = [a(:,j);a1];
end

t_end = toc(t_start);
disp(['Overall Time Cost: ', num2str(t_end), 'seconds.']);
diary off


%% 	Figure
U_all = Ul(1:case_num,:);

speed_all = 15.*ones(1,veh_num+1);

position_all = [62 50 38 26 14 2];


% speed
for j = 1:veh_num+1
for k = 1:case_num-1      
speed_all(k+1,j)  = speed_all(k,j) + tau.* U_all(k,j);
position_all(k+1,j) =  position_all(k,j) + tau.* speed_all(k,j) + tau^2/2.* U_all(k,j) ;
end    
end 
fig_handle = figure;
plt = PlotSettings; 

t = 0.01:0.01:20;
for j = 1:veh_num
for k = 1:case_num 
speed_error(k,j)=  speed_all(k,j) - speed_all(k,j+1);
position_error(k,j)=  position_all(k,j) - position_all(k,j+1)-r*speed_all(k,j+1)-length-Dis;
end
end

mean_position_error = mean(position_error);

figure(1)
 for j = 1:veh_num+1
     y = U_all(:,j);
    plot(t,y, ...
    plt.line_styles{j}, ...
     'Color', plt.line_colors{j}, ...
     'LineWidth', 2);
      hold all
      box on
      grid on
 end
xlabel_str =  'Time (s)';
ylabel_str = '$a_{i}(k)$ (m/s)';
 set(gca,'XLim',[0 20]);
legend({'Leader0','Follower1','Follower2','Follower3','Follower4','Follower5'},...
    'location','northwest','NumColumns',1);
title_str = 'DRMPC (Ours)';
SetLabelLegendTitle([], xlabel_str, ylabel_str, title_str, ...
    plt); % SetLabelLegendTitle can be re-used in other scripts


figure(2)
 for j = 1:veh_num+1
     y = speed_all(:,j);
    plot(t,y, ...
    plt.line_styles{j}, ...
     'Color', plt.line_colors{j}, ...
     'LineWidth', 2);
      hold all
      box on
      grid on
 end
xlabel_str =  'Time (s)';
ylabel_str = '$v_{i}(k)$ (m/s)';
 set(gca,'XLim',[0 20]);
legend({'Leader0','Follower1','Follower2','Follower3','Follower4','Follower5'},...
    'location','northwest','NumColumns',1);
title_str = [];
SetLabelLegendTitle([], xlabel_str, ylabel_str, title_str, ...
    plt); % SetLabelLegendTitle can be re-used in other scripts
% 
figure(3)
 for j = 1:veh_num+1
     y = position_all(:,j);
    plot(t,y, ...
    plt.line_styles{j}, ...
     'Color', plt.line_colors{j}, ...
     'LineWidth', 2);
      hold all
      box on
      grid on
 end
xlabel_str =  'Time (s)';
ylabel_str = '$p_{i}(k)$ (m)';
legend({'Leader0','Follower1','Follower2','Follower3','Follower4','Follower5'},...
    'location','northwest','NumColumns',1);
 set(gca,'XLim',[0 20]);
title_str = [];
SetLabelLegendTitle([], xlabel_str, ylabel_str, title_str, ...
    plt); % SetLabelLegendTitle can be re-used in other scripts
%      'Color', plt.line_colors{j}, ...
% 
figure(4)
 for j = 1:veh_num
     y = speed_error(:,j);
    plot(t,y, ...
    plt.line_styles{j}, ...
    'Color', plt.line_colors{j}, ...
     'LineWidth', 2);
      hold all
      box on
      grid on
 end
xlabel_str =  'Time (s)';
ylabel_str = '$\bigtriangleup v_i(k)$ (m/s)';
legend({'Leader0 & Follower1','Follower1&2','Follower2&3','Follower3&4','Follower4&5'},...
    'location','northwest','NumColumns',1);
set(gca,'XLim',[0 20]);
%set(gca,'YLim',[-0.5 0.2]);
title_str = 'DRMPC (Ours)';
SetLabelLegendTitle([], xlabel_str, ylabel_str, title_str, ...
    plt); % SetLabelLegendTitle can be re-used in other scripts

figure(5)
 for j = 1:veh_num
     y = position_error(:,j);
    plot(t,y, ...
    plt.line_styles{j}, ...
     'Color', plt.line_colors{j}, ...
     'LineWidth', 2);
      hold all
      box on
      grid on
 end
xlabel_str =  'Time (s)';
ylabel_str = '$\bigtriangleup p_i(k) -d_{i}$ (m)';
plot(xlim,[0,0],'k-.','LineWidth',1);
set(gca,'XLim',[0 20]);
set(gca,'YLim',[-0.5 2]);
title_str = 'DRMPC (Ours)';
legend({'Leader0 \& Follower1','Follower1\&2','Follower2\&3','Follower3\&4','Follower4\&5'},'Interpreter','latex',...
  'location','northwest','NumColumns',1);
SetLabelLegendTitle([], xlabel_str, ylabel_str, title_str, ...
    plt); % SetLabelLegendTitle can be re-used in other scripts



% The folder to store the results and figures
save_path  = '..\ours';

data_filename = ['simulation_DRMPC(Uniform)_string_on_', ...
    datestr(now,'yyyy_mm_dd_HH_MM'), '.mat'];

% Save all the data
save(fullfile(save_path, data_filename));





