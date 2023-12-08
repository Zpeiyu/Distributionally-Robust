function [A, b] = Work4_SetInequationC(parameters)

prediction_step_num = parameters.prediction_step_num;

I1 = parameters.I1; % 对角线为

I2 = parameters.I2; % 对角线为1

M_phy = parameters.M_phy;

M_B = parameters.M_B;

M_C = parameters.M_C;

xmeasure = parameters.xmeasure;

U_refer = parameters.U_refer;

U_zeta = parameters.U_zeta;

Omege1 = parameters.Omege1; % 关于控制

Omege2 = parameters.Omege2;  % 关于状态


K_bar = parameters.K_bar;


% 两车间的距离差s_{i-1}-s_{i}-r*v_{i}-L-D的最大值为3m
% D为10m, 因此最大的上界为13m,速度差最大值为6m/s;

% 构建G_x
g1 = eye(2,2);
g2 = -1*eye(2,2);
g3 = [g1;g2];
G_x = blkdiag(g3,g3);
for i = 3:prediction_step_num
G_x = blkdiag(G_x,g3);    
end

% 构建f_x
x_max = [5; 3]; 
x_min = [0; -3];

f_x1 = [x_max;-x_min];
f_x =  repmat(f_x1,prediction_step_num,1);


% 构建关于状态约束的AX<=B
A1 = G_x * M_B;

b1 = f_x - G_x*M_phy*xmeasure - G_x*M_C*U_refer - G_x*M_C*Omege2*I2*((U_zeta).^2).^0.5;


% 构建G_u
G_u1 = [1;-1];
G_u = blkdiag(G_u1,G_u1);
for i = 3:prediction_step_num
G_u = blkdiag(G_u,G_u1);    
end

% 构建f_u
f_u1 = [1.2;1.2];
f_u  = repmat(f_u1,prediction_step_num,1);


% 构建关于控制约束的AX<=B
 
A2 = G_u*(K_bar*M_B+I1);

b2 = f_u - G_u*K_bar*M_phy*xmeasure - G_u*K_bar*M_C*U_refer - G_u*K_bar*M_C*Omege1*I2*((U_zeta).^2).^0.5;


A = [A1; A2];

b = [b1; b2];

end

