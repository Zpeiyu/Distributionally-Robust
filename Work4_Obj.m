function val = Work4_Obj(X, parameters)

M_phy = parameters.M_phy;

M_B = parameters.M_B;

M_C = parameters.M_C;

Q_x = parameters.Q_x;

R_u = parameters.R_u;

K_bar = parameters.K_bar;

I1 = parameters.I1;

I2 = parameters.I2;

U_refer = parameters.U_refer;

U_zeta = parameters.U_zeta;

std_U_zeta =parameters.std_U_zeta;

xmeasure = parameters.xmeasure;

zeta = M_B'*Q_x*M_B + (K_bar*M_B + I1)'*R_u*(K_bar*M_B + I1);

h1 = 2*xmeasure'*M_phy'*Q_x*M_B;

h2 = 2*U_refer'*M_C'*Q_x*M_B;

h3 = 2*xmeasure'*M_phy'*K_bar'*R_u*(K_bar*M_B + I1);

h4 = 2*U_refer'*M_C'*K_bar'*R_u*(K_bar*M_B + I1);

h = h1 + h2+ h3 + h4; % 此处代表的是函数h的转置

% 常数项

s1 = M_C'*Q_x*M_C;

variance = I1.*std_U_zeta;

s2 = trace(s1*variance); % 矩阵的迹

s3 =  M_C'*K_bar'*R_u*K_bar * M_C;

s4 = trace(s3*variance); % 矩阵的迹

s = xmeasure'*M_phy'*Q_x*M_phy*xmeasure + ...
    2*xmeasure'*M_phy'*Q_x*M_C*U_refer+...
    U_refer'*M_C'*Q_x*M_C*U_refer+...
    U_zeta'*s2* U_zeta+...
    xmeasure'*M_phy'*K_bar'*R_u*K_bar*M_phy*xmeasure+...
    2*U_refer'*M_C'*K_bar'*R_u*K_bar*M_phy*xmeasure+...
    U_refer'*M_C'*K_bar'*R_u*K_bar*M_C*U_refer+...
    U_zeta'*s4*U_zeta;

val = X'* zeta * X + h * X +s;
    
end

