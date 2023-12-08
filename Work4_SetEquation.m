function [Aeq, beq] = Work4_SetEquation(parameters)
% 建立等式约束
prediction_step_num = parameters.prediction_step_num;

phy = parameters.A;

B = parameters.B;

xmeasure = parameters.xmeasure;

S_phy = size(phy, 1);

S_B = size(B, 1);

U_refer = parameters.U_refer;

M_phy = parameters.M_phy;
M_B = parameters.M_B;
M_C = parameters.M_C;

M_phy1 = M_phy (S_phy*(prediction_step_num-1)+1:S_phy*(prediction_step_num-1)+S_phy,:);
M_B1 = M_B (S_B*(prediction_step_num-1)+1:S_B*(prediction_step_num-1)+S_B,:);
M_C1 = M_C (S_B*(prediction_step_num-1)+1:S_B*(prediction_step_num-1)+S_B,:);

Aeq = M_B1; 

beq = -M_phy1 * xmeasure -M_C1*U_refer;
end

