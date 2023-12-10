function K = Feekback_K_solution(parameters)

A = parameters.A;

B = parameters.B;

C = parameters.C;

Q = parameters.Q;

R = parameters.R;

Qf = parameters.Q;

T_array = parameters.T_array;

U_found = parameters.U_found;

[p,L,k] = dare(A,B,Q,R);

x0 = parameters.xmeasure;

N = T_array+5;

P=zeros(2,2,N+1);

P(:,:,N+1)=Qf;

 
for t=2:(N+1)
    tUsed=(N+3-t);
    P(:,:,tUsed-1)=Q+A'*P(:,:,tUsed)*A-A'*P(:,:,tUsed)*B/(R+B'*P(:,:,tUsed)*B)*B'*P(:,:,tUsed)*A;
end
s=P(:,:,1);
 
K=zeros(N,2);
U=zeros(N,1);
X=zeros(N+1,2);
X(1,:)=x0;
for t=1:N
    K(t,:)=-(R+B'*P(:,:,t+1)*B)\B'*P(:,:,t+1)*A;
    U(t,:)=K(t,:)*X(t,:)';
    X(t+1,:)=(A*X(t,:)'+B*U(t,:)+C*U_found(t,:))';
end
end

%t= 1:1:60;
%U_1 = Ul(1:60);
%plot(t, U1, t, U);


