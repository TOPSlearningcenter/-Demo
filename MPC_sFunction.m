function [sys,x0,str,ts]=MPC_sFunction(t,x,u,flag)
switch flag
    case 0
        [sys,x0,str,ts]=mdlInitializeSizes;
    case 1
        sys=mdlDerivatives(t,x,u);
    case 3
        sys=mdlOutputs(t,x,u);
    case {2, 4, 9}
        sys = [];
    otherwise
        error(['Unhandled flag = ',num2str(flag)]);
end
end

function [sys,x0,str,ts]=mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 2;
sizes.NumInputs      = 3;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 0;
sys=simsizes(sizes);
x0=[];
str=[];
ts=[];

% 初始处理化处理计算整条轨迹的曲率车速和理论航向角
load("reference.mat")
% 一阶导数
dydx=diff(y_ref)./diff(x_ref);
% 二阶导数
d2ydx2 = diff(dydx)./diff(x_ref(1:end-1));

dydx=[dydx,dydx(end)];
d2ydx2=[d2ydx2(1),d2ydx2,d2ydx2(end)];

% 求曲率
Wq=zeros(size(t_ref));
for i=1:length(t_ref)
    Wq(i)=(d2ydx2(i))/((1+dydx(i)^2)^(1.5));
end

% 算参考横摆角
Psi_d = atan(dydx);

% 算参考车速
v_ref=zeros(size(t_ref));

for i=1:length(x_ref)-1
    Delta_dis(i) = norm([x_ref(i+1)-x_ref(i),y_ref(i+1)-y_ref(i)]);
end
v_ref(1:end-1) = Delta_dis./diff(t_ref);
v_ref(end)=v_ref(end-1);

save("reference_processed.mat",'t_ref','x_ref',"y_ref","Psi_d","Wq","v_ref");

end

function sys=mdlDerivatives(t,x,u)
sys=[];
end

function sys=mdlOutputs(t,x,u)

% u(1) psi
% u(2) x_g
% u(3) y_g

yaw=u(1);
x=u(2);
y=u(3);

load("reference_processed.mat")

refPos_x = x_ref';
refPos_y = y_ref';
refPos_yaw = Psi_d';
refPos_k = Wq';

dt = 0.1; % MPC计算步长,s
L=3;%车辆轴距，m
U=[0;0];% 初始输出
target_v = interp1(t_ref,v_ref,t); % 目标车速, m/s

[Delta_real,v_real]= mpc_control(x,y,yaw,refPos_x,refPos_y,refPos_yaw,refPos_k,dt,L,U,target_v);

sys(1)=Delta_real;
sys(2)=v_real;

end

function [Delta_real,v_real]= ....
    mpc_control(x,y,yaw,refPos_x,refPos_y,refPos_yaw,refPos_k,dt,L,U,target_v)
%% MPc预设参数

Nx=3; %状态量个数
Nu=2; % 控制量个数

Np=10; %预测步数
Nc=6; %控制步数


% 权重矩阵
Q=100*eye(Np*Nx);
R=50*eye(Nc*Nu);
rho=10;%松弛因子

%控制量约束条件
umin=[-0.2;-0.64];
umax=[0.2;0.64];
delta_umin=[-0.05;-0.1];%小写delta表示变化量
delta_umax=[0.05;0.1];

%% 原运动学误差状态方程的相关矩阵
%计算参考控制量
idx=calc_target_index(x,y,refPos_x,refPos_y);
curvature=refPos_k(idx);
Delta_r=atan(L*curvature);%参考前轮转角
v_r=target_v;% 参考车速

%实际状态量与参考状态量
X_real=[x,y,yaw];
Xr=[refPos_x(idx),refPos_y(idx),refPos_yaw(idx)];

%a,b两个矩阵
a=[ 1,0,-v_r*sin(yaw)*dt;
    0,1,v_r*cos(yaw)*dt;
    0,0,1];
b=[cos(yaw)*dt 0;
    sin(yaw)*dt 0;
    tan(yaw)*dt/L v_r*dt/(L*(cos(Delta_r)^2))];

%% 新状态空间方程
%新的状态量
kesi=zeros(Nx+Nu,1);
kesi(1:Nx)=X_real-Xr;
kesi(Nx+1:end)=U;

%新的A矩阵
A_cell=cell(2,2);
A_cell{1,1}=a;
A_cell{1,2}=b;
A_cell{2,1}=zeros(Nu,Nx);
A_cell{2,2}=eye(Nu);
A=cell2mat(A_cell);

%新的B矩阵
B_cell=cell(2,1);
B_cell{1,1}=b;
B_cell{2,1}=eye(Nu);
B=cell2mat(B_cell);

%新的C矩阵
C=[eye(Nx),zeros(Nx,Nu)];

%phi矩阵
PHI_cell=cell(Np,1);
for i=1:Np
    PHI_cell{i,1}=C*A^i;
end
PHI=cell2mat(PHI_cell);

%THETA矩阵
THETA_cell=cell(Np,Nc);
for i=1:Np
    for j=1:Nc
        if j<=i
            THETA_cell{i,j}=C*A^(i-j)*B;
        else
            THETA_cell{i,j}=zeros(Nx,Nu);
        end
    end
end

THETA=cell2mat(THETA_cell);

%% 二次型目标函数的相关矩阵
%H矩阵
H_cell=cell(2,2);
H_cell{1,1}=THETA'*Q*THETA+R;
H_cell{1,2}=zeros(Nu*Nc,1);
H_cell{2,1}=zeros(1,Nu*Nc);
H_cell{2,2}=rho;              %加入松弛因子得到最优解放到对角线上
H=cell2mat(H_cell);  %[Nu*Nc+1*Nu*Nc+1]

%E矩阵
E=PHI*kesi;

%g矩阵
g_cell=cell(1,2);
g_cell{1,1}=E'*Q*THETA;
g_cell{1,2}=0;  %为了与H匹配
g=cell2mat(g_cell);

%% 约束条件相关矩阵
%A_I矩阵
A_t=zeros(Nc,Nc); %下三角方阵
for i=1:Nc
    A_t(i,1:i)=1;
end
A_I=kron(A_t,eye(Nu));

%Ut矩阵
Ut=kron(ones(Nc,1),U);

%控制量与控制量变化量的约束
Umin=kron(ones(Nc,1),umin);
Umax=kron(ones(Nc,1),umax);
delta_Umin=kron(ones(Nc,1),delta_umin);
delta_Umax=kron(ones(Nc,1),delta_umax);

%二次型规划函数quadprog
%用于quadprog函数不等式约束Ax<=b的矩阵A
A_cons_cell={A_I,zeros(Nu*Nc,1);-A_I,zeros(Nu*Nc,1)};
A_cons=cell2mat(A_cons_cell);

%用quadprog函数不等式约束Ax<=b的向量b
b_cons_cell={Umax-Ut;-Umin+Ut};
b_cons=cell2mat(b_cons_cell);

%ΔU的上下界约束
lb=[delta_Umin;0];
ub=[delta_Umax;1];

%% 开始求解过程
options=optimoptions('quadprog','Display','iter','MaxIterations',100,'TolFun',1e-16);
delta_U=quadprog(H,g,A_cons,b_cons,[],[],lb,ub,[],options);

%% 计算输出
%只取求解的delta_U的第一组控制量
%为v_tilde变化量和Delta_tilde变化量（偏差量的变化量）
delta_v_tilde=delta_U(1);
delta_Delta_tilde=delta_U(2);

%更新这一时刻的控制量
%即速度和前轮转角的偏差量
U(1)=kesi(4)+delta_v_tilde;
U(2)=kesi(5)+delta_Delta_tilde;

%求解真正的控制量
v_real=U(1)+v_r;
Delta_real=U(2)+Delta_r;

end

% 轨迹最近点匹配
function target_idx = calc_target_index(x,y,refPos_x,refPos_y)

i=1:length(refPos_x)-1;
dist=sqrt((refPos_x(i)-x).^2+(refPos_y(i)-y).^2);
[~,target_idx]=min(dist);

end