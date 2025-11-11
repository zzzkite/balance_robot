% 参数假设
mp = 1;
mw = 0.5;
M = 10;
Iw = 0.0324;
Ip = 0.02;
Im = 0.3;
R = 0.18;
l = 0.15;
L = 0.25;
Lm = 0.3;
g = 9.8;

% 定义变量
syms Iw;
syms R;
syms mw;
syms mp;
syms M;
syms L;
syms Lm;
syms l;
syms Im;
syms Ip;
syms g;

syms theta;
syms x;
syms phi;
syms t;
syms theta1;
syms theta2;
syms x1;
syms x2;
syms phi1;
syms phi2;
syms T;
syms Tp;

syms Nm;
syms N;
syms Pm;
syms P;
syms L0;

    R1=0.15/2;                         %驱动轮半径
    Lq1=double(0.35/2);                  %摆杆重心到驱动轮轴距离
    Lm1=double(0.35/2);                 %摆杆重心到其转轴距离
    l1=0.03;                          %机体质心距离转轴距离
    mw1=1;                         %驱动轮质量
    mp1=0.55;                         %杆质量
    M1=15;                          %机体质量
    Iw1=mw1*R1^2;                     %驱动轮转动惯量
    Ip1=mp1*((Lq1+Lm1)^2+0.05^2)/12.0; %摆杆转动惯量
    Im1=M1*(0.3^2+0.12^2)/12.0;       %机体绕质心转动惯量
    g1=9.8;

Nm=M*{x2+(L+Lm)*(-theta1*sin(theta)+theta2*cos(theta))-l*(-phi1*sin(phi)+phi2*cos(phi))};
N=mp*{x2+L*(-theta1*sin(theta)+theta2*cos(theta))}+Nm;
Pm=M*g+M*{(L+Lm)*(-theta2*sin(theta)-theta1*cos(theta))+l*(-phi2*sin(phi)-phi1*cos(phi))};
P=Pm+mp*g+mp*L*(-theta2*sin(theta)-theta1*cos(theta));

% 构建方程组进行解耦
f1 = x2==(T-N*R)/(Iw/R+mw*R);
f2 = Ip*theta2==(P*L+Pm*Lm)*sin(theta)-(N*L+Nm*Lm)*cos(theta)-T+Tp;
f3 = Im*phi2==Tp+Nm*l*cos(phi)+Pm*l*sin(phi);

% 得到解耦后的方程f1 f2 f3
equ=[f1,f2,f3];
ans_dott=solve(equ,[theta2 x2 phi2]);
z=[theta theta1 x x1 phi phi1]';
dz=[theta1 theta2 x1 x2 phi1 phi2]';
u=[T Tp]';

% 求解jacobi
A = jacobian([theta1 ans_dott.theta2 x1 ans_dott.x2 phi1 ans_dott.phi2],z');
B = jacobian([theta1 ans_dott.theta2 x1 ans_dott.x2 phi1 ans_dott.phi2],u');

% 变量赋值（并带入平衡点）
% A=subs(A,{theta theta1 x1 phi phi1 T Tp},{0 0 0 0 0 0 0});
A=subs(A,{theta theta1 x1 phi phi1 T Tp mp mw M Iw Ip Im R l L Lm g},{0 0 0 0 0 0 0 mp1 mw1 M1 Iw1 Ip1 Im1 R1 l1 Lq1 Lm1 g1});
B=subs(B,{theta theta1 x1 phi phi1 T Tp mp mw M Iw Ip Im R l L Lm g},{0 0 0 0 0 0 0 mp1 mw1 M1 Iw1 Ip1 Im1 R1 l1 Lq1 Lm1 g1});
A=double(A)
B=double(B)

% 控制器设计
Q = [10 0 0 0 0 0
     0 1 0 0 0 0
     0 0 100 0 0 0
     0 0 0 10 0 0
     0 0 0 0 5000 0
     0 0 0 0 0 1];
R = [1 0
     0 0.25];
K = lqr(A,B,Q,R)

% 控制器反馈(注意这里线性化了，实际的状态变量要加上平衡点)
zd = [0 0 0 3 0 0]';
zf = [0 0 0 0 0 0];
z_actual = z + zf; 
% u = K(zd - z);

% VMC部分
Ld = 0.55; %腿长参考值
Lm = Lm1;
L = Lq1;

L1 = 0.25;
L2 = 0.3;
L3 = L2;
L4 = L1;
L5 = 0.1;
ph1 = pi*2/3;
ph4 = pi/9;

xb=L1*cos(ph1);
yb=L1*sin(ph1);
xd=L5+L4*cos(ph4);
yd=L4*sin(ph4);

A0 = 2*L2*(xd-xb);
B0 = 2*L2*(yd-yb);
LBD_2 = (xd-xb)^2 + (yd-yb)^2; 
C0 = L2^2 - L3^2 + LBD_2;

% ph2_first = 2*atan( (B0+sqrt(A0^2+B0^2-C0^2)) / (C0+A0) );
% ph2_second = 2*atan( (B0-sqrt(A0^2+B0^2-C0^2)) / (C0+A0) );
% 
% % 定义上应该取正值，换算成角度(弧度制)如下，实际要根据机械限制来判断怎么取，这里取较长时候的值
% if(ph2_first>0)
%     ph2 = ph2_first;
% else
%     ph2 = ph2_second;
% end
ph2 = 2*atan2((B0 + sqrt(A0*A0 + B0*B0 - C0*C0)),A0 + C0);
ph3 = atan2(yb-yd+L2*sin(ph2),xb-xd+L2*cos(ph2));

% 最终的C点坐标和ph2角
xc=xb+L2*cos(ph2);
yc=yb+L2*sin(ph2);

% 极坐标下的腿长和角度(弧度制)
ph0 = atan2(yc,xc-L5/2); 
theta = pi/2 - ph0
L0 = sqrt((xc-L5/2)^2+yc^2)

% 解算输出部分d_theta
phi_1 = ph1;
phi_4 = ph4;
d_phi1 = pi/9;
d_phi4 = pi/9;

    l5=L5;
    l1=L1;
    l2=L2;
    l3=L3;
    l4=L4;

d_phi0 = (d_phi1*((l1*cos(phi_1) + (2*l2*cos(2*atan(((4*l2^2*(l1*sin(phi_1) - l4*sin(phi_4))^2 - ((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2)^2 + 4*l2^2*(l5 - l1*cos(phi_1) + l4*cos(phi_4))^2)^(1/2) - 2*l2*(l1*sin(phi_1) - l4*sin(phi_4)))/((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2 + 2*l2*(l5 - l1*cos(phi_1) + l4*cos(phi_4)))))*(((8*l1*l2^2*cos(phi_1)*(l1*sin(phi_1) - l4*sin(phi_4)) - 2*(2*l1*cos(phi_1)*(l1*sin(phi_1) - l4*sin(phi_4)) + 2*l1*sin(phi_1)*(l5 - l1*cos(phi_1) + l4*cos(phi_4)))*((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2) + 8*l1*l2^2*sin(phi_1)*(l5 - l1*cos(phi_1) + l4*cos(phi_4)))/(2*(4*l2^2*(l1*sin(phi_1) - l4*sin(phi_4))^2 - ((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2)^2 + 4*l2^2*(l5 - l1*cos(phi_1) + l4*cos(phi_4))^2)^(1/2)) - 2*l1*l2*cos(phi_1))/((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2 + 2*l2*(l5 - l1*cos(phi_1) + l4*cos(phi_4))) - (((4*l2^2*(l1*sin(phi_1) - l4*sin(phi_4))^2 - ((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2)^2 + 4*l2^2*(l5 - l1*cos(phi_1) + l4*cos(phi_4))^2)^(1/2) - 2*l2*(l1*sin(phi_1) - l4*sin(phi_4)))*(2*l1*l2*sin(phi_1) + 2*l1*cos(phi_1)*(l1*sin(phi_1) - l4*sin(phi_4)) + 2*l1*sin(phi_1)*(l5 - l1*cos(phi_1) + l4*cos(phi_4))))/((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2 + 2*l2*(l5 - l1*cos(phi_1) + l4*cos(phi_4)))^2))/(((4*l2^2*(l1*sin(phi_1) - l4*sin(phi_4))^2 - ((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2)^2 + 4*l2^2*(l5 - l1*cos(phi_1) + l4*cos(phi_4))^2)^(1/2) - 2*l2*(l1*sin(phi_1) - l4*sin(phi_4)))^2/((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2 + 2*l2*(l5 - l1*cos(phi_1) + l4*cos(phi_4)))^2 + 1))/(l2*cos(2*atan(((4*l2^2*(l1*sin(phi_1) - l4*sin(phi_4))^2 - ((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2)^2 + 4*l2^2*(l5 - l1*cos(phi_1) + l4*cos(phi_4))^2)^(1/2) - 2*l2*(l1*sin(phi_1) - l4*sin(phi_4)))/((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2 + 2*l2*(l5 - l1*cos(phi_1) + l4*cos(phi_4))))) - l5/2 + l1*cos(phi_1)) + ((l2*sin(2*atan(((4*l2^2*(l1*sin(phi_1) - l4*sin(phi_4))^2 - ((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2)^2 + 4*l2^2*(l5 - l1*cos(phi_1) + l4*cos(phi_4))^2)^(1/2) - 2*l2*(l1*sin(phi_1) - l4*sin(phi_4)))/((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2 + 2*l2*(l5 - l1*cos(phi_1) + l4*cos(phi_4))))) + l1*sin(phi_1))*(l1*sin(phi_1) + (2*l2*sin(2*atan(((4*l2^2*(l1*sin(phi_1) - l4*sin(phi_4))^2 - ((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2)^2 + 4*l2^2*(l5 - l1*cos(phi_1) + l4*cos(phi_4))^2)^(1/2) - 2*l2*(l1*sin(phi_1) - l4*sin(phi_4)))/((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2 + 2*l2*(l5 - l1*cos(phi_1) + l4*cos(phi_4)))))*(((8*l1*l2^2*cos(phi_1)*(l1*sin(phi_1) - l4*sin(phi_4)) - 2*(2*l1*cos(phi_1)*(l1*sin(phi_1) - l4*sin(phi_4)) + 2*l1*sin(phi_1)*(l5 - l1*cos(phi_1) + l4*cos(phi_4)))*((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2) + 8*l1*l2^2*sin(phi_1)*(l5 - l1*cos(phi_1) + l4*cos(phi_4)))/(2*(4*l2^2*(l1*sin(phi_1) - l4*sin(phi_4))^2 - ((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2)^2 + 4*l2^2*(l5 - l1*cos(phi_1) + l4*cos(phi_4))^2)^(1/2)) - 2*l1*l2*cos(phi_1))/((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2 + 2*l2*(l5 - l1*cos(phi_1) + l4*cos(phi_4))) - (((4*l2^2*(l1*sin(phi_1) - l4*sin(phi_4))^2 - ((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2)^2 + 4*l2^2*(l5 - l1*cos(phi_1) + l4*cos(phi_4))^2)^(1/2) - 2*l2*(l1*sin(phi_1) - l4*sin(phi_4)))*(2*l1*l2*sin(phi_1) + 2*l1*cos(phi_1)*(l1*sin(phi_1) - l4*sin(phi_4)) + 2*l1*sin(phi_1)*(l5 - l1*cos(phi_1) + l4*cos(phi_4))))/((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2 + 2*l2*(l5 - l1*cos(phi_1) + l4*cos(phi_4)))^2))/(((4*l2^2*(l1*sin(phi_1) - l4*sin(phi_4))^2 - ((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2)^2 + 4*l2^2*(l5 - l1*cos(phi_1) + l4*cos(phi_4))^2)^(1/2) - 2*l2*(l1*sin(phi_1) - l4*sin(phi_4)))^2/((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2 + 2*l2*(l5 - l1*cos(phi_1) + l4*cos(phi_4)))^2 + 1)))/(l2*cos(2*atan(((4*l2^2*(l1*sin(phi_1) - l4*sin(phi_4))^2 - ((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2)^2 + 4*l2^2*(l5 - l1*cos(phi_1) + l4*cos(phi_4))^2)^(1/2) - 2*l2*(l1*sin(phi_1) - l4*sin(phi_4)))/((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2 + 2*l2*(l5 - l1*cos(phi_1) + l4*cos(phi_4))))) - l5/2 + l1*cos(phi_1))^2))/((l2*sin(2*atan(((4*l2^2*(l1*sin(phi_1) - l4*sin(phi_4))^2 - ((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2)^2 + 4*l2^2*(l5 - l1*cos(phi_1) + l4*cos(phi_4))^2)^(1/2) - 2*l2*(l1*sin(phi_1) - l4*sin(phi_4)))/((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2 + 2*l2*(l5 - l1*cos(phi_1) + l4*cos(phi_4))))) + l1*sin(phi_1))^2/(l2*cos(2*atan(((4*l2^2*(l1*sin(phi_1) - l4*sin(phi_4))^2 - ((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2)^2 + 4*l2^2*(l5 - l1*cos(phi_1) + l4*cos(phi_4))^2)^(1/2) - 2*l2*(l1*sin(phi_1) - l4*sin(phi_4)))/((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2 + 2*l2*(l5 - l1*cos(phi_1) + l4*cos(phi_4))))) - l5/2 + l1*cos(phi_1))^2 + 1) - (d_phi4*((2*l2*cos(2*atan(((4*l2^2*(l1*sin(phi_1) - l4*sin(phi_4))^2 - ((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2)^2 + 4*l2^2*(l5 - l1*cos(phi_1) + l4*cos(phi_4))^2)^(1/2) - 2*l2*(l1*sin(phi_1) - l4*sin(phi_4)))/((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2 + 2*l2*(l5 - l1*cos(phi_1) + l4*cos(phi_4)))))*(((8*l2^2*l4*cos(phi_4)*(l1*sin(phi_1) - l4*sin(phi_4)) - 2*(2*l4*cos(phi_4)*(l1*sin(phi_1) - l4*sin(phi_4)) + 2*l4*sin(phi_4)*(l5 - l1*cos(phi_1) + l4*cos(phi_4)))*((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2) + 8*l2^2*l4*sin(phi_4)*(l5 - l1*cos(phi_1) + l4*cos(phi_4)))/(2*(4*l2^2*(l1*sin(phi_1) - l4*sin(phi_4))^2 - ((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2)^2 + 4*l2^2*(l5 - l1*cos(phi_1) + l4*cos(phi_4))^2)^(1/2)) - 2*l2*l4*cos(phi_4))/((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2 + 2*l2*(l5 - l1*cos(phi_1) + l4*cos(phi_4))) - (((4*l2^2*(l1*sin(phi_1) - l4*sin(phi_4))^2 - ((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2)^2 + 4*l2^2*(l5 - l1*cos(phi_1) + l4*cos(phi_4))^2)^(1/2) - 2*l2*(l1*sin(phi_1) - l4*sin(phi_4)))*(2*l2*l4*sin(phi_4) + 2*l4*cos(phi_4)*(l1*sin(phi_1) - l4*sin(phi_4)) + 2*l4*sin(phi_4)*(l5 - l1*cos(phi_1) + l4*cos(phi_4))))/((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2 + 2*l2*(l5 - l1*cos(phi_1) + l4*cos(phi_4)))^2))/((((4*l2^2*(l1*sin(phi_1) - l4*sin(phi_4))^2 - ((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2)^2 + 4*l2^2*(l5 - l1*cos(phi_1) + l4*cos(phi_4))^2)^(1/2) - 2*l2*(l1*sin(phi_1) - l4*sin(phi_4)))^2/((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2 + 2*l2*(l5 - l1*cos(phi_1) + l4*cos(phi_4)))^2 + 1)*(l2*cos(2*atan(((4*l2^2*(l1*sin(phi_1) - l4*sin(phi_4))^2 - ((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2)^2 + 4*l2^2*(l5 - l1*cos(phi_1) + l4*cos(phi_4))^2)^(1/2) - 2*l2*(l1*sin(phi_1) - l4*sin(phi_4)))/((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2 + 2*l2*(l5 - l1*cos(phi_1) + l4*cos(phi_4))))) - l5/2 + l1*cos(phi_1))) + (2*l2*sin(2*atan(((4*l2^2*(l1*sin(phi_1) - l4*sin(phi_4))^2 - ((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2)^2 + 4*l2^2*(l5 - l1*cos(phi_1) + l4*cos(phi_4))^2)^(1/2) - 2*l2*(l1*sin(phi_1) - l4*sin(phi_4)))/((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2 + 2*l2*(l5 - l1*cos(phi_1) + l4*cos(phi_4)))))*(l2*sin(2*atan(((4*l2^2*(l1*sin(phi_1) - l4*sin(phi_4))^2 - ((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2)^2 + 4*l2^2*(l5 - l1*cos(phi_1) + l4*cos(phi_4))^2)^(1/2) - 2*l2*(l1*sin(phi_1) - l4*sin(phi_4)))/((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2 + 2*l2*(l5 - l1*cos(phi_1) + l4*cos(phi_4))))) + l1*sin(phi_1))*(((8*l2^2*l4*cos(phi_4)*(l1*sin(phi_1) - l4*sin(phi_4)) - 2*(2*l4*cos(phi_4)*(l1*sin(phi_1) - l4*sin(phi_4)) + 2*l4*sin(phi_4)*(l5 - l1*cos(phi_1) + l4*cos(phi_4)))*((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2) + 8*l2^2*l4*sin(phi_4)*(l5 - l1*cos(phi_1) + l4*cos(phi_4)))/(2*(4*l2^2*(l1*sin(phi_1) - l4*sin(phi_4))^2 - ((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2)^2 + 4*l2^2*(l5 - l1*cos(phi_1) + l4*cos(phi_4))^2)^(1/2)) - 2*l2*l4*cos(phi_4))/((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2 + 2*l2*(l5 - l1*cos(phi_1) + l4*cos(phi_4))) - (((4*l2^2*(l1*sin(phi_1) - l4*sin(phi_4))^2 - ((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2)^2 + 4*l2^2*(l5 - l1*cos(phi_1) + l4*cos(phi_4))^2)^(1/2) - 2*l2*(l1*sin(phi_1) - l4*sin(phi_4)))*(2*l2*l4*sin(phi_4) + 2*l4*cos(phi_4)*(l1*sin(phi_1) - l4*sin(phi_4)) + 2*l4*sin(phi_4)*(l5 - l1*cos(phi_1) + l4*cos(phi_4))))/((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2 + 2*l2*(l5 - l1*cos(phi_1) + l4*cos(phi_4)))^2))/((((4*l2^2*(l1*sin(phi_1) - l4*sin(phi_4))^2 - ((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2)^2 + 4*l2^2*(l5 - l1*cos(phi_1) + l4*cos(phi_4))^2)^(1/2) - 2*l2*(l1*sin(phi_1) - l4*sin(phi_4)))^2/((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2 + 2*l2*(l5 - l1*cos(phi_1) + l4*cos(phi_4)))^2 + 1)*(l2*cos(2*atan(((4*l2^2*(l1*sin(phi_1) - l4*sin(phi_4))^2 - ((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2)^2 + 4*l2^2*(l5 - l1*cos(phi_1) + l4*cos(phi_4))^2)^(1/2) - 2*l2*(l1*sin(phi_1) - l4*sin(phi_4)))/((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2 + 2*l2*(l5 - l1*cos(phi_1) + l4*cos(phi_4))))) - l5/2 + l1*cos(phi_1))^2)))/((l2*sin(2*atan(((4*l2^2*(l1*sin(phi_1) - l4*sin(phi_4))^2 - ((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2)^2 + 4*l2^2*(l5 - l1*cos(phi_1) + l4*cos(phi_4))^2)^(1/2) - 2*l2*(l1*sin(phi_1) - l4*sin(phi_4)))/((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2 + 2*l2*(l5 - l1*cos(phi_1) + l4*cos(phi_4))))) + l1*sin(phi_1))^2/(l2*cos(2*atan(((4*l2^2*(l1*sin(phi_1) - l4*sin(phi_4))^2 - ((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2)^2 + 4*l2^2*(l5 - l1*cos(phi_1) + l4*cos(phi_4))^2)^(1/2) - 2*l2*(l1*sin(phi_1) - l4*sin(phi_4)))/((l1*sin(phi_1) - l4*sin(phi_4))^2 + (l5 - l1*cos(phi_1) + l4*cos(phi_4))^2 + l2^2 - l3^2 + 2*l2*(l5 - l1*cos(phi_1) + l4*cos(phi_4))))) - l5/2 + l1*cos(phi_1))^2 + 1);
d_theta = -d_phi0

% LQR解算关节电机力矩
% 假设
F = 100;

U1 = K*(zd - [theta 0 0 0 0 0]);
T=vpa(U1(1,1))
Tp=vpa(U1(1,2))

T1 = vpa( -L1*sin(ph1-ph2)*(Tp*cos(ph0-ph3) + F*L0*sin(ph0-ph3)) / (L0*sin(ph2-ph3)) )
T2 = vpa( -L4*sin(ph3-ph4)*(Tp*cos(ph0-ph2) + F*L0*sin(ph0-ph2)) / (L0*sin(ph2-ph3)) )