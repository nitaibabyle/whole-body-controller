close all; clc;clear;

syms q1 q2 q3 q4 q5 dq1 dq2 dq3 dq4 dq5 ddq1 ddq2 ddq3 ddq4 ddq5 'real' % joint angle velocity acceleration
syms l1 l2 l3 l4 l5 'real'             % link length
syms m1 m2 m3 m4 m5 'real'     % mass 
syms I1 I2 I3 'real'     % inertia
syms g 'real'                       % gravatical
syms tal1 tal2 tal3 tal4 tal5 'real'   %input


%%%20211111 zjt
I=[I1 0 0;
    0 I2 0;
    0 0 I3];

% unit vectors
i = [1; 0; 0];  j = [0; 1; 0];  k = [0; 0; 1];
% variables vectorization
q = [q1 q2 q3 q4 q5]';
qd = [dq1 dq2 dq3 dq4 dq5]';
qdd = [ddq1 ddq2 ddq3 ddq4 ddq5]';
G = -g * j; % gravity vector
aaaa=[0,0,0,1];% homogeneous matrix
R1=[cos(q1), -sin(q1), 0;
    sin(q1), cos(q1), 0;
          0,       0, 1];
O1=[0;0;0];
%rotation matrix and the origin

R2=[cos(q2), -sin(q2), 0;
    sin(q2), cos(q2), 0;
           0,       0, 1];  
R12=R1*R2;
O2=[l1;0;0];
%rotation matrix and the origin

R3=[cos(q3), -sin(q3), 0;
    sin(q3), cos(q3), 0;
           0,       0, 1];  
R13=R1*R2*R3;
O3=[l2;0;0];
%rotation matrix and the origin

R4=[ cos(q4), -sin(q4), 0;
    sin(q4), cos(q4), 0;
           0,       0, 1];  
R14=R1*R2*R3*R4;
O4=[l3;0;0];
%rotation matrix and the origin

R5=[ cos(q5), -sin(q5), 0;
    sin(q5), cos(q5), 0;
           0,       0, 1];  
R15=R1*R2*R3*R4*R5;
O5=[l4;0;0];
%rotation matrix and the origin

%transformation matrix local
T1=[R1,O1;
    aaaa];
T2=[R2,O2;
    aaaa];
T3=[R3,O3;
    aaaa];
T4=[R4,O4;
    aaaa];
T5=[R5,O5;
    aaaa];

%transformation matrix world
T12=T1*T2;
T13=T1*T2*T3;
T14=T1*T2*T3*T4;
T15=T1*T2*T3*T4*T5;

%joint position
P1=T1*[l1;0;0;1];
P2=T12*[l2;0;0;1];
P3=T13*[l3;0;0;1];
P4=T14*[l4;0;0;1];
P5=T15*[l5;0;0;1];

%mass postion
PG1=T1*[l1/2;0;0;1]; 
PG2=T12*[l2/2;0;0;1];
PG3=T13*[l3/2;0;0;1]; 
PG4=T14*[l4/2;0;0;1];
PG5=T15*[l5/2;0;0;1]; 


P_ee=P5(1:3);%end effector position
P_cent=(PG1(1:3)*m1+PG2(1:3)*m2+PG3(1:3)*m3+PG4(1:3)*m4+PG5(1:3)*m5)/(m1+m2+m3+m4+m5);%centroid position

V_l1 = jacobian(PG1,q)*qd;       % link  1 CoM velocity
V_l2 = jacobian(PG2,q)*qd;       % link  2 CoM velocity
V_l3 = jacobian(PG3,q)*qd;       % link  3 CoM velocity
V_l4 = jacobian(PG4,q)*qd;       % link  4 CoM velocity
V_l5 = jacobian(PG5,q)*qd;       % link  5 CoM velocity



Jee=jacobian(P_ee(1:3),q); %end effector jacobian
Jeedot=[(jacobian(Jee(1,:),q)*qd)';
       (jacobian(Jee(2,:),q)*qd)';
       (jacobian(Jee(3,:),q)*qd)'];% end effector jacobian first deriviation
   
Jct=jacobian(P_cent(1:3),q);%centroid jacobian
Jctdot=[(jacobian(Jct(1,:),q)*qd)';
       (jacobian(Jct(2,:),q)*qd)';
       (jacobian(Jct(3,:),q)*qd)'];% centroid jacobian first deriviation

W_l1 = dq1 * k;                % link  1 angular velocity
W_l2 = (dq1 + dq2) * k;        % link  2 angular velocity
W_l3 = (dq1 + dq2+dq3) * k;                % link  3 angular velocity
W_l4 = (dq1 + dq2+dq3+dq4) * k;        % link  4 angular velocity
W_l5 = (dq1 + dq2+dq3+dq4+dq5) * k;                % link  5 angular velocity


% inertia momentum w.r.t. inertial frame(a.k.a. world frame)

I_l1_w = R1 * I * R1';       % link   1 inertia
I_l2_w = R12 * I * R12';       % link   2 inertia
I_l3_w = R13 * I * R13';       % link   3 inertia
I_l4_w = R14 * I * R14';       % link   4 inertia
I_l5_w = R15 * I * R15';       % link   5 inertia

% kinematic energy
KE = 0.5 * m1 * dot(V_l1, V_l1) + 0.5 *m2 * dot(V_l2, V_l2) + 0.5 * W_l1' * I_l1_w * W_l1 + 0.5 * W_l2' * I_l2_w * W_l2...
    +0.5 * m3 * dot(V_l3, V_l3) + 0.5 *m4 * dot(V_l4, V_l4) + 0.5 * W_l3' * I_l3_w * W_l3 + 0.5 * W_l4' * I_l4_w * W_l4...
    +0.5 * m5 * dot(V_l5, V_l5) + 0.5 * W_l5' * I_l5_w * W_l5;

%vertical pos
pg1=PG1(1:3);
pg2=PG2(1:3);
pg3=PG3(1:3);
pg4=PG4(1:3);
pg5=PG5(1:3);

% potential energy
PE = m1 * g * PG1(2) + m2 * g * PG2(2)...
    +m3 * g * PG3(2) + m4 * g * PG4(2)...
    +m5 * g * PG5(2);

%% Lagrangian derivation
L = simplify(KE - PE);
DL_Ddq = jacobian(L,qd);

DL_Dq = jacobian(L,q);

dDL_Ddq_dt = jacobian(DL_Ddq, q) * qd + jacobian(DL_Ddq, qd) *qdd;

tal=[tal1;tal2;tal3;tal4;tal5];
eqn = simplify(dDL_Ddq_dt - DL_Dq'-tal);%dynamic equation

[MM, FF] = equationsToMatrix(eqn, qdd); % convert equation to : MM * ddq = FF
