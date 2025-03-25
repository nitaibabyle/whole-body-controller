
clear functions %
clear all

%%%%%%%%% INITIALIZE PARAMETERS %%%%%%
%Mechanical parameters.
homework.m1  =  1;    
homework.m2  =  1;  % masses
homework.m3  =  1;    
homework.m4  =  1;  % masses
homework.m5  =  1;    



homework.I1  =  0.5*eye(3);  
homework.I2  =  0.5*eye(3);  % inertias about cmsslip.l1   =  1
homework.I3  =  0.5*eye(3);  
homework.I4  =  0.5*eye(3);  % inertias about cmsslip.l1   =  1
homework.I5  =  0.5*eye(3);  


homework.l1  =  0.5;
homework.l2  =  0.5;
homework.l3  =  0.5;
homework.l4  =  0.5;
homework.l5  =  0.5;
homework.g   =  10;
homework.leg =  0.9;
homework.kp  =  0;
homework.kd  =  -0.5;
global pee_ref;
pee_ref=[0;-0.8660254037844386];
global pee_real;
pee_real=[0;-0.8660254037844386];


global ttt;
ttt=0;
% Initial conditions and other settings.
framespersec=50;  %if view is not speeded or slowed in dbpend_animate
         %duration of animation  in seconds

q1    = -pi/2; %angle made by link1 with vertical
qd1    = 0;        %abslolute velocity of link1   
q2    = 0.5 ;      %angle made by link2 with vertical
qd2    = 0;        %abslolute velocity of link2
q3    = 0; %angle made by link1 with vertical
qd3    = 0;        %abslolute velocity of link1   
q4    = 0.5 ;      %angle made by link2 with vertical
qd4    = 0;        %abslolute velocity of link2
q5    = 0; %angle made by link1 with vertical
qd5    = 0;        %abslolute velocity of link1 
u1=0;
u2=0;
u3=0;
u4=0;
u5=0;

t0=0;
z0=[q1 qd1 q2 qd2 q3 qd3 q4 qd4 q5 qd5];
t_ode=t0;
z_ode=z0;
dt=0.001;
N=10;
fps=10;
tic
for i=1:40000
i
%%%%%%%%%%%%%% CONTROLLER %%%%%%%%%%%%%% 


%z0=z0+0.1*rand(1,4);%%%信号的随机扰动
%4s后施加扰动
% if i==3000
%     z0(4)=z0(4)+0.05;
% end
% 

[tal1,tal2,tal3,tal4,tal5]=controller(t0,z0,homework);%%%pd control
%[tal1,tal2]=inverse_dynamic_controller(t0,z0,homework);%%%inverse dynamic control,based on feed-forward torque
% tal1=0;tal2=0;


%%%%%%% INTEGRATOR or ODE SOLVER %%%%%%%
options=odeset('abstol',1e-13,'reltol',1e-13);
tspan=linspace(t0,t0+dt,N);
[t_temp,z_temp] = ode113(@rhs,tspan,z0,options,tal1,tal2,tal3,tal4,tal5,homework);

t0 = t_temp(end,:);
        z0 = z_temp(end,:);
        t_ode = [t_ode;t_temp(2:end)];
        z_ode = [z_ode;z_temp(2:end,:)];
 
end
[t,z] = loco_interpolate(t_ode,z_ode,fps);
toc
%%%%%%% POSTPROCESSING %%%%%%%%%%%%%%%%
% A routine to animate the results
% To speed up change the framespersecond
figure(1)
pee=[];
X=[0,0];
Y=[0,0];
for i=1:length(t)

%     q1=z(i,1);
%     q2=z(i,3);
%     
%     xm1=slip.l1*sin(q1);%-l*sin(z(i,1));%modified by cxc in Jan. 2018
%     ym1=slip.l1*-cos(q1);
%     zm1=0;
%     xm2=slip.l1*sin(q1) + slip.l2*sin(q1-q2);%xm1-l*sin(z(i,3));%modified by cxc in Jan. 2018
%     ym2=slip.l1*-cos(q1) + slip.l2*-cos(q1-q2);
%     zm2=0;

    [P1,P2,P3,P4,P5,Pcenter]=kinematic(z(i,1:10),i,homework);

    window_xmin = -3*homework.l1*2; window_xmax = 3*homework.l1*2;
    window_ymin = -3*homework.l1*2; window_ymax =3*homework.l1*2;
    X=[X,Pcenter(1),P5(1)];
    Y=[Y,Pcenter(2),P5(2)];
    plot(X,Y,'o','color',[0.5 1 1], 'markersize',5, 'linewidth', 1);
    
    %plot([0],[0],'ko','MarkerSize',3); %pivot point
    line([0 P1(1)],[0 P1(2)],'Linewidth',4,'Color',[0.8 0 0]);% first pendulum
line([P1(1) P2(1)],[P1(2) P2(2)],'Linewidth',4,'Color',[0 0.8 0]);% second pendulum
line([P3(1) P2(1)],[P3(2) P2(2)],'Linewidth',4,'Color',[0 0 0.8]);% second pendulum
line([P3(1) P4(1)],[P3(2) P4(2)],'Linewidth',4,'Color',[1 0.8 0.2]);% second pendulum
line([P5(1) P4(1)],[P5(2) P4(2)],'Linewidth',4,'Color',[1 0.2 1]);% second pendulum
    
    axis('equal')
    axis on
    axis([window_xmin window_xmax window_ymin window_ymax])
    F(i)=getframe;
end
v = VideoWriter('123.avi');
open(v);
writeVideo(v,F);
close(v);

aa=pee_ref-pee_real;
figure(3)
hold on
xlabel('时间(s)')
ylabel('误差(m)')
title('跟踪误差')
plot(ttt(1:end),pee_ref(1,:)-pee_real(1,:),'r-')
plot(ttt(1:end),pee_ref(2,:)-pee_real(2,:),'b-')
legend('水平方向','竖直方向') 
