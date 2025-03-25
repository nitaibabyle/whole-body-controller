
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
homework.I2  =  0.5*eye(3);  
homework.I3  =  0.5*eye(3);  
homework.I4  =  0.5*eye(3); 
homework.I5  =  0.5*eye(3);  


homework.l1  =  0.5;
homework.l2  =  0.5;
homework.l3  =  0.5;
homework.l4  =  0.5;
homework.l5  =  0.5;
homework.g   =  10;
homework.leg =  0.9;
homework.kd  =  -0;


% Initial conditions and other settings.
framespersec=50;  

q1    = pi/2;
qd1    = 0;       
q2    = 0 ;   
qd2    = 0;   
q3    = 0; 
qd3    = 0;  
q4    = 0.5 ;
qd4    = 0;     
q5    = 0; 
qd5    = 0; 
u1=0;
u2=0;
u3=0;
u4=0;
u5=0;
z0=[q1 qd1 q2 qd2 q3 qd3 q4 qd4 q5 qd5];
t0=0;
t_ode=t0;
z_ode=z0;
dt=0.001;
N=10;
fps=30;

for i=1:4000
i
options=odeset('abstol',1e-13,'reltol',1e-13);
tspan=linspace(t0,t0+dt,N);

%%%%%%% INTEGRATOR or ODE SOLVER %%%%%%%
tal1=homework.kd*z0(2);
tal2=homework.kd*z0(4);
tal3=homework.kd*z0(6);
tal4=homework.kd*z0(8);
tal5=homework.kd*z0(10);

[t_temp,z_temp] = ode113(@rhs,tspan,z0,options,tal1,tal2,tal3,tal4,tal5,homework);

t0 = t_temp(end,:);
        z0 = z_temp(end,:);
        t_ode = [t_ode;t_temp(2:end)];
        z_ode = [z_ode;z_temp(2:end,:)];
end
[t,z] = loco_interpolate(t_ode,z_ode,fps);

%%%%%%% POSTPROCESSING %%%%%%%%%%%%%%%%
% A routine to animate the results
% To speed up change the framespersecond
for i=1:length(t)



    [P1,P2,P3,P4,P5,Pcenter]=kinematic(z(i,1:10),i,homework);

    window_xmin = -3*homework.l1*2; window_xmax =3*homework.l1*2;
    window_ymin = -3*homework.l1*2; window_ymax = 3*homework.l1*2;
    plot(Pcenter(1),Pcenter(2),'bo')

    line([0 P1(1)],[0 P1(2)],'Linewidth',4,'Color',[0.8 0 0]);
line([P1(1) P2(1)],[P1(2) P2(2)],'Linewidth',4,'Color',[0 0.8 0]);
line([P3(1) P2(1)],[P3(2) P2(2)],'Linewidth',4,'Color',[0 0 0.8]);
line([P3(1) P4(1)],[P3(2) P4(2)],'Linewidth',4,'Color',[1 0.8 0.2]);
line([P5(1) P4(1)],[P5(2) P4(2)],'Linewidth',4,'Color',[1 0.2 1]);
    
    axis('equal')
    axis on
    axis([window_xmin window_xmax window_ymin window_ymax])
    F(i)=getframe;
end
v = VideoWriter('123.avi');
open(v);
writeVideo(v,F);
close(v);

for i=1:length(t)
    TE(i)=energy(t,z(i,1:10),homework);
end

TE_diff = diff(TE);

figure(3)
plot(t(1:end-1),TE_diff)
