clear functions %
clear all

% masses
slip.m1   =  1;
slip.m2  =  1;    
slip.m3  =  1;  
slip.m4   =  1;
slip.m5  =  1;    

% inertias
slip.I1  =  0.5*eye(3);  
slip.I2  =  0.5*eye(3);  
slip.I3  =  3*eye(3);

% length
slip.l1  =  0.5;
slip.l2  =  0.5;
slip.l3  =  0.5;
slip.l4  =  0.5;
slip.l5  =  0.5;

% simulation para
framespersec=50;  %if view is not speeded or slowed in dbpend_animate
T=5;             %duration of animation  in seconds
tspan=linspace(0,T,T*framespersec);

% initial state
q1    = pi/2; 
qd1    = 0;    
q2    = 0 ;  
qd2    = 0;
q3    = 0; 
qd3    = 0; 
q4    = 0 ;
qd4    = 0; 
q5    = 0;
qd5    = 0;   

%state vector
z0=[q1 qd1 q2 qd2 q3 qd3 q4 qd4 q5 qd5]';

%test kinematics
 [P1,P2,P3,P4,P5,Pcenter]=kinematic(z0,i,slip);

%plot
figure(1)

    plot(Pcenter(1),Pcenter(2),'ko','MarkerSize',8); %centroid point
    hold on
    line([0 P1(1)],[0 P1(2)],'Linewidth',4,'Color',[0.8 0 0]);% 1 pendulum
    line([P1(1) P2(1)],[P1(2) P2(2)],'Linewidth',4,'Color',[0 0.8 0]);% 2 pendulum
    line([P3(1) P2(1)],[P3(2) P2(2)],'Linewidth',4,'Color',[0 0 0.8]);% 3 pendulum
    line([P3(1) P4(1)],[P3(2) P4(2)],'Linewidth',4,'Color',[1 0.8 0.2]);% 4 pendulum
    line([P5(1) P4(1)],[P5(2) P4(2)],'Linewidth',4,'Color',[1 0.2 1]);% 5 pendulum

    axis([-5.5*slip.l1 5.5*slip.l1 -5.5*slip.l1 5.5*slip.l1]);
    axis square
    hold off