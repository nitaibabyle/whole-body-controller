function [p,pdot,pddot]=traj(t,z,homework,nnn)
T=15;
if nnn==1%%end traj
    if t<T
    w=pi/2;
        x=0.3*sin(w*(t));
        xd=0.3*w*cos(w*(t));
        xdd=-0.3*w*w*sin(w*(t));
        y=1.5-0.3*cos(w*(t));
        yd=0.3*w*sin(w*(t));
        ydd=0.3*w*w*cos(w*(t));%用正弦函数规划末端点的位置、速度、加速度
    p=[x,y,0]';
    pdot=[xd,yd,0]';
    pddot=[xdd,ydd,0]';
    else
        w=-pi/2;
        x=-1+0.5*sin(w*(t));
        xd=0.5*w*cos(w*(t));
        xdd=-0.5*w*w*sin(w*(t));
        y=1.1-0.5*cos(w*(t));
        yd=0.5*w*sin(w*(t));
        ydd=0.5*w*w*cos(w*(t));%用正弦函数规划末端点的位置、速度、加速度
        if x>-0.7
            x=-0.7;
            xd=0;
        end
        if x<-1.3
            x=-1.3;
            xd=0;
        end
        if y>1.4
            y=1.5;
            yd=0;
        end
        if y<0.8
            y=0.7;
            yd=0;
        end
        
    
    p=[x,y,0]';
    pdot=[xd,yd,0]';
    pddot=[xdd,ydd,0]';
    end
else%%center traj
    if t<T
    w=-pi/2;
        x=0.3*sin(w*(t));
        xd=0.3*w*cos(w*(t));
        xdd=-0.3*w*w*sin(w*(t));
        y=0.8-0.3*cos(w*(t));
        yd=0.3*w*sin(w*(t));
        ydd=0.3*w*w*cos(w*(t));%用正弦函数规划末端点的位置、速度、加速度
    
        if x>0.2
            x=0.2;
            xd=0;
        end
        if x<-0.2
            x=-0.2;
            xd=0;
        end
        if y>1
            y=1;
            yd=0;
        end
        if y<0.6
            y=0.6;
            yd=0;
        end
    p=[x,y,0]';
    pdot=[xd,yd,0]';
    pddot=[xdd,ydd,0]';
    else
        w=pi/2;
        x=-0.5-0.4*sin(w*(t));
        xd=-0.4*w*cos(w*(t));
        xdd=0.4*w*w*sin(w*(t));
        y=0.5-0.4*cos(w*(t));
        yd=0.4*w*sin(w*(t));
        ydd=0.4*w*w*cos(w*(t));%用正弦函数规划末端点的位置、速度、加速度
    p=[x,y,0]';
    pdot=[xd,yd,0]';
    pddot=[xdd,ydd,0]';
    end
end
