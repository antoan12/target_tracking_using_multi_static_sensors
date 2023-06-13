
function [pos]= path_for_2D_KalmanFilter(Vx,Vy,x0,y0,w,M,N)
%% path
% N=300; 
% M=2;
pos=zeros(M,N); 
velocity=zeros(M,N);
dt=1;
% parameters for linear 
% Vx=100; 
% Vy=-100;

% parameters for circle

% delta_theta=1.8;
v_circle=pi;

% first iteration is zeros -> start path from (0,0)
pos(1,1)=x0;
pos(2,1)=y0;
for i=2:N %first coulmn is zeros
    if(i<=101) % linear
        pos(1,i)=pos(1,i-1)+dt*Vx;
        velocity(1,i)=Vx;
        velocity(2,i)=0;
    end 
   
    if(i>(N/3+1) && i<=2*(N/3+1)) % circle ,100 iterations
   
%      w=(delta_theta*2*pi)/360;
       r=v_circle/w; % this is the circular movement equation
       pos(1,i)=pos(1,i-1)+dt*r*cos((((i-102)*2.7)*2*pi/360));
        pos(2,i)=pos(2,i-1)+dt*r*sin((((i-102)*2.7)*2*pi/360));
         velocity(1,i)=pos(1,i)-pos(1,i-1);
         velocity(2,i)=pos(2,i)-pos(2,i-1);
    end 
    if(i> 2*(N/3+1) && i<=N) % for second linear 
       pos(1,i)=pos(1,i-1);
       pos(2,i)=pos(2,i-1)+dt*Vy; 
       velocity(2,i)=Vy;
       velocity(1,i)=0;  
    end
        

    
end 
%   figure(1); 
%   plot(pos(1,1:N),pos(2,1:N));
%   grid on;
%   ylim([-2000,5000]); 
%   
