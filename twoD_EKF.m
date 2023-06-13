
function [x_tag_p,P_0_p]= twoD_EKF(sigma_wx,sigma_wy,sigma_v,M,N,path_true,sen1_pos_x,sen1_pos_y,sen2_pos_x,sen2_pos_y)
%% system parameters 
% sigma_wx=2;
% sigma_wy=2;


pos=path_true;


T=1;
F=([1,T,0,0;0,1,0,0;0,0,1,T;0,0,0,1]);
G=[0.5*T^2,0;T,0;0,0.5*T^2;0,T];
%%H=[1,0,0,0;0,0,1,0];
Q=G*([sigma_wx^2,0;0,sigma_wy^2])*(G.');


R=sigma_v^2;
x_true=zeros(4,N);
x_true(1,1:N)=pos(1,1:N);
x_true(3,1:N)=pos(2,1:N);

%% Kalman Filter equations 

% GIVEN VALUES
P_0_m=(10^5)*eye(4); %%start value as Zgiven
P_0_p=P_0_m;
x_0_m=([0,100,0,0]).'; %% start value as given
x_tag_m=zeros(4,N);
x_tag_p=zeros(4,N);
%first iteration (which is starting point)
x_tag_p(:,1)=x_0_m;
x_tag_m(:,1)=x_0_m;

%%  time update and measurment update ...notes: history of x is saved but P,K are not 
% second iteration which is first measuremnt
 
 x_tag_m(:,2)=F*x_tag_p(:,1);
 
 [z]= mdedot_multi_static(pos(:,1),sigma_v,sen1_pos_x,sen1_pos_y,sen2_pos_x,sen2_pos_y);
 [h,H] =  calcH(x_tag_m(:,2),sen1_pos_x,sen1_pos_y,sen2_pos_x,sen2_pos_y);
 
 P_m=F*P_0_p*F.'+Q;

 K=P_m*(H.')*(inv(H*P_m*(H.')+R*eye(2)));

 x_tag_p(:,2)=x_tag_m(:,2)+K*(z-h); % k=2 but its first measurment 
 P_0_p=(eye(4)-K*H)*P_m;
 
 % other iterations 
for k=3:N+1
 x_tag_m(:,k)=F*x_tag_p(:,k-1);
 
 [z]= mdedot_multi_static(pos(:,k-1),sigma_v,sen1_pos_x,sen1_pos_y,sen2_pos_x,sen2_pos_y);
 [h,H] =  calcH(x_tag_m(:,k),sen1_pos_x,sen1_pos_y,sen2_pos_x,sen2_pos_y);
 
 P_m=F*P_0_p*F.'+Q;

 K=P_m*(H.')*(inv(H*P_m*(H.')+R*eye(2)));
 
 x_tag_p(:,k)=x_tag_m(:,k)+K*(z-h); 
 P_0_p=(eye(4)-K*H)*P_m;
end


% final plot
figure(1);
plot(x_true(1,1:N),x_true(3,1:N),'b');
hold on;
plot(x_tag_p(1,1:N),x_tag_p(3,1:N),'red');


