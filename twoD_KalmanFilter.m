
function [x_tag_p,P_0_p,P_total]= twoD_KalmanFilter(sigma_wx,sigma_wy,sigma_v,M,N,path_true,meas_true)
%% system parameters 
% sigma_wx=2;
% sigma_wy=2;
pos=path_true;
z=meas_true;
P_total=zeros(4,4*300);

T=1;
F=([1,T,0,0;0,1,0,0;0,0,1,T;0,0,0,1]);
G=[0.5*T^2,0;T,0;0,0.5*T^2;0,T];
H=[1,0,0,0;0,0,1,0];
Q=G*([sigma_wx^2,0;0,sigma_wy^2])*(G.');


R=sigma_v^2;
x_true=zeros(4,N);
x_true(1,1:N)=pos(1,1:N);
x_true(3,1:N)=pos(2,1:N);

%% Kalman Filter equations 

% GIVEN VALUES
P_0_m=zeros(4); %%start value as Zgiven
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
 P_m=F*P_0_p*F.'+Q;
 K=P_m*(H.')*(inv(H*P_m*(H.')+R*eye(2)));%daniel debug: to put R*eye(2) as meshtane
 

 x_tag_p(:,2)=x_tag_m(:,2)+K*(z(:,1)-H*x_tag_m(:,2)); % k=2 but its first measurment
 P_0_p=(eye(4)-K*H)*P_m;
 P_total(:,1:4)=P_0_p;
 
 % other iterations 
for k=3:N+1
   x_tag_m(:,k)=F*x_tag_p(:,k-1);
 P_m=F*P_0_p*F.'+Q;
 K=P_m*(H.')*(inv(H*P_m*(H.')+R*eye(2)));
 
x_tag_p(:,k)=x_tag_m(:,k)+K*(z(:,k-1)-H*x_tag_m(:,k)); 
 P_0_p=(eye(4)-K*H)*P_m;
 P_total(:,((k-1)*4-3):(k-1)*4)=P_0_p;
end


% % final plot
% % figure(1);
% % plot(x_true(1,1:N),x_true(3,1:N),'b');
% % hold on;2
% % plot(x_tag_p(1,1:N),x_tag_p(3,1:N),'red');
% % hold on;
% % plot(z(1,1:N),z(2,1:N));
