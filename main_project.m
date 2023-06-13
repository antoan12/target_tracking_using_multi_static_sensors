M=2;
N=300;
MC_iter  = 200;
vx=100;
vy=-100;
x0=0;
y0=0;
w=0.01*pi;
sigma_v=50;
sigma_wx=200;
sigma_wy=200;



SE_flt1_x = zeros(MC_iter,N); %%each row is for one nesoy, only for x axes
SE_meas_x =zeros(MC_iter,N);
SE_flt1_y = zeros(MC_iter,N); %%each row is for one nesoy, only for y axes
SE_meas_y =zeros(MC_iter,N);

for i_iter = 1:MC_iter
    path_true = path_for_2D_KalmanFilter(vx,vy,x0,y0,w,M,N);
    meas_true = mdedot_for_2D_KalmanFilter(sigma_v,M,N,path_true);
    [x_hat1,P,P_total] =  twoD_KalmanFilter(sigma_wx,sigma_wy,sigma_v,M,N,path_true,meas_true); % x_hat1=x_tag_p,P = P_0_p
    
    x_hat1_pos_x(1,:) = x_hat1(1,:); %% to get pos of x from x_hat
    x_hat1_pos_y(1,:) = x_hat1(3,:); %% to get pos of y from x_hat
    
    err_kf_x= path_true(1,:)- x_hat1_pos_x(1,2:(N+1));
    err_kf_y= path_true(2,:)- x_hat1_pos_y(1,2:(N+1));
    
    err_meas_x= path_true(1,:)-meas_true(1,:);
    err_meas_y= path_true(2,:)-meas_true(2,:);
    
    SE_flt1_x(i_iter,:) = SE_flt1_x(i_iter,:) + err_kf_x.^2; % this is se not mse
    SE_flt1_y(i_iter,:) = SE_flt1_y(i_iter,:) + err_kf_y.^2; 
    
    SE_meas_x(i_iter,:) = SE_meas_x(i_iter,:) + err_meas_x.^2; 
    SE_meas_y(i_iter,:) = SE_meas_y(i_iter,:) + err_meas_y.^2; 
    
end

MSE_flt1_x = sum(SE_flt1_x)/MC_iter; %returns a row vector containing the sum of each column
MSE_flt1_y = sum(SE_flt1_y)/MC_iter;
RMSE_flt1_x = sqrt(MSE_flt1_x);
RMSE_flt1_y = sqrt(MSE_flt1_y);


MSE_meas_x= sum(SE_meas_x)/MC_iter;
MSE_meas_y= sum(SE_meas_y)/MC_iter;
RMSE_meas_x=sqrt(MSE_meas_x);
RMSE_meas_y=sqrt(MSE_meas_y);

% RMSE_flt1 = sqrt(MSE_flt1); % 4 X  path length should be similar to sqrt(diag(P))
% RMSE_meas = sqrt(MSE_meas); % 4 X  path length. should be simialar to sigma_v
% cheking these two:
sigma_v_300_points=sigma_v*ones(1,300);
figure(1);
time=linspace(1,300,300);
plot(time,sigma_v_300_points);
hold on;
plot(time,RMSE_meas_x);
ylim([0,100]);
title('RMSE of meas and sigma_v vs. time');

figure(2);
P_total_x=zeros(1,300);
for j=1:300
  P_total_x(j)=P_total(1,j*4-3);  
end
plot(time,sqrt(P_total_x));
hold on;
plot(time,RMSE_flt1_x);
ylim([25,75]);
title('RMSE of flt and sqrt(diagP) vs. time');