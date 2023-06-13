% parameters
M=2;
N=300;
MC_iter  =200;
Vx=100;
Vy=-100;
x0=0;
y0=0;
w=0.01*pi;
sigma_v=50;
sigma_wx=2;
sigma_wy=2;

sen_num=5;
path_center=6500;
sensors_mat=zeros(2,sen_num);
r=8000;
i=sen_num;
while(i>0)
   theta=(2*pi/(sen_num))*i;
   sensors_mat(1,sen_num-i+1)=r*cos(theta)+path_center;
   sensors_mat(2,sen_num-i+1)=r*sin(theta);
   i=i-1; 
end
[pos] = path_for_2D_KalmanFilter(Vx,Vy,x0,y0,w,M,N); 


% covarince of delta_x in comparison with monte carlo 

SE_tri_x =zeros(MC_iter,N);
SE_tri_y =zeros(MC_iter,N);
for i=1:MC_iter 
   [tri_mat,cov_tri]= trilateration(pos,sigma_v,sensors_mat,N,sen_num);
   err_tri_x= pos(1,:)-tri_mat(1,:);
   err_tri_y= pos(2,:)-tri_mat(2,:);
   SE_tri_x(i,:) = SE_tri_x(i,:) + err_tri_x.^2; 
   SE_tri_y(i,:) = SE_tri_y(i,:) + err_tri_y.^2; 
     
end 

MSE_tri_x= sum(SE_tri_x)/MC_iter;
MSE_tri_y= sum(SE_tri_y)/MC_iter;
RMSE_tri_x=sqrt(MSE_tri_x);
RMSE_tri_y=sqrt(MSE_tri_y);

cov_total_x=zeros(1,300);
for j=1:300
  cov_total_x(j)=cov_tri(1,j*2-1);  
end
time=linspace(1,300,300);
figure(2);
plot(time,sqrt(cov_total_x));
hold on;
plot(time,RMSE_tri_x);
title('RMSE of tri and sqrt(cov of delta_x) vs. time');