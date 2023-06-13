
function [z]= mdedot_for_2D_KalmanFilter(sigma_v,M,N,path_true)
%% path with noise with std=50 
pos=path_true;
z=zeros(M,N);
% sigma_v=50;


for k=2:N+1
rand1=sigma_v*randn;
rand2=sigma_v*randn;
z(1,k-1)=(pos(1,k-1)+rand1);
z(2,k-1)=(pos(2,k-1)+rand2);

end
% figure(1); 
%   plot(z(1,1:N),z(2,1:N));
%   grid on;
%   ylim([-2000,5000]); 



