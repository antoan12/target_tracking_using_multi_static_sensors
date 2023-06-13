function [tri_mat,cov_tri]= trilateration(pos,sigma_v,sensors_mat,N,sen_num)
cov_tri=zeros(2,2*N);

tri_mat=zeros(2,N);
%% first time point

% first iteration is with x0=real pos 

[z]= mdedot_multi_static_morhav(pos(:,1),sigma_v,sensors_mat);
[h,H] =  calcH_morhav_tri(pos(:,1),sensors_mat);

delta_z=z-h; %delta z is delta y in teaaorea..
delta_x=H\delta_z; 

obj_x=delta_x(1,1)+pos(1,1); 
obj_y=delta_x(2,1)+pos(2,1); 
[obj]=zeros(2,1);
obj(1,1)=obj_x;
obj(2,1)=obj_y;


% other iteration to improve delta_x by putting x0 to be the obj vector

while (norm(delta_x)> 0.01)
    

[h,H] =  calcH_morhav_tri(obj(:,1),sensors_mat);

delta_z=z-h;
delta_x=H\delta_z; 

obj_x=delta_x(1,1)+obj(1,1); 
obj_y=delta_x(2,1)+obj(2,1); 
[obj]=zeros(2,1);
obj(1,1)=obj_x;
obj(2,1)=obj_y;  
    
end 
tri_mat(1,1)=obj_x;
tri_mat(2,1)=obj_y;

cov_tri(:,1:2)=(H.'*H)\eye(2)*sigma_v^2;

%% other time points 

for k=2:N; % for is for the time points
 
 %first iteration is with the last obj values
[z]= mdedot_multi_static_morhav(pos(:,k),sigma_v,sensors_mat); 
[h,H] =  calcH_morhav_tri(obj(:,1),sensors_mat);

delta_z=z-h; %delta z is delta y in teaaorea..
delta_x=H\delta_z; 

obj_x=delta_x(1,1)+pos(1,k); 
obj_y=delta_x(2,1)+pos(2,k); 
[obj]=zeros(2,1);
obj(1,1)=obj_x;
obj(2,1)=obj_y;


% other iteration

while (norm(delta_x)> 0.01)
    

[h,H] =  calcH_morhav_tri(obj(:,1),sensors_mat);

delta_z=z-h;
delta_x=H\delta_z; 

obj_x=delta_x(1,1)+obj(1,1); 
obj_y=delta_x(2,1)+obj(2,1); 
[obj]=zeros(2,1);
obj(1,1)=obj_x;
obj(2,1)=obj_y;  
    
end 
    
tri_mat(1,k)=obj_x;
tri_mat(2,k)=obj_y;    

cov_tri(:,2*k-1:2*k)=(H.'*H)\eye(2)*sigma_v^2;
end  
   

%figures
figure(1); 
   plot(tri_mat(1,1:N),tri_mat(2,1:N),'blue');
  grid on;
  xlim([-500,13000]);
  ylim([-8000,8000]);  
hold on;
plot(sensors_mat(1,1:sen_num),sensors_mat(2,1:sen_num),'*');
hold on;
 plot(pos(1,1:N),pos(2,1:N),'red');
 
 

end 