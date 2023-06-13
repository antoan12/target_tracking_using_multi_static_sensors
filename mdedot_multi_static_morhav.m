function [z]= mdedot_multi_static_morhav(pos,sigma_v,sensors_mat)%%x_tag_m just one coulmn each time

sens_num=width(sensors_mat);
% object position
  ob_pos_x=pos(1,:);
  ob_pos_y=pos(2,:);
  
  z=zeros(sens_num,1);
 % distance from main sensor
   sen1_ob_without_noise= sqrt((ob_pos_x-sensors_mat(1,1))^2+(ob_pos_y-sensors_mat(2,1))^2);
   sen1_ob= sen1_ob_without_noise +sigma_v*randn;
   z(1,1)=sen1_ob;%main sensor tvah 
   
   for i=1:sens_num-1 %% disntances to all bestatic 
       
   sen2_ob=sqrt((sensors_mat(1,i+1)-ob_pos_x)^2+(sensors_mat(2,i+1)-ob_pos_y)^2);
   sen2_total= sen1_ob_without_noise+sen2_ob+sigma_v*randn;
       
    z(i+1,1)=sen2_total; %secondary sensor tvah 

   end
  
end 
   