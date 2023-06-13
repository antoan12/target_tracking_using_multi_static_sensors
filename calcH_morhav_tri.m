function [h,H]= calcH_morhav_tri(x_tag_m,sensors_mat) %%x_tag_m just one coulmn each time

sens_num=width(sensors_mat);
%object position
ob_pos_x=x_tag_m(1,:); 
ob_pos_y=x_tag_m(2,:);
 

 h=zeros(sens_num,1);
 H=zeros(sens_num,2);
 
%main sensor 
h(1,1)=sqrt((ob_pos_x-sensors_mat(1,1))^2+(ob_pos_y-sensors_mat(2,1))^2); %first sensor tavhem 
H(1,1)=(2*ob_pos_x - 2*sensors_mat(1,1))/(2*((ob_pos_x - sensors_mat(1,1))^2 + (ob_pos_y - sensors_mat(2,1))^2)^(1/2)); %diff(h1,x,x_tag_m(1,2))
H(1,2)=(2*ob_pos_y - 2*sensors_mat(2,1))/(2*((ob_pos_x - sensors_mat(1,1))^2 + (ob_pos_y - sensors_mat(2,1))^2)^(1/2));   %diff(h1,y,x_tag_m(2,2));

% bestatic sensors
 for i=1:sens_num-1 
       
   h(i+1,1)= h(1,1)+sqrt((sensors_mat(1,i+1)-ob_pos_x)^2+(sensors_mat(2,i+1)-ob_pos_y)^2);
   H(i+1,1)=(2*ob_pos_x - 2*sensors_mat(1,1))/(2*((ob_pos_x - sensors_mat(1,1))^2 + (ob_pos_y - sensors_mat(2,1))^2)^(1/2)) + (2*ob_pos_x - 2*sensors_mat(1,i+1))/(2*((ob_pos_x - sensors_mat(1,i+1))^2 + (ob_pos_y - sensors_mat(2,i+1))^2)^(1/2));
   H(i+1,2)=(2*ob_pos_y - 2*sensors_mat(2,1))/(2*((ob_pos_x - sensors_mat(1,1))^2 + (ob_pos_y - sensors_mat(2,1))^2)^(1/2)) + (2*ob_pos_y - 2*sensors_mat(2,i+1))/(2*((ob_pos_x - sensors_mat(1,i+1))^2 + (ob_pos_y - sensors_mat(2,i+1))^2)^(1/2));
   
       
  end










end