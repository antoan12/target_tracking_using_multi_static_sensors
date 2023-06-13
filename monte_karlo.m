%% monte karlo

% get the random values
iteration_num=200; % number of nesoyeem
error_matx=zeros(iteration_num,N); %%each row is for one iteration, for x axes
error_maty=zeros(iteration_num,N); %%each row is for one iteration, for y axes

for i=1:iteration_num %% after each iteration here we finish one row for x and one row for y
    for j=1:N
       rand1=sigma_v*randn;
       rand2=sigma_v*randn;
       error_matx(i,j)=rand1;
       error_maty(i,j)=rand2;   
        
    end
end

 % SE (squarred error)
  sigma_mat=ones(iteration_num,N).*sigma_v;
  se_matx=error_matx-sigma_mat;      
  se_maty=error_maty-sigma_mat;  
  
  se_matx= se_matx.* se_matx;
  se_maty= se_maty.* se_maty;
  
  % MSE 
  
  mse_matx=zeros(1,N);
  mse_maty=zeros(1,N);
  for j=1:N %% go through each one of the col to get the average -> so we get the represntive value of SE
      countx=0;
      county=0;
     for i=1:iteration_num
        countx=countx+ se_matx(i,j);
        county=county+ se_maty(i,j); 
     end
   mse_matx(j)=countx/iteration_num;
   mse_maty(j)=county/iteration_num;
  end
  
  % RMSE 
   rmse_matx=sqrt(mse_matx);
   rmse_maty=sqrt(mse_maty);
  
      
  