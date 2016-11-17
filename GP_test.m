data1=load('/home/lci/workspace/wheelchair/data/seq3.txt');
[N2,d2]=size(data1);

for i=16:N2
    j=i-15;
    xtest_lf(j,1)=data1(i-15,2);
    xtest_lf(j,2)=data1(i-14,2);
    xtest_lf(j,3)=data1(i-13,2);
    xtest_lf(j,4)=data1(i-12,2);
    xtest_lf(j,5)=data1(i-11,2);
    xtest_lf(j,6)=data1(i-10,2);
    xtest_lf(j,7)=data1(i-9,2);
    xtest_lf(j,8)=data1(i-8,2);
    xtest_lf(j,9)=data1(i-7,2);
    xtest_lf(j,10)=data1(i-6,2);
    xtest_lf(j,11)=data1(i-5,2);
    xtest_lf(j,12)=data1(i-4,2);
    xtest_lf(j,13)=data1(i-3,2);
    xtest_lf(j,14)=data1(i-2,2);
    xtest_lf(j,15)=data1(i-1,2);
    xtest_lf(j,16)=data1(i,2);
    xtest_lf(j,17)=data1(i-15,5);
    xtest_lf(j,18)=data1(i-14,5);
    xtest_lf(j,19)=data1(i-13,5);
    xtest_lf(j,20)=data1(i-12,5);
    xtest_lf(j,21)=data1(i-11,5);
    xtest_lf(j,22)=data1(i-10,5);
    xtest_lf(j,23)=data1(i-9,5);
    xtest_lf(j,24)=data1(i-8,5);
    xtest_lf(j,25)=data1(i-7,5);
    xtest_lf(j,26)=data1(i-6,5);
    xtest_lf(j,27)=data1(i-5,5);
    xtest_lf(j,28)=data1(i-4,5);
    xtest_lf(j,29)=data1(i-3,5);
    xtest_lf(j,30)=data1(i-2,5);
    xtest_lf(j,31)=data1(i-1,5);
    ytest_angular(j,1)=data1(i,5);
end




% PREDICTION
[mu0_lf,s2_lf] = spgp_pred(y0_angular,x_lf,xb_lf,xtest_lf,hyp_lf); %mean and standard deviation
mu_lf = mu0_lf + me_y_angular; % add the mean back on
% if you want predictive variances to include noise variance add noise:
s2_lf = s2_lf + exp(hyp_lf(end));  
 

%%%%%%%%%
% Plotting - just for 1D demo - remove for real data set
% Hopefully, the predictions should look reasonable

clf
hold on
plot(x_lf,y_angular,'.m') % data points in magenta

plot(xtest_lf,mu_lf,'b') % mean predictions in blue
plot(xtest_lf,mu_lf+2*sqrt(s2_lf),'r') % plus/minus 2 std deviation  % predictions in red
%f = [mu+2*sqrt(s2);flipdim(mu-2*sqrt(s2),1)];
%fill([xtest; flipdim(xtest,1)], f, [7 7 7]/8, 'EdgeColor', [7 7 7]/8);
hold on;               

plot(xtest_lf,mu_lf-2*sqrt(s2_lf),'r')
% x-location of pseudo-inputs as black crosses
plot(xb_lf,-2.75*ones(size(xb_lf)),'k+','markersize',20)
hold off
title('Spare Gaussian Processes using Pseudo-inputs')

[N3_lf,d3_lf]= size(mu_lf);
for i=1:N3_lf
    test_error_lf(i,1)=mu_lf(i,1)-ytest_angular(i,1);
    relative_test_error_lf(i,1)=abs(test_error_lf(i,1))/ytest_angular(i,1);
    error_sd_lf(i,1)=test_error_lf(i,1)/sqrt(s2_lf(i,1));
end  
  

figure 

plot(test_error_lf)
D_lf = abs(mu_lf-ytest_angular).^2;
MSE = sum(D_lf(:))/numel(mu_lf)
title('Sparse GP prediction error for angular velocity')

figure
plot(test_error_lf,'color','r')
hold on;
plot(mu_lf,'color','b');
hold on;
plot(ytest_angular, 'color','y');
legend('error','prediction','ground truth');
title('Sparse GP for angular velocity prediction')

figure

processed_relative_error=relative_test_error_lf(abs(relative_test_error_lf)<6);

histogram(processed_relative_error,100)
title('Relative test error histogram')

figure
histogram(test_error_lf,100)
title('Test error histogram')

figure
plot(error_sd_lf,'color','g');
title('Sparse GP for angular velocity prediction error over variance');

figure
histogram(error_sd_lf)
title('Sparse GP for angular velocity prediction error over standard deviation histogram');


figure
plot(mu_lf,'color','b')
hold on;
plot(mu_lf+2*sqrt(s2_lf),'color','r')
hold on;
plot(mu_lf-2*sqrt(s2_lf),'color','r')
legend('mean prediction mu','mu+2*Sigma','mu-2*Sigma');
title('Sparse GP for angular velocity prediction')

figure
histogram(ytest_angular,300)
title('Ground truth angular velocity histogram')

figure
histogram(mu_lf,300)
title('Predicted angular velocity histogram')


for i=16:N2
    j=i-15;
    xtest_fb(j,1)=data1(i-15,3);
    xtest_fb(j,2)=data1(i-14,3);
    xtest_fb(j,3)=data1(i-13,3);
    xtest_fb(j,4)=data1(i-12,3);
    xtest_fb(j,5)=data1(i-11,3);
    xtest_fb(j,6)=data1(i-10,3);
    xtest_fb(j,7)=data1(i-9,3);
    xtest_fb(j,8)=data1(i-8,3);
    xtest_fb(j,9)=data1(i-7,3);
    xtest_fb(j,10)=data1(i-6,3);
    xtest_fb(j,11)=data1(i-5,3);
    xtest_fb(j,12)=data1(i-4,3);
    xtest_fb(j,13)=data1(i-3,3);
    xtest_fb(j,14)=data1(i-2,3);
    xtest_fb(j,15)=data1(i-1,3);
    xtest_fb(j,16)=data1(i,3);
    xtest_fb(j,17)=data1(i-15,4);
    xtest_fb(j,18)=data1(i-14,4);
    xtest_fb(j,19)=data1(i-13,4);
    xtest_fb(j,20)=data1(i-12,4);
    xtest_fb(j,21)=data1(i-11,4);
    xtest_fb(j,22)=data1(i-10,4);
    xtest_fb(j,23)=data1(i-9,4);
    xtest_fb(j,24)=data1(i-8,4);
    xtest_fb(j,25)=data1(i-7,4);
    xtest_fb(j,26)=data1(i-6,4);
    xtest_fb(j,27)=data1(i-5,4);
    xtest_fb(j,28)=data1(i-4,4);
    xtest_fb(j,29)=data1(i-3,4);
    xtest_fb(j,30)=data1(i-2,4);
    xtest_fb(j,31)=data1(i-1,4);
    ytest_linear(j,1)=data1(i,4);
end
% load demo data set (1D inputs for easy visualization-
% this script should work fine for multidimensional inputs)


% PREDICTION
[mu0_fb,s2_fb] = spgp_pred(y0_fb,x_fb,xb_fb,xtest_fb,hyp_fb); %mean and standard deviation
mu_fb = mu0_fb + me_y_fb; % add the mean back on
% if you want predictive variances to include noise variance add noise:
s2_fb = s2_fb + exp(hyp_fb(end));  
 

%%%%%%%%%
% Plotting - just for 1D demo - remove for real data set
% Hopefully, the predictions should look reasonable

clf
hold on
plot(x_fb,y_linear,'.m') % data points in magenta

plot(xtest_fb,mu_fb,'b') % mean predictions in blue
plot(xtest_fb,mu_fb+2*sqrt(s2_fb),'r') % plus/minus 2 std deviation  % predictions in red
%f = [mu+2*sqrt(s2);flipdim(mu-2*sqrt(s2),1)];
%fill([xtest; flipdim(xtest,1)], f, [7 7 7]/8, 'EdgeColor', [7 7 7]/8);
hold on;               

plot(xtest_fb,mu_fb-2*sqrt(s2_fb),'r')
% x-location of pseudo-inputs as black crosses
plot(xb_fb,-2.75*ones(size(xb_fb)),'k+','markersize',20)
hold off
title('Spare Gaussian Processes using Pseudo-inputs')

[N3_fb,d3_fb]= size(mu_fb);
for i=1:N3_fb
    test_error_fb(i,1)=mu_fb(i,1)-ytest_linear(i,1);
    relative_test_error_fb(i,1)=abs(test_error_fb(i,1))/ytest_linear(i,1);
    error_sd_fb(i,1)=test_error_fb(i,1)/sqrt(s2_fb(i,1));
end  
  


figure 

plot(test_error_fb)
D_fb = abs(mu_fb-ytest_linear).^2;
MSE = sum(D_fb(:))/numel(mu_fb)
title('Sparse GP prediction error for linear velocity')

figure
plot(test_error_fb,'color','r')
hold on;
plot(mu_fb,'color','b');
hold on;
plot(ytest_linear, 'color','y');
legend('error','prediction','ground truth');
title('Sparse GP for linear velocity prediction')

figure

processed_relative_error_fb=relative_test_error_fb(abs(relative_test_error_fb)<1);

histogram(processed_relative_error_fb,300)
title('Sparse GP linear velocity relative training error histogram')
xlim([-1 1])

figure
histogram(test_error_fb,100)
title('Test error histogram')



figure
plot(error_sd_fb,'color','g');
title('Sparse GP for linear velocity prediction error over variance');

figure
histogram(error_sd_fb)
title('Sparse GP for linear velocity prediction error over standard deviation histogram');


figure
plot(mu_fb,'color','b')
hold on;
plot(mu_fb+2*sqrt(s2_fb),'color','r')
hold on;
plot(mu_fb-2*sqrt(s2_fb),'color','r')
legend('mean prediction mu','mu+2*Sigma','mu-2*Sigma');
title('Sparse GP for linear velocity prediction')

figure
histogram(ytest_linear,300)
title('Ground truth linear velocity histogram')

figure
histogram(mu_fb,300)
title('Predicted linear velocity histogram')

