clear all;
close all;

data=load('/home/lci/workspace/wheelchair/data/seq2.txt');


[N1,d1]=size(data);

for i=16:N1 
    j=i-15;
    x_fb(j,1)=data(i-15,3);
    x_fb(j,2)=data(i-14,3);
    x_fb(j,3)=data(i-13,3);
    x_fb(j,4)=data(i-12,3);
    x_fb(j,5)=data(i-11,3);
    x_fb(j,6)=data(i-10,3);
    x_fb(j,7)=data(i-9,3);
    x_fb(j,8)=data(i-8,3);
    x_fb(j,9)=data(i-7,3);
    x_fb(j,10)=data(i-6,3);
    x_fb(j,11)=data(i-5,3);
    x_fb(j,12)=data(i-4,3);
    x_fb(j,13)=data(i-3,3);
    x_fb(j,14)=data(i-2,3);
    x_fb(j,15)=data(i-1,3);
    x_fb(j,16)=data(i,3);
    x_fb(j,17)=data(i-15,4);
    x_fb(j,18)=data(i-14,4);
    x_fb(j,19)=data(i-13,4);
    x_fb(j,20)=data(i-12,4);
    x_fb(j,21)=data(i-11,4);
    x_fb(j,22)=data(i-10,4);
    x_fb(j,23)=data(i-9,4);
    x_fb(j,24)=data(i-8,4);
    x_fb(j,25)=data(i-7,4);
    x_fb(j,26)=data(i-6,4);
    x_fb(j,27)=data(i-5,4);
    x_fb(j,28)=data(i-4,4);
    x_fb(j,29)=data(i-3,4);
    x_fb(j,30)=data(i-2,4);
    x_fb(j,31)=data(i-1,4);
    y_linear(j,1)=data(i,4);
end





data1=load('/home/lci/workspace/wheelchair/data/seq3.txt');


[N2,d2]=size(data1);
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


me_y_fb = mean(y_linear);
y0_fb = y_linear - me_y_fb; % zero mean the data

[N_fb, dim_fb] = size(x_fb);

M_fb = 20; % number of pseudo-inputs

% initialize pseudo-inputs to a random subset of training inputs
[dum_fb, I_fb] = sort(rand(N_fb,1)); 
clear dum_fb;
I_fb = I_fb(1:M_fb);
xb_init = x_fb(I_fb,:);

% initialize hyperparameters sensibly (see spgp_lik for how 
% the hyperparameters are encoded)
hyp_init_fb(1:dim_fb,1) = -2*log((max(x_fb)-min(x_fb))'/2); %log 1/(lengthscales)^2
hyp_init_fb(dim_fb+1,1) = log(var(y0_fb,1)); % log size
hyp_init_fb(dim_fb+2,1) = log(var(y0_fb,1)/4); %log noise

% optimize hyperparameters and pseudo-inputs
w_init_fb = [reshape(xb_init, M_fb*dim_fb,1); hyp_init_fb];
[w_fb,f_fb] = minimize(w_init_fb, 'spgp_lik', -200, y0_fb, x_fb, M_fb);
% [w,f] = lbfgs(w_init,'spgp_lik',200,10,y0,x,M); % an alternative
xb_fb = reshape(w_fb(1:M_fb*dim_fb,1),M_fb,dim_fb);
hyp_fb = w_fb(M_fb*dim_fb+1:end,1);


% PREDICTION
[mu0_fb,s2_fb] = spgp_pred(y0_fb,x_fb,xb_fb,xtest_fb,hyp_fb); %mean and standard deviation
mu_fb = mu0_fb + me_y_fb; % add the mean back on
% if you want predictive variances to include noise variance add noise:
s2_fb = s2_fb + exp(hyp_fb(end));  
 


for i=16:N1 
    j=i-15;
    x_lf(j,1)=data(i-15,2);
    x_lf(j,2)=data(i-14,2);
    x_lf(j,3)=data(i-13,2);
    x_lf(j,4)=data(i-12,2);
    x_lf(j,5)=data(i-11,2);
    x_lf(j,6)=data(i-10,2);
    x_lf(j,7)=data(i-9,2);
    x_lf(j,8)=data(i-8,2);
    x_lf(j,9)=data(i-7,2);
    x_lf(j,10)=data(i-6,2);
    x_lf(j,11)=data(i-5,2);
    x_lf(j,12)=data(i-4,2);
    x_lf(j,13)=data(i-3,2);
    x_lf(j,14)=data(i-2,2);
    x_lf(j,15)=data(i-1,2);
    x_lf(j,16)=data(i,2);
    x_lf(j,17)=data(i-15,5);
    x_lf(j,18)=data(i-14,5);
    x_lf(j,19)=data(i-13,5);
    x_lf(j,20)=data(i-12,5);
    x_lf(j,21)=data(i-11,5);
    x_lf(j,22)=data(i-10,5);
    x_lf(j,23)=data(i-9,5);
    x_lf(j,24)=data(i-8,5);
    x_lf(j,25)=data(i-7,5);
    x_lf(j,26)=data(i-6,5);
    x_lf(j,27)=data(i-5,5);
    x_lf(j,28)=data(i-4,5);
    x_lf(j,29)=data(i-3,5);
    x_lf(j,30)=data(i-2,5);
    x_lf(j,31)=data(i-1,5);
    y_angular(j,1)=data(i,5);
end

% load demo data set (1D inputs for easy visualization-
% this script should work fine for multidimensional inputs)


me_y_angular = mean(y_angular);
y0_angular = y_angular - me_y_angular; % zero mean the data

[N_lf, dim_lf] = size(x_lf);

M_lf = 20; % number of pseudo-inputs

% initialize pseudo-inputs to a random subset of training inputs
[dum_lf, I_lf] = sort(rand(N_lf,1)); 
clear dum_lf;
I_lf = I_lf(1:M_lf);
xb_init = x_lf(I_lf,:);

% initialize hyperparameters sensibly (see spgp_lik for how 
% the hyperparameters are encoded)
hyp_init_lf(1:dim_lf,1) = -2*log((max(x_lf)-min(x_lf))'/2); %log 1/(lengthscales)^2
hyp_init_lf(dim_lf+1,1) = log(var(y0_angular,1)); % log size
hyp_init_lf(dim_lf+2,1) = log(var(y0_angular,1)/4); %log noise

% optimize hyperparameters and pseudo-inputs
w_init_lf = [reshape(xb_init, M_lf*dim_lf,1); hyp_init_lf];
[w_lf,f_lf] = minimize(w_init_lf, 'spgp_lik', -200, y0_angular, x_lf, M_lf);
% [w,f] = lbfgs(w_init,'spgp_lik',200,10,y0,x,M); % an alternative
xb_lf = reshape(w_lf(1:M_lf*dim_lf,1),M_lf,dim_lf);
hyp_lf = w_lf(M_lf*dim_lf+1:end,1);

