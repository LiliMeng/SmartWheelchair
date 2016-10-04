clear all;
close all;

%data_raw=load('/home/lci/workspace/odometry/data_preparation/short_velocity.txt')
%data_interp=load('/home/lci/workspace/odometry/data_preparation/short_interp_vel.txt')
data_raw=load('/home/lci/workspace/odometry/data_preparation/processed_velocity.txt')
data_interp=load('/home/lci/workspace/odometry/data_preparation/interpolated_velocity.txt')


timestamp_raw=data_raw(:,1);
vel_raw=data_raw(:,2);
figure
plot(timestamp_raw, vel_raw, '*-');
hold on;



timestamp_interp=data_interp(:,1);
vel_interp=data_interp(:,2);
%plot(timestamp_interp,vel_interp, 'o-');
hold on;
p_xLinearVelocity = pchip(timestamp_raw, vel_raw, timestamp_interp);
linear_interp_vel = interp1q(timestamp_raw, vel_raw, timestamp_interp);
%plot(timestamp_interp, p_xLinearVelocity, 'o-');
%hold on;
plot(timestamp_interp, linear_interp_vel, 'o-');

t=title('Visual odometry linear velocity with the timestamp');
le=legend('interpolated data(16Hz)','raw data(20~27Hz)');
set(le, 'FontSize', 16);
set(t, 'FontSize', 20);
xl=xlabel('timestamp(s)');
yl=ylabel('visual odometry linear velocity(m/s)')
set(xl, 'FontSize', 18);
set(yl, 'FontSize', 18);


