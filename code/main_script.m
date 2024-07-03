%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Main_scrip_Robot_Mobile_Project%5%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;
close all;
clc
%%--------------------------------------------------------------------------------------------------%%
%% Parameters
%Beacons position
S=[  0  100;
    100 100;
    100  0 ;
     0   0 ];
L=length(S);
%Tracking Control Parameters
k1=0.5;
k2=1;
b=2;
%Sensor Fution
W=10*diag([1 1 deg2rad(5)].^2);
V=eye(3)*0.01^2;
% V=eye(3);
P0=diag([0.15 0.15 deg2rad(10)].^2);
%Regulation Control Parameters
k1_r=1;
k2_r=10;
k3_r=10;
%% PATH PLANNING
disp('%%%%%%%%%%%%%%Path_Planning%%%%%%%%%%%%%%%%%%%');
disp('Visibility Graph');
disp('Choose de initial position and the goal (Press enter at the end)');
axis_enviroment=[0 100 0 100];
x0_g=enviroment_plot(axis_enviroment);
[points_x,points_y,time_traj]=visible_graph(x0_g(1,:),x0_g(2,:));
N=length(time_traj);
t_sw=time_traj(N-1)+((time_traj(N)-time_traj(N-1))*8/10);
%starting of real robot (deberia sacar cada cierto sampling )
x0=x0_g(1,1)+10;
y0=x0_g(1,2)-10;
theta0=0.001;
REF=sim('reference_signal_in_time');

%% plots

enable_real_plot=1;
enable_measurements_plot=0;
enable_estimate_plot=0;
enable_virtual_plot=0;

X=REF.X_real.signals.values';
Y=REF.X_measure.signals.values';
X_hat=REF.X_prediction.signals.values';
d=REF.d_measure.signals.values';
X_virtual=REF.x_virtual.signals.values';  



if enable_real_plot
        h=figure();
        title('Real Trayectory')
        plot_robot_traj(X,-1,1,[-5 105 -5 105],h);
        hold off
end
if enable_virtual_plot
        h=figure();
        title('Virtual Trayectory')
        plot_robot_traj(X_virtual,-1,1,[-5 105 -5 105],h);
        hold off
end
if enable_estimate_plot
        h=figure();
        title('Estimate Trayectory')
        plot_robot_traj(X_hat,-1,1,[-5 105 -5 105],h);
        hold off
end
if enable_measurements_plot
        h=figure();
        title('Trayectory Measure')
        for k=1:2:size(d,2)
            cla;
            x=Y(:,k);
            y=d(:,k);
            title('Trayectory Measured')
            plot_robot_traj(Y,k,1,[-5 105 -5 105],h);
            plot_sensors_data(x,y,h,S);
            pause(0.1);
        end
        hold off
end




