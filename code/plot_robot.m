function[h]=plot_robot(Xk,robot_type,h_fig)
d=3;
if nargin<3
    h=figure();
else
    h=h_fig;
    figure(h);
end

%%P1
%%| d/2 \
%%Xk -d - P3
%%|   /
%%P2

if robot_type==1
    color1=[1 0 0];
    color2=[0 0 1];
elseif robot_type==2
    color1=[0 0 1];
    color2=[1 0 0];
else
    color1=[1 0 1];
    color2=[0 1 0];
end

P1=[Xk(1)+d/2*cos(pi/2+Xk(3)) Xk(2)+d/2*sin(pi/2+Xk(3))];
P2=[Xk(1)+d/2*cos(-pi/2+Xk(3)) Xk(2)+d/2*sin(-pi/2+Xk(3))];
P3=[Xk(1)+d*cos(Xk(3)) Xk(2)+d*sin(Xk(3))];
plot(Xk(1),Xk(2),'o','color',color2,'LineWidth',2);
hold on;
%line([Xk(1) Xk(1)+d*cos(Xk(3))],[Xk(2) Xk(2)+d*sin(Xk(3))],'color','red');
line([P1(1) P3(1)],[P1(2) P3(2)],'color',color1,'LineWidth',2);
line([P1(1) P2(1)],[P1(2) P2(2)],'color',color1,'LineWidth',2);
line([P2(1) P3(1)],[P2(2) P3(2)],'color',color1,'LineWidth',2);
end