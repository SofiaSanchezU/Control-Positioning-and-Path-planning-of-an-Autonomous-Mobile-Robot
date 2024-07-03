function [x0_g] = enviroment_plot(axis_enviroment)
figure(1)
title('Enviroment')
rectangle('Position',[20 20 15 10],'Curvature',1 , 'FaceColor',[0.3010 0.7450 0.9330])
axis(axis_enviroment)
axis square
rectangle('Position',[60 10 35 35], 'Curvature',[1 1] , 'FaceColor',[0.3010 0.7450 0.9330])
P = [15 50; 25 60; 30 85; 25 95; 15 95];
pgon = polyshape(P);
P1 = [45 65; 55 65; 55 60; 65 60; 65 75; 45 75];
pgon1 = polyshape(P1);
hold on
plot(pgon,'FaceColor',[0.2010 0.5450 0.9]) 
plot(pgon1,'FaceColor',[0.2010 0.5450 0.9])
x0_g=ginput;
hold off
end

