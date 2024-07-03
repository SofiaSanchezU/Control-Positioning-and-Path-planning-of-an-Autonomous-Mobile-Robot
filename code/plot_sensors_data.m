function[h]=plot_sensors_data(X,y,h,S)
if nargin==2 || isempty(h)
    h=figure();
end
% parameters;
hold on;
plot(S(:,1),S(:,2),'*','LineWidth',2);
for i=1:length(y)
    theta=atan2(X(2)-S(i,2),X(1)-S(i,1));
    plot([S(i,1) S(i,1)+y(i)*cos(theta)],[S(i,2) S(i,2)+y(i)*sin(theta)],'--r','LineWidth',2);
end
end