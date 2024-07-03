function[theta]=enrollTheta(theta)
for i=1:length(theta)
    
    theta(i)=mod(theta(i),2*pi);
    if theta(i)>pi
        theta(i)=theta(i)-2*pi;
    end
    
%     while theta(i)>pi
%         theta(i)=theta(i)-2*pi;
%     end
%     while theta(i)<-pi
%         theta(i)=theta(i)+2*pi;
%     end
end