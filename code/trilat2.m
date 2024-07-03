function[theta_hat,k_hat]=trilat2(beacons,dhat)
N=size(beacons,1);
Phi=[ones(N,1) -2*beacons(:,1) -2*beacons(:,2)];
b=dhat.^2-beacons(:,1).^2-beacons(:,2).^2;
theta_hat=pinv(Phi)*b;
k_hat=theta_hat(1);
theta_hat=theta_hat(2:3);
end