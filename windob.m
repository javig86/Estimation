function dx = windob(t,x,A,C,K,a,ft)
K = transpose(K);
w = a*sin(2*pi*ft*t)+.5;

% Gamma = [1;0;0;0;0;0];
Abar = [A, zeros(length(A)); K*C, A-K*C];
dx =Abar*x+ K*y %Gamma*w;
end
