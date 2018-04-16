function dx =SDOFC(t,x,tv,yv,A,C,K)
K = transpose(K);

y = interp1(tv,yv,t)'

 dx = (A-K*C)*x + K*y;
end