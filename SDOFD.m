function [dx]=SDOFD(t,x,A,E,a,fr)
w = a*sin(2*pi*fr*t);
dx = A*x + E*w;