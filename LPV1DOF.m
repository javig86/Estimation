clear all; clc; close all;
%% LPV 1DOF Particle Example


%% Parameters
Xu = -.83;
m  = 1;

%% State Space Model
A = [0 1;0 Xu/m];
B = [0;1/m];
C = [1 0];
D = 0;

%% Observability Analysis
Om = obsv(A,C);

%% simulation 
a = 1;
f = 15/30;
dt = 1/10;
tv = 0:dt:180-dt;
u = a*sin(2*pi*f*tv);
[tv1,Y1]=ode45(@(t,x) SDOF(t,x,A,B,f,a),tv,[0,3]);
%% Controller
Qc = [1,0;0,1];
Rc = 1/10;
[Kc,Sc,Ec]=lqr(A,B,Qc,Rc);
%% Stabilized Model
Ac = A-B*Kc;
Cc = eye(2);
Ec = [1;0];

a = 1;
fr = 10/60;
w = a*sin(2*pi*fr*tv);
[T,X]=ode45(@(t,x) SDOFD(t,x,Ac,Ec,a,fr),tv,[0 0]);

figure
subplot(3,1,1)
plot(T,X(:,1),'-','LineWidth',2)
ylabel('Position')
ylim([-3 3])
grid minor
subplot(3,1,2)
plot(T,X(:,2),'-','LineWidth',2)
ylabel('Speed')
ylim([-3 3])
grid minor
subplot(3,1,3)
plot(tv,w,'-',tv,gradient(X(:,1),0.1)-X(:,2),'--','LineWidth',2)
ylabel('Wind Disturbance')
ylim([-3 3])
grid minor

%%
Aa = [Ac,[1;0];[0 0 0]];
Ca = [1 0 0;0 1 1];

%% Observability
Oma = obsv(Aa,Ca);
if rank(Oma)==length(Aa)
    disp('system is observable!')
else
    disp('system is not observable!')
end

%% Observer Design
Qa = eye(3);
Ra = eye(2)/10;
[Ka,Sa,Ea]=lqr(Aa',Ca',Qa,Ra);

xa0 = zeros(1,3)

[T2,Xh] = ode45(@(t,x) SDOFC(t,x,tv,[X(:,1),gradient(X(:,1),0.1)],Aa,Ca,Ka),tv,[0,0,0])


% [ta,Ya]=ode45(@(t,x) windob(t,x,Aa,Ca,Ka,a,fr),tv,xa0);
%%
figure('rend','painters','pos',[200 70 800 500])
subplot(3,1,1)
plot(T2,Xh(:,1),'LineWidth',1.5)
ylabel('Position [m]','Interpreter','Latex','FontSize',12)
ylim([-2 2])
grid minor
subplot(3,1,2)
plot(T2,Xh(:,2),'LineWidth',1.5)
ylabel('Speed [m/s]','Interpreter','Latex','FontSize',12)
ylim([-2 2])
grid minor
subplot(3,1,3)
plot(T2,w,T2,Xh(:,3),'-.','LineWidth',1.5)
xlabel('Time [sec]','Interpreter','Latex','FontSize',12)
ylabel('Wind Speed [m/s]','Interpreter','Latex','FontSize',12)
ylim([-2 2])
grid minor
lg = legend('Actual', 'Estimate','Location',[.441 .306 .7 0] );
lg.Interpreter = 'Latex';
lg.Orientation =  'Horizontal';


%% 
% E = [0;Xu/m]*1;
% fr = 0.01;
% amp = 0.25;
% [T1,Y1] = ode45(@(t,x) SDOFB(t,x,Ao,E,amp,fr),tv, [0 0])
% [T2,Y2] = ode45(@(t,x) SDOFC(t,x,tv,Y1,Aa,Ca,Ka),tv,[0,0,0])
% 
% figure
% plot(T1,Y1)
% grid minor
%%
% figure
% subplot(3,1,1)
% plot(T2,Y2(:,1))
% grid minor
% subplot(3,1,2)
% plot(T2,Y2(:,2))
% grid minor
% subplot(3,1,3)
% plot(tv,amp*sin(2*pi*fr*tv),T2,Y2(:,3))
% legend('Wind Disturbance','Wind Estimate')
% xlabel('Time')
% grid minor
