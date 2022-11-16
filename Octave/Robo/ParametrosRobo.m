pkg load control
clear all
close all
format short g

%Offset = 1.48
%Motor sem offset
K =  0.2109;
J = 6.6691e-03;
B = 0.055734;
%Motor com offset
##K = 0.5906;
##J = 6.2127e-03;
##B = 0.082566;

%Robo
mb= 1.013;%massa do corpo do robo(kg)
Ib= 0.11;%Momento de Inercia do Corpo## 0.12 
l=  0.09;%Distancia entre o centro de rotação e o centro de massa
%Roda
mw= 0.070;%massa da roda(kg)
Iw= 0.0002;%Momento de Inercia da Roda## 0.0002
r= 0.04; %Raio da Roda
bw= 0.03;#### 0.03
%Gravidade
g=9.80665;


s = tf('s');

E = [(mb+2*mw)*r^2+2*Iw , mb*r*(l+r)+2*mw*r^2+2*Iw;
      mb*l*r  , mb*l*(l+r)+Ib];
      
F = [ 2*B+2*bw , 2*bw;
      -2*B ,  0];
      
G = [0 , 0;
     0 , -mb*g*l];

H = [2*K;
     -2*K];
     
A1=(-E^(-1))*G;
A2=(-E^(-1))*F;
B1=(-E^(-1))*H;

A = [ 0, 0, 1, 0;
      0, 0, 0, 1;
      A1,A2];
B = [0;0;B1];
C = [1,0,0,0;0,1,0,0];

T = 0.002; %Tempo de aquisição
stname = {'AngMotor', 'Theta', 'VelMotor','dTheta'};
sys = ss(A,B,C,'stname',stname); %Sistema Continuo;

data3 = csvread("V3.csv");
data4 = csvread("V4.csv");
data5 = csvread("V5.csv");
data6 = csvread("V599.csv");
a = 310
t3R = data3(1:a,1)/1000;
y3R = data3(1:a,2:3);
a = 208
t4R = data4(1:a,1)/1000;
y4R = data4(1:a,2:3);
a = 197
t5R = data5(1:a,1)/1000;
y5R = data5(1:a,2:3);
a = 156
t6R = data6(1:a,1)/1000;
y6R = data6(1:a,2:3);

subplot(4,1,1)
plot(t3R,y3R(:,2))
x0R = [0;y3R(1,2);0;0];
u = 3*ones(length(t3R),1);
hold on
[y2, t2, x2] = lsim(sys,u,t3R,x0R)
plot(t2,y2(:,2))
title("u = 3V")
legend("Real","Simulado")
grid on
subplot(4,1,2)
plot(t4R,y4R(:,2))
x0R = [0;y4R(1,2);0;0];
u = 4*ones(length(t4R),1);
hold on
[y2, t2, x2] = lsim(sys,u,t4R,x0R)
plot(t2,y2(:,2))
grid on
title("u = 4V")
legend("Real","Simulado")
subplot(4,1,3)
plot(t5R,y5R(:,2))
x0R = [0;y5R(1,2);0;0];
u = 5*ones(length(t5R),1);
hold on
[y2, t2, x2] = lsim(sys,u,t5R,x0R)
plot(t2,y2(:,2))
grid on
title("u = 5V")
legend("Real","Simulado")
subplot(4,1,4)
plot(t6R,y6R(:,2))
x0R = [0;y6R(1,2);0;0];
u = 5.99*ones(length(t6R),1);
hold on
[y2, t2, x2] = lsim(sys,u,t6R,x0R)
plot(t2,y2(:,2))
grid on
title("u = 5.99V")
legend("Real","Simulado")

figure
subplot(4,1,1)
plot(t3R,-y3R(:,1))
x0R = [0;y3R(1,2);0;0];
u = 3*ones(length(t3R),1);
hold on
[y2, t2, x2] = lsim(sys,u,t3R,x0R)
plot(t2,y2(:,1))
title("u = 3V")
legend("Real","Simulado")
grid on
subplot(4,1,2)
plot(t4R,-y4R(:,1))
x0R = [0;y4R(1,2);0;0];
u = 4*ones(length(t4R),1);
hold on
[y2, t2, x2] = lsim(sys,u,t4R,x0R)
plot(t2,y2(:,1))
grid on
title("u = 4V")
legend("Real","Simulado")
subplot(4,1,3)
plot(t5R,-y5R(:,1))
x0R = [0;y5R(1,2);0;0];
u = 5*ones(length(t5R),1);
hold on
[y2, t2, x2] = lsim(sys,u,t5R,x0R)
plot(t2,y2(:,1))
grid on
title("u = 5V")
legend("Real","Simulado")
subplot(4,1,4)
plot(t6R,-y6R(:,1))
x0R = [0;y6R(1,2);0;0];
u = 5.99*ones(length(t6R),1);
hold on
[y2, t2, x2] = lsim(sys,u,t6R,x0R)
plot(t2,y2(:,1))
grid on
title("u = 5.99V")
legend("Real","Simulado")