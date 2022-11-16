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

M = 0
T = 0.002; %Tempo de aquisição
stname = {'AngMotor', 'Theta', 'VelMotor','dTheta'};
sys = ss(A,B,C,'stname',stname); %Sistema Continuo;
x0 = [0;0.15;0;0];%Estado Inicial
R = 0.08;
Q = diag([0.0000000001,100,0,0.5]);
tempo = 5;

k = lqr(A,B,Q,R)
sysC = ss(A-B*k,B*k(2),C); %Sistema Continuo com controle
x0 = [0;0.15;0;0];
[y,t,x] = initial(sysC, x0,tempo);
u = -(k(1)*x(:,1)+k(2)*x(:,2)+k(3)*x(:,3)+k(4)*x(:,4));

figure
impulse(tf(sys)(2,1))
delete (findobj ("tag", "legend"))
xlabel("Tempo(s)")
ylabel("Ângulo(rad)")
title("")
print('Impulso.svg','-dsvg')

sysD = c2d(sys,T); %Sistema Discreto
Ad = sysD.a;
Bd = sysD.b;


F = Q;
for i =1:100000
  W = R + Bd'*F*Bd;
  P = F - F*Bd*(W^-1)*Bd'*F;
  F = Ad'*P*Ad+Q;
endfor
k1 = (W^-1)*Bd'*F*Ad;

##k = [0.99225       50.704       1.5073       23.721];

sysDC = ss(Ad-Bd*k1,Bd*k1(2),sys.c,0,T,'stname',stname); %Sistema Discreto com Controle
[y1, t1, x1] = initial(sysDC, x0,tempo);
u1 = -(k1(1)*x1(:,1)+k1(2)*x1(:,2)+k1(3)*x1(:,3)+k1(4)*x1(:,4));%Ação de Controle


##%Observador de Ordem Reduzida
Amm = Ad(1:2,1:2);
Amo = Ad(1:2,3:4);
Aom = Ad(3:4,1:2);
Aoo = Ad(3:4,3:4);
Bm = Bd(1:2);
Bo = Bd(3:4);


A1o = Aoo;
C1o = Amo;


tal = [0 0 0 0]; %Constante de Tempo
tal(1) = -T/log(eig(Ad-Bd*k1)(1));
tal(2) = -T/log(eig(Ad-Bd*k1)(2));
tal(3) = -T/log(eig(Ad-Bd*k1)(3));
tal(4) = -T/log(eig(Ad-Bd*k1)(4));
tal_menor = min(tal);
p4 = exp(-T/(tal_menor/2));
talp4 = -T/log(p4);
p = [p4 p4];
L = place(A1o',C1o',p)';

%Simulação do Observador
x2 = x0;
xO = zeros(2,tempo/T);
uO = zeros(1,tempo/T);
uO(1) = -(k1(1)*x'(1,1)+k1(2)*x'(2,1)+k1(3)*xO(1,1)+k1(4)*xO(2,1));
if(u(1)>(5.99))
uO(1) = (5.99);
endif
if(uO(1)<-(5.99))
uO(1) = -(5.99);
endif
for i = 2:(tempo/T+1)
x2(:,i) = Ad*x2(:,i-1) + Bd*uO(i-1);
y2(:,i) = C*x2(:,i); %% sensores
##y2(:,i) = y2(:,i) + [rand()*0.002;rand()*0.005];

xO(:,i) = (Aoo-L*Amo)*xO(:,i-1)+(Bo-L*Bm)*uO(:,i-1)+L*y2(:,i)+(Aom-L*Amm)*(y2(:,i-1));
uO(i) = -(k1(1)*y2(1,i)+k1(2)*y2(2,i)+k1(3)*xO(1,i)+k1(4)*xO(2,i));

if(uO(i)>(5.99))
uO(i) = (5.99);
endif
if(uO(i)<-(5.99))
uO(i) = -(5.99);
endif
%Saturação do Motor


endfor

figure
[ax,h1,h2] = plotyy(t,y(:,2),t,u);
set(ax(1),'YLim',[-0.2 0.2])
set(ax(2),'YLim',[-10 10])
legend("Angulo do Robo","Ação de Control")

figure
plot(t,u)
hold on
plot(t1,u1)
plot(t1,uO)
title("Ações de Controle")
grid on
xlabel("Tempo(s)")
ylabel("Tensão(V)")
legend("Continuo","Discreto","Discreto com Observador")


figure
plot(t,x(:,2))
hold on
plot(t1,x1(:,2))
plot(t1,x2(2,:))
title("Angulo do Robo")
grid on
xlabel("Tempo(s)")
ylabel("Radianos")
legend("Continuo","Discreto","Discreto com Observador")

figure
subplot(2,2,1)
plot(t,x(:,1),t1,x1(:,1),t1,x2(1,:))
title("AngMotor")
legend("Continuo","Discreto","Discreto com Observador")
grid on
subplot(2,2,2)
plot(t,x(:,2),t1,x1(:,2),t1,x2(2,:))
grid on
title("Theta")
legend("Continuo","Discreto","Discreto com Observador")
subplot(2,2,3)
plot(t,x(:,3),t1,x1(:,3),t1,x2(3,:))
grid on
title("VelMotor")
legend("Continuo","Discreto","Discreto com Observador")
subplot(2,2,4)
plot(t,x(:,4),t1,x1(:,4),t1,x2(4,:))
grid on
title("dTheta")
legend("Continuo","Discreto","Discreto com Observador")

figure
subplot(2,1,1)
plot(t1,xO(1,:),t1,x2(3,:))
title("VelMotor")
legend("Observado","Real")
subplot(2,1,2)
plot(t1,xO(2,:),t1,x2(4,:))
title("dTheta")
legend("Observado","Real")

Aoc = (Aoo-L*Amo)
Boc=(Bo-L*Bm)
L1c = L
L2c = Aom-L*Amm



##data3 = csvread("AbertaV3.csv");
##data4 = csvread("AbertaV4.csv");
##data5 = csvread("AbertaV5.csv");
##a = 242
##t3R = data3(1:a,1)/1000;
##y3R = data3(1:a,2:3);
##a = 245
##t4R = data4(1:a,1)/1000;
##y4R = data4(1:a,2:3);
##a = 206
##t5R = data5(1:a,1)/1000;
##y5R = data5(1:a,2:3);


##subplot(3,1,1)
##plot(t3R,y3R(:,2))
##x0R = [0;y3R(1,2);0;0];
##u = 3.3868*ones(length(t3R),1);
##hold on
##[y2, t2, x2] = lsim(sys,u,t3R,x0R)
##plot(t2,y2(:,2))
##title("V = 3")
##legend("Real","Simulado")
##grid on
##subplot(3,1,2)
##plot(t4R,y4R(:,2))
##x0R = [0;y4R(1,2);0;0];
##u = 4.3868*ones(length(t4R),1);
##hold on
##[y2, t2, x2] = lsim(sys,u,t4R,x0R)
##plot(t2,y2(:,2))
##grid on
##title("V = 4")
##legend("Real","Simulado")
##subplot(3,1,3)
##plot(t5R,y5R(:,2))
##x0R = [0;y5R(1,2);0;0];
##u = 5.3868*ones(length(t5R),1);
##hold on
##[y2, t2, x2] = lsim(sys,u,t5R,x0R)
##plot(t2,y2(:,2))
##grid on
##title("V = 5")
##legend("Real","Simulado")
##
##figure
##subplot(3,1,1)
##plot(t3R,-y3R(:,1))
##x0R = [0;y3R(1,2);0;0];
##u = 3.3868*ones(length(t3R),1);
##hold on
##[y2, t2, x2] = lsim(sys,u,t3R,x0R)
##plot(t2,y2(:,1))
##title("V = 3")
##legend("Real","Simulado")
##grid on
##subplot(3,1,2)
##plot(t4R,-y4R(:,1))
##x0R = [0;y4R(1,2);0;0];
##u = 4.3868*ones(length(t4R),1);
##hold on
##[y2, t2, x2] = lsim(sys,u,t4R,x0R)
##plot(t2,y2(:,1))
##grid on
##title("V = 4")
##legend("Real","Simulado")
##subplot(3,1,3)
##plot(t5R,-y5R(:,1))
##x0R = [0;y5R(1,2);0;0];
##u = 5.3868*ones(length(t5R),1);
##hold on
##[y2, t2, x2] = lsim(sys,u,t5R,x0R)
##plot(t2,y2(:,1))
##grid on
##title("V = 5")
##legend("Real","Simulado")

data = csvread("TesteV2.csv");
tR = data(:,1)/1000;
yR = data(:,2:3);
xoR = data(:,4:5);
uRR = data(:,6);
for i = 1:(size(uRR))
if(uRR(i)>(5.99))
uRR(i) = (5.99);
endif
if(uRR(i)<-(5.99))
uRR(i) = -(5.99);
endif
endfor


figure
[ax,h1,h2] = plotyy(tR,yR(:,2),tR,uRR);
legend("Angulo do Robo","Ação de Control")

figure
subplot(2,2,1)
plot(tR,yR(:,1))
title("AngMotor")
grid on
subplot(2,2,2)
plot(tR,yR(:,2))
grid on
title("Theta")
subplot(2,2,3)
plot(tR,xoR(:,1))
grid on
title("VelMotor")
subplot(2,2,4)
plot(tR,xoR(:,2))
grid on
title("dTheta")