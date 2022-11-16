pkg load control
close all
clear all
s = tf('s')
n= 34; %Redução
Vp = 6; %Tensão de operação
tal1 = 0.0057686;
tal2 = 0.014998;
i1 = 0.6; %Corrente de operação
i2 = 1.1;
w1 = 17.802;
w2 = 11.519;





%Constante do Motor 
##Ke = Vp/w0;
Kt = (tal1 - tal2)/(i1 - i2);
%Resistencia da Armadura
R = Vp*(w2-w1)/(i1*w2-i2*w1);
Ke = (Vp - R*(i1))/n*(w1)
%G(s) = K/(s(Js+B)) = X/(s(s+Y))
X = 31.63;
Y = 8.357;
K = Kt*n/R;
J = K/X;
B = Y*J;

G = K/(s*(J*s+B));
Iw = 0.0002;
Gw = K/(s*((Iw+J)*s+(B)));



figure
data1 = csvread("Roda.csv");
t1 = data1(:,1)/1000;
x1 = data1(:,2);
plot(t1,x1)
hold on
data1 = csvread("Vazio.csv");
t1 = data1(:,1)/1000;
x1 = data1(:,2);
plot(t1,x1)
step(5.99*Gw,'r',20)
legend("Roda","Vazio","Simulado","fontsize", 12)
xlabel("Tempo(s)","fontsize", 12)
title("Seno em Malha Fechada","fontsize", 18)

figure
data1 = csvread("SenoRoda.csv");
t1 = data1(:,1)/1000;
x1 = data1(:,2);
plot(t1,x1)
hold on
lsim(feedback((5.99/pi)*Gw),(pi)*sin(2*pi*t1/2),t1,'r')
legend("Real","Simulado","Referencia","fontsize", 12)
xlabel("Tempo(s)","fontsize", 12)
title("Seno em Malha Fechada","fontsize", 18)

figure
data1 = csvread("SenoRoda2.csv");
t1 = data1(:,1)/1000;
x1 = data1(:,2);
plot(t1,x1)
hold on
lsim(feedback((5.99/(pi/2))*Gw),(pi/2)*sin(2*pi*t1),t1,'r')
legend("Real","Simulado","Referencia","fontsize", 12)
xlabel("Tempo(s)","fontsize", 12)
title("Seno em Malha Fechada","fontsize", 18)

figure
data1 = csvread("SenoRoda3.csv");
t1 = data1(:,1)/1000;
x1 = data1(:,2);
plot(t1,x1)
hold on
lsim(feedback((5.99/(2*pi))*Gw),(2*pi)*sin(2*pi*t1/3),t1,'r')
legend("Real","Simulado","Referencia","fontsize", 12)
xlabel("Tempo(s)","fontsize", 12)
title("Seno em Malha Fechada","fontsize", 18)
