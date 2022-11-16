pkg load control
close all
clear all

A = (5.99)/(pi/4);
B = (5.99)/(pi);
B1 = (5.99)/(pi/2);
B2 = (5.99)/(2*pi);

data = csvread("Degrau1.csv");

ref = pi/4;

t = data(:,1)/1000;
x = data(:,2);
tp = 0.21;
Mp = (1.112-ref)/(ref);
zeta = -log(Mp)*sqrt(pi^2+(log(Mp))^2)/(pi^2+(log(Mp))^2);
wn = pi/(tp*sqrt(1-zeta^2));

s = tf('s');
Gmf = wn^2/(s^2+2*zeta*wn*s+wn^2);
plot(t,x)
hold on
step((ref)*Gmf,t,'r',4)
line([0 4],[ref ref])
legend("Real","Simulado","Referencia","fontsize", 12)
title("Degrau em Malha Fechada","fontsize", 18)
xlabel("Tempo(s)","fontsize", 12)
H=1;
G = minreal(Gmf/(1-Gmf*H))/A;

figure
data1 = csvread("Degrau2.csv");
t1 = data1(:,1)/1000;
x1 = data1(:,2);
plot(t1,x1)
hold on
step((pi/4)*feedback(4*G),'r',4)
line([0 4],[pi/4 pi/4])
legend("Real","Simulado","Referencia","fontsize", 12)
xlabel("Tempo(s)","fontsize", 12)
title("Degrau em Malha Fechada","fontsize", 18)

figure
data1 = csvread("Degrau3.csv");
t1 = data1(:,1)/1000;
x1 = data1(:,2);
plot(t1,x1)
hold on
step((pi/2)*feedback(2*G),'r',4)
line([0 4],[pi/2 pi/2])
legend("Real","Simulado","Referencia","fontsize", 12)
xlabel("Tempo(s)","fontsize", 12)
title("Degrau em Malha Fechada","fontsize", 18)

figure
data1 = csvread("Degrau4.csv");
t1 = data1(:,1)/1000;
x1 = data1(:,2);
plot(t1,x1)
hold on
step((pi)*feedback(1.5*G),'r',4)
line([0 4],[pi pi])
legend("Real","Simulado","Referencia","fontsize", 12)
xlabel("Tempo(s)","fontsize", 12)
title("Degrau em Malha Fechada","fontsize", 18)

figure
data1 = csvread("Seno1.csv");
t1 = data1(:,1)/1000;
x1 = data1(:,2);
plot(t1,x1)
hold on
lsim(feedback(G*B),(pi)*sin(2*pi*t1/2),t1,'r')
legend("Real","Simulado","Referencia","fontsize", 12)
xlabel("Tempo(s)","fontsize", 12)
title("Seno em Malha Fechada","fontsize", 18)

figure
data1 = csvread("Seno2.csv");
t1 = data1(:,1)/1000;
x1 = data1(:,2);
plot(t1,x1)
hold on
lsim(feedback(G*B1),(pi/2)*sin(2*pi*t1/3),t1,'r')
legend("Real","Simulado","Referencia","fontsize", 12)
xlabel("Tempo(s)","fontsize", 12)
title("Seno em Malha Fechada","fontsize", 18)

figure
data1 = csvread("Seno3.csv");
t1 = data1(:,1)/1000;
x1 = data1(:,2);
plot(t1,x1)
hold on
lsim(feedback(G*B1),(pi/2)*sin(2*pi*t1),t1,'r')
legend("Real","Simulado","Referencia","fontsize", 12)
xlabel("Tempo(s)","fontsize", 12)
title("Seno em Malha Fechada","fontsize", 18)

figure
data1 = csvread("Seno4.csv");
t1 = data1(:,1)/1000;
x1 = data1(:,2);
plot(t1,x1)
hold on
lsim(feedback(G*B2),(2*pi)*sin(2*pi*t1/3),t1,'r')
legend("Real","Simulado","Referencia","fontsize", 12)
xlabel("Tempo(s)","fontsize", 12)
title("Seno em Malha Fechada","fontsize", 18)

figure
data2 = csvread("Rampa1.csv")
t2 = data2(:,1)/1000;
x2 = data2(:,2);
plot(t2,x2)
hold on
lsim(feedback(G),10*t2,t2,'r')
legend("Real","Simulado","Referencia","fontsize", 12)
xlabel("Tempo(s)","fontsize", 12)
title("Rampa em Malha Fechada","fontsize", 18)

figure
data2 = csvread("Rampa2.csv")
t2 = data2(:,1)/1000;
x2 = data2(:,2);
plot(t2,x2)
hold on
lsim(feedback(G),20*t2,t2,'r')
legend("Real","Simulado","Referencia","fontsize", 12)
xlabel("Tempo(s)","fontsize", 12)
title("Rampa em Malha Fechada","fontsize", 18)

figure
data2 = csvread("Rampa3.csv")
t2 = data2(:,1)/1000;
x2 = data2(:,2);
plot(t2,x2)
hold on
lsim(feedback(G),100/15*t2,t2,'r')
legend("Real","Simulado","Referencia","fontsize", 12)
xlabel("Tempo(s)","fontsize", 12)
title("Rampa em Malha Fechada","fontsize", 18)

figure
data1 = csvread("Vazio.csv");
t1 = data1(:,1)/1000;
x1 = data1(:,2);
plot(t1,x1)
hold on
step(5.99*G,'r',20)
legend("Real","Simulado","Referencia","fontsize", 12)
xlabel("Tempo(s)","fontsize", 12)
title("Malha Aberta 5.99V","fontsize", 18)

##figure
##data1 = csvread("Roda.csv");
##t1 = data1(:,1)/1000;
##x1 = data1(:,2);
##plot(t1,x1)
##hold on
##step(5*G,'r',10)
##legend("Real","Simulado","Referencia","fontsize", 12)
##xlabel("Tempo(s)","fontsize", 12)
##title("Seno em Malha Fechada","fontsize", 18)
