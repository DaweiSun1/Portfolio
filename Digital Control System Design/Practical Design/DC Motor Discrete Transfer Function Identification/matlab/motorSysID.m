format long
Ts=5e-3;
data = data5ms;

t = data(:,1);
v = data(:,2);
u = data(:,3);
u(1)=0;

plot(t,v,t,u)
legend('$\omega_m(t)$','$u(t)$','Interpreter','latex')


i=1;
T=0;
while v(i)<300*0.63
    i=i+1;
end
T = t(i);
K = v(length(v))/u(length(u));

% 1st order TF
b = v(2:length(v));
A = [v(1:length(v)-1) u(1:length(u)-1)];
c1 = A\b;

TF1 = tf(c1(2), [1 -c1(1)], 0.005);

Td = -Ts/log(c1(1));
Kd = c1(2)/(1-c1(1));
% 
% K
% Kd
% T
% Td
% c

% 3rd order tf

b = v(4:length(v));
A = [u(3:length(u)-1) u(2:length(u)-2) u(1:length(u)-3) -v(3:length(v)-1) -v(2:length(v)-2) -v(1:length(v)-3)];
c3 = A\b;
TF3 = tf(c3(1:3)',[1 c3(4:6)'], 0.005);
bode(TF1,TF3)