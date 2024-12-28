data = serialreadSPIRAM("COM5");
time = data(:,1);
omega = data(:,2);
u = data(:,3);

figure;
subplot(2,1,1);
plot(t,omega);
ylabel("$\frac{rad}{s}$","Interpreter","latex");
legend("$\omega$","Interpreter","latex");

subplot(2,1,2);
plot(t,u);
xlabel("$t$", "Interpreter","latex");
legend("$u$","Interpreter","latex");