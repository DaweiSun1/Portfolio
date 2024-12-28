Vz = tf([0.169635], [1 -0.997287],0.001);

Vs = d2c(Vz);
IntS = d2c(tf([0.001],[1 -1],0.001));

Ps = Vs*IntS;
Pz = c2d(Ps,0.001);

contDeriv = tf([500 0], [1 500]);
discDeriv = c2d(contDeriv,0.001, "tustin");
bode(tf([1 0],1),contDeriv, discDeriv)
legend("$s$", "$\frac{500s}{s+500}$","discrete","Interpreter","latex")