%EXAMPLE m-FILE
%Set up transfer function
num=[60];
den=[.3 1];
sys=tf(num,den)
%Now use c2d to map your transfer function to the z-domain
sysz1=c2d(sys,.00001,'zoh')
%Now add a plant numerical error by adding .009 to your discrete model.
sysz1.den{1} = sysz1.den{1} + [0 0.009]
%Now use d2c to map your transfer function back to the s-domain
sysc1=d2c(sysz1)

bode(sys,sysc1)