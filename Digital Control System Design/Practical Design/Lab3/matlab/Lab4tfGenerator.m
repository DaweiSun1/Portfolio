clear all;
tf500 = tf(500,[1 500]);
tf500D = c2d(tf500,.001,'tustin');
bsingle = single(tf500D.num{1});
asingle = single(tf500D.den{1});
tf500Dsingle = tf(bsingle,asingle,.001);
tf100 = tf(100,[1 100]);
tf100D = c2d(tf100,.001,'tustin');
bsingle = single(tf100D.num{1});
asingle = single(tf100D.den{1});
tf100Dsingle = tf(bsingle,asingle,.001);
tf10 = tf(10,[1 10]);
tf10D = c2d(tf10,.001,'tustin');
bsingle = single(tf10D.num{1});
asingle = single(tf10D.den{1});
tf10Dsingle = tf(bsingle,asingle,.001); 

tf500D4 = c2d(tf500^4,0.001,'tustin');
bsingle = single(tf500D4.num{1});
asingle = single(tf500D4.den{1});
tf500D4single = tf(bsingle,asingle,.001);

tf100D4 = c2d(tf100^4,0.001,'tustin');
bsingle = single(tf100D4.num{1});
asingle = single(tf100D4.den{1});
tf100D4single = tf(bsingle,asingle,.001);

tf10D4 = c2d(tf10^4,0.001,'tustin');
bsingle = single(tf10D4.num{1});
asingle = single(tf10D4.den{1});
tf10D4single = tf(bsingle,asingle,.001);