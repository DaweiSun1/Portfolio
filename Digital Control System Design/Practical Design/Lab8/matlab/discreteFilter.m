ctFilter = tf(20, [1 20])^6;
filter = c2d(ctFilter,0.001, 'tustin');

arraytoCformat_double(filter.den{1})