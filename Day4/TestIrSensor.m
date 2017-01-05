%% Test irSensor

A = load('log.txt')

k = [16,10];
d = linspace(0,100,size(A,1));

fun = irsensor(k,d);

lsqcurvefit(fun, k, d, A(:,1))


