n = 6;
N = n^2;
[txs, tys] = meshgrid(0:(n-1),0:(n-1));    % target position
tpositions = [reshape(txs, 1, N); reshape(tys, 1, N)];
uavpositions = (n-1)*2*rand(2, N) - (n-1)/2;


a = UAVdata(N, tpositions, uavpositions, [0 0], [5*n 5*n]);
a = a.run
delete(a)
clear a

