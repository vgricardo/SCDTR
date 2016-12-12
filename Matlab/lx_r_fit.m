Rmin = 3000;
Rmax = 40000;

lxmin = 2;
lxmax = 80;

logrmin = log10(Rmin);
logrmax = log10(Rmax);
loglxmin = log10(lxmin);
loglxmax = log10(lxmax);

delta = -(logrmax-logrmin)/(loglxmax-loglxmin);
b = logrmax - delta*loglxmin;
a = 10^b;
c = -1/delta;

o = [0:1:255];

vin = i./1023.*5;

R = 50000./vin-10000;
I = (a./R).^c;


plot(o,I)