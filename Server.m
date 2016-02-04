

t = tcpip('dev.mirrorworlds.icat.vt.edu',9999);
fopen(t);

x = '{' + '"id": 0' +'"origin: {" + '}';



while (1)
fwrite(t,x);
%delay(1);
end