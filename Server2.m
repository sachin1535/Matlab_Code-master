t = tcpip('localhost',9999);
fopen(t);

x = '123';

writeApiKey = 'Your Write API Key';
data = 42;
data = struct('api_key',writeApiKey,'field1',data);


options = weboptions('MediaType','application/json');
response = webwrite('http://localhost:9999',data,options)
%fwrite(t,data);