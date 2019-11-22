% Ethernet testing
% Listen on all available Ethernet interfaces at local port 8000.
% Specify a LocalHost (host name or IP address) if known
% #define HOST_IP			"192.168.0.20"
% #define HOST_PORT		1884
instrreset 
% u = udp('', 'LocalHost', '', 'LocalPort', 8000);
u = udp('', 'LocalHost', '192.168.0.20', 'LocalPort', 1884);
fopen(u);
% Receive a single UDP packet
% u.Status
t0 = tic;
while toc(t0) < 5
    packetData = fscanf(u);
end
% Clean up
fclose(u);
delete(u);
clear u
