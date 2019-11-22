% udp packet testing with stm32f746zg mbed
clear variables
global data remoteIP messageReceived

% Port client is expecting to receive on
remotePort = 57185;
% Local port we'll listen on
localPort = 59000;
% Create UDP socket on broadcast IP
socket = udp('255.255.255.255', remotePort, 'LocalPort', localPort);
% Create callback function to receive packets
socket.DatagramReceivedFcn = @readDatagram;
% Open socket
fopen(socket);
% Send ID packet to client
fwrite(socket, [uint8(1) uint8(23) typecast(uint16(localPort), 'uint8')])
% Wait for reply with correct signature
while(~strcmp(data, 'ping'))
end

% Change socket remoteIP
socket.RemoteHost = remoteIP
socket.DatagramReceivedFcn = @readDatagram;

% % Close socket and reopen with client IP (not broadcast IP)
% fclose(socket);
% socket = udp(remoteIP, remotePort, 'LocalPort', localPort);
% socket.DatagramReceivedFcn = @readDatagram;
% fopen(socket);

timings = zeros(1, 1000);
messageReceived = false;
disp('Waiting 1s');
pause(1);
disp('Starting');
for i = 1:20
    startTime = tic;
    fwrite(socket, [uint8(i-1) uint8(rand(1, 20)*255)]);
    while(~messageReceived)
        if toc(startTime) > 5
            disp('Timeout, closing socket');
            fclose(socket); 
            return;
        end
    end
    messageReceived = false;
%     toc(startTime);
    timings(i) = toc(startTime);
%     pause(1);
end

fclose(socket);
disp(['Mean freq: ' num2str(1/mean(timings)) ' Hz'])

