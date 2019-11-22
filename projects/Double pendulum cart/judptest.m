% udp packet testing with stm32f746zg mbed
clear variables
% global data remoteIP messageReceived

% Port client is expecting to receive on
remotePort = 57185;
% Local port we'll listen on
localPort = 59000;
% Create UDP socket on broadcast IP

judp('SEND', remotePort, '255.255.255.255', [uint8(1) uint8(23) typecast(uint16(localPort), 'uint8')])
[msg, remoteIP] = judp('receive', localPort, 128); 
msg = char(msg')
if ~strcmp(msg, 'ping')
    disp('Wrong ack message received');
    return;
end

timings = zeros(1, 1000);
disp('Waiting 1s');
pause(1);
disp('Starting');
for i = 1:500
    startTime = tic;
%     judp('SEND', remotePort, remoteIP, [uint8(i-1) uint8(rand(1, 39)*255)]);
    msg = judp('receive', localPort, 128);
    timings(i) = toc(startTime);
end

% % % fclose(socket);
disp(['Mean freq: ' num2str(1/mean(timings)) ' Hz'])

