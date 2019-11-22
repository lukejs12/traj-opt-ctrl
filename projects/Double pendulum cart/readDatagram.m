function readDatagram(obj, ~)
    global data remoteIP messageReceived

%     disp('Datagram received');
    [data, ~, ~, remoteIP, ~] = fscanf(obj);
    messageReceived = true;
%     remoteIP = obj.DatagramAddress;
    
end