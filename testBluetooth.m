clc,clear all,close all
%% Initializtion
% a = instrhwinfo('Bluetooth') ;
delete(instrfind) ;
aerospaceBluetooth = Bluetooth('btspp://98D341FD55DE',1) ;
fopen(aerospaceBluetooth)
t = 0 ;
i = 1 ;
fwrite(aerospaceBluetooth,1,'int8')
while 1
%     fwrite(aerospaceBluetooth,1,'int8')

    readData = fread(aerospaceBluetooth,[1,2],'float') ;
    t(i) = readData(1) ;
    y(i) = readData(2) ;
    fprintf('time = %.5f\n',t(i)) ;
    i = i + 1 ;
end
plot(t,y)