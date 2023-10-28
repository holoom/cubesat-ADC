clc,clear all,close all
%% Initializtion
% a = instrhwinfo('Bluetooth') ;
% delete(instrfind) ;

aerospaceBluetooth = bluetooth('98D341FD55DE',1) ;
time = 0 ;
i = 1 ;
i_old = 1 ;
n = 0 ;

% Control Variables 
flag =1 ;
control_flag=0  ;  % control_flag =0 --> angle control , control_flag =1 --> angular velocity control 
Angle_dg =90 ;
des_omega=0;
kp =2000 ;
ki =0 ;
kd =0 ;

while 1
 %Sending flags and gains to arduino    
    write(aerospaceBluetooth,[flag,control_flag],'int8')
    write(aerospaceBluetooth,[des_omega,Angle_dg,kp,ki,kd],'single')

 %Reading Data from ardunio 
    readData1 = read(aerospaceBluetooth,5,'single') ;
    readData2 = read(aerospaceBluetooth,1,'int32') ;
    
%
    time(i) = readData1(1) ;
    omega(i) = readData1(2) ;
    inn(i) = readData1(3);
    desired(i)= readData1(4);
    angle(i) = readData1(5) ;
    pwm(i) = readData2(1) ;   
    
    
    if i == 1
        l = plot(time,angle) ;
        l.XDataSource = 'time' ;
        l.YDataSource = 'angle' ;
    end
    
    if i - i_old>=5
        fprintf('time = %.5f, angle = %.5f, omega = %.5f , PWM = %d ,inn= %.5f , Desired Andle =%.5f \n',time(i),angle(i),omega(i),pwm(i),inn(i),desired(i)) ;        
       % fprintf(flag , control_flag , Angle_dg , kp , ki , kd ) ;
        i_old = i ;
        refreshdata(l,'caller') ;
        drawnow;
    end
    i = i + 1 ;    
end