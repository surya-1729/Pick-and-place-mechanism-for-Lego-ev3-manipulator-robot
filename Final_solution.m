clc;clear;
% intialising Robot
 myev3 = legoev3('USB');
 
%Sending Robot to Limit Switches
homing(myev3)
 
%Performing Tasks
pick('c', myev3)  %position and robot env variable
place('a',myev3)
pick('a', myev3)
place('b',myev3)
pick('b', myev3)
place('c',myev3)


%Definition of bevaiour 
function pick(p, myev3)
inverse_kinematics(p,myev3, 1)  %pick from Position 
end


function place(p, myev3)
inverse_kinematics(p,myev3, 0)  %place at Position 
end

%Function for Inverse Kinematics
 function inverse_kinematics(p,myev3,pick)
l0 = 70; l1= 50; l2= 95; l3 = 185; l4 = 110;    %Robot Links length

GearRatioA = 12/36;                             %Gear Ratio for Motors
GearRatioB = 8/40;
theta2 = pi/4;
theta0 = 110*pi/180;                            % theta3 at switch point

motorA = motor(myev3, 'C');
motorB = motor(myev3, 'B');
motorC = motor(myev3, 'A');
 
rotationA = readRotation(motorA);
rotationB = readRotation(motorB);
  
 %x,y,z is in table's frame. Note: Robot has only two degree of freedom
 switch p 
     case 'a'
        x = 115;             % range of x is -116 to 116
        y = 0;                % same as x
        z = -120;              % - 229 to 0, o at table negavtive upwards
      case 'b'
        x = 0;             % range of x is -116 to 116
        y = -115;                % same as x
        z = 0;              % - 229 to 0, o at table negavtive upwards
      case 'c'
        x = -115;             % range of x is -116 to 116
        y = 0;                % same as x
        z = 0;              % - 229 to 0, o at table negavtive upwards
 end
  
%inverse kinematics
theta1 = atan (y/x);
if (x<0)
    theta1 = -(pi - theta1);
end
theta3 = asin((-z+l4-l0 - l1 - l2*sin(theta2))/(l3))+pi/4;

disp(theta1);
disp(theta3);

%PID values
C1_A= 2;
C2_A= 0.5; 
C3_A = 0.1;

C1_B= 3.5;
C2_B= 0.5; 
C3_B = 1.5;

%Error for motor A
errorA = (180/pi)*theta1 - rotationA*GearRatioA;
errorA_k1 =0;
errorA_k2 = 0;

z2 = -140;
theta3_2 = asin((-z2+l4-l0 - l1 - l2*sin(theta2))/(l3))+pi/4;

Tol = 3;             %Tolerance for error

%stop criteria
stopA = abs(errorA);


%Opening the arm if picking
if (pick == 1)
    motorC.Speed = 10;
    start(motorC);
    pause(0.2);
    stop(motorC)
end
    
 while(stopA > Tol)
    rotationA = readRotation(motorA);
    errorA = (180/pi)*theta1 - rotationA*GearRatioA;
    motorA.Speed = (C1_A * errorA + C2_A * errorA_k1 + C3_A * errorA_k2);
    
    disp(errorA);
    
    if (abs(motorA.Speed) > 15)
            motorA.Speed = 15 * motorA.Speed/abs(motorA.Speed);
    end
    
    if (abs(motorA.Speed) < 10)
         motorA.Speed = 10 * motorA.Speed/abs(motorA.Speed);
    end
        
    
    start(motorA);
    errorA_k2 = errorA_k1;
    errorA_k1 = errorA;  
    stopA = abs(errorA);
    pause (100/1000) %sampling time
 end 
 stop(motorA);
 
 %error of motor B
errorB = (180/pi)*(theta0-theta3) - rotationB*GearRatioB;
errorB_k1 =0;
errorB_k2 = 0;
stopB = abs(errorB);

 while (stopB > Tol)
     rotationB = readRotation(motorB);
     errorB = (180/pi)*(theta0-theta3) - rotationB*GearRatioB;
    motorB.Speed = (C1_B * errorB + C2_B * errorB_k1 + C3_B * errorB_k2);
    
    if (abs(motorB.Speed) > 30)
        motorB.Speed = 30 * motorB.Speed/abs(motorB.Speed);
    end
    
    if (abs(motorB.Speed) < 15)
         motorB.Speed = 15 * motorB.Speed/abs(motorB.Speed);
    end
    
    start(motorB);
    
    errorB_k2 = errorB_k1;
    errorB_k1 = errorB;
    stopB = abs(errorB);
    pause (100/1000) %sampling time
 end
stop(motorB);

%motor C motion
if (pick == 1)
    motorC.Speed = 10;
    start(motorC);
    pause(0.5);
    stop(motorC) 
else 
    motorC.Speed = -10; 
    start(motorC);
    pause(0.2);
    stop(motorC);
    
end

errorB = (180/pi)*(theta0-theta3_2) - rotationB*GearRatioB;
errorB_k1 =0;
errorB_k2 = 0;
stopB = abs(errorB) + abs(errorB_k1) + abs(errorB_k2);

while (stopB > Tol)
    rotationB = readRotation(motorB);
    errorB = (180/pi)*(theta0-theta3_2) - rotationB*GearRatioB;
    motorB.Speed = (C1_B * errorB + C2_B * errorB_k1 + C3_B * errorB_k2);
    
    if (abs(motorB.Speed) > 30)
        motorB.Speed = 30 * motorB.Speed/abs(motorB.Speed);
    end
    if (abs(motorB.Speed) < 17)
         motorB.Speed = 17 * motorB.Speed/abs(motorB.Speed);
    end
    
    start(motorB);
    
    errorB_k2 = errorB_k1;
    errorB_k1 = errorB;
    stopB = abs(errorB) + abs(errorB_k1) + abs(errorB_k2);
    pause (100/1000) %sampling time
end

if (pick == 0)
    motorC.Speed = -10;
    start(motorC);
    pause(1);
    stop(motorC);
end

 end
 
 %Homing Function
function homing(myev3) 

mytouchsensor1 = touchSensor(myev3,1);
 mytouchsensor2 = touchSensor(myev3,3);
 pressed1 = readTouch(mytouchsensor1);
 pressed2= readTouch(mytouchsensor2);

 motorA = motor(myev3, 'C');
 motorB = motor(myev3, 'B');
 motorC = motor(myev3, 'A');


%Homing
 motorC.Speed = -10;
 start(motorC);
 pause(1);
 stop(motorC)

while((pressed2==0) || (pressed1==0))
    motorB.Speed = -30;
    start(motorB);
    pressed2 = readTouch(mytouchsensor2);
    
    motorA.Speed = 15;
    start(motorA);
    pressed1 = readTouch(mytouchsensor1);
end

resetRotation(motorB)
stop(motorB);

resetRotation(motorA);
stop(motorA);
end
