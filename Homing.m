clc;clear;

%  intialising Robot
 myev3 = legoev3('USB');
 mytouchsensor1 = touchSensor(myev3,1);
 mytouchsensor2 = touchSensor(myev3,3);
 pressed1 = readTouch(mytouchsensor1);
 pressed2= readTouch(mytouchsensor2);

 motorA = motor(myev3, 'C');
 motorB = motor(myev3, 'B');
 motorC = motor(myev3, 'A');


%Homing

while((pressed2==0) || (pressed1==0))
    motorB.Speed = -30;
    start(motorB);
    pressed2 = readTouch(mytouchsensor2);
    
    motorA.Speed = 10;
    start(motorA);
    pressed1 = readTouch(mytouchsensor1);
end

resetRotation(motorB)

rotationB = readRotation(motorB);
disp(rotationB);
stop(motorB)

resetRotation(motorA);
stop(motorA);
rotationA = readRotation(motorA);
disp(rotationA);
