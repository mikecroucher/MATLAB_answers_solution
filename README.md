# MATLAB answers solution

While attempting to help with https://www.mathworks.com/matlabcentral/answers/1929070-help-with-ode-tolerance-options?s_tid=srchtitle, the original poster saw different behaviour to me so I created this to ensure that we are both running the exact same thing.

To run this, do 
```

A = load("torque_for_Sid_60s.mat");
T_a = A.torque_60s(1:2:1000001);                  %Torque (Nm) chnages with time step
speed = 1000/60; 
tspan=0:0.00001*2:10;    %can try 4 or 5
y0 = [0;0;0;0;0;-104.719;0;0;0;0;0;104.719/3]; 
opts = odeset('RelTol',1e-1,'AbsTol',1e-1);
tic
[t,y] = ode45(@(t,y) reduced2(t,y,T_a),tspan,y0,opts);  
oldtime = toc
tic
%These are constant for every call to reduced2 so compute them here and
%pass them into the modified function reduced3
time = 0:0.00001*2:10;
Torque = griddedInterpolant(time, T_a);
[tnew,ynew] = ode45(@(t,y) reduced3(t,y,Torque),tspan,y0,opts); 
newtime = toc
fprintf("Speedup is %fx\n",oldtime/newtime)
fprintf("Are all results equal?\n")
all(tnew==t)
all(ynew==y,'all')
% Driver gear graphs
nexttile  
plot(t,y(:,2))
ylabel('(y) up and down velocity (m/s)')
xlabel('Time')
title('Driver gear velocity in y direction vs time')
axis padded  
grid on
nexttile  
plot(t,y(:,4))  
ylabel('(z) side to side velocity (m/s)')
xlabel('Time')
title('Driver gear velocity in z direction vs time')  
axis padded  
grid on
nexttile  
plot(t,y(:,6))  
ylabel('angular velocity (rad/s)')
xlabel('Time')
title('Driver gear angular velocity vs time')  
axis padded  
grid on
% Driven gear graphs
nexttile  
plot(t,y(:,8))  
ylabel('(y) up and down velocity (m/s)')
xlabel('Time')
title('Driven gear velocity in y direction vs time')  
axis padded  
grid on
nexttile  
plot(t,y(:,10))  
ylabel('(z) side to side velocity (m/s)')
xlabel('Time')
title('Driven gear velocity in z direction vs time')  
axis padded  
grid on
nexttile  
plot(t,y(:,12))  
ylabel('angular velocity (rad/s)')
xlabel('Time')
title('Driven gear angular velocity vs time')  
axis padded  
grid on
```
