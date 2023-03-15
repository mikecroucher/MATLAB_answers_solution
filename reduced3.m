function [yp] = reduced3(t,y,Torque)  
beta = 13*(pi/180);                      %Helix angle (rad)
P = 20*(pi/180);                         %Pressure angle (rad)
speed = 1000/60; 
Freq = 1000*20/60;
R_a = 51.19e-3;                             %Radius
R_p = 139.763e-3;                           %Radius
% Common parameters
e_a = (pi*2*R_a*tan(beta))/(2*cos(P));      %circumferential displacement of the driver gear (m)
e_p = (pi*2*R_p*tan(beta))/(2*cos(P));      %circumferential displacement of the driver gear (m)
% e = 0;
e = (pi*2*(R_a+R_p)*tan(beta))/(4*cos(P));
z = e*tan(beta);                %axial tooth displacement caused by internal excitation (m)
Cs = 0.1 + 0.0001*sin(2*pi*Freq*t);                     %Shear damping
% Driver gear  
m_a = 0.5;                      %mass of driver gear (kg)
c_ay=500;                       %Damping of driver gear in y direction (Ns/m)
c_az=500;                       %Damping of driver gear in z direction (Ns/m)
k_ay= 8e7;                      %Stiffness of driver gear in y direction (N/m)
k_az= 5e7;                      %Stiffness of driver gear in z direction (N/m)
I_a = 0.5*m_a*(R_a^2);          %Moment of inertia of driver gear (Kg.m3)
% y_ac= e_a + theta_a_vec*R_a;    %circumferential displacement at the meshing point on the driver gear (m)
% y_pc = e_p - theta_a_vec*R_a;   %circumferential displacement at the meshing point on the driven gear (m)
z_a = e_a*tan(beta);  
z_p = e_p*tan(beta);  
% z_ac = (z_a-y_ac)*tan(beta);    %axial displacements of the meshing point on the driving gear  (m)
% z_pc = (z_p+y_pc)*tan(beta);    %axial displacements of the meshing point on the driven gear   (m)
% Driven gear  
m_p = 0.5;                                  %mass of driven gear (kg)
c_py=500;                                   %Damping of driven gear in y direction (Ns/m)
c_pz=500;                                   %Damping of driven gear in z direction (Ns/m)
k_py= 9.5e7;                                %Stiffness of driven gear in y direction (N/m)
k_pz =9e7;                                  %Stiffness of driven gear in z direction (N/m)
I_p = 0.5*m_p*(R_p^2);                      %Moment of inertia of driven gear (Kg.m3)
n_a = 20;                           % no of driver gear teeth  
n_p = 60;                           % no of driven gear teeth  
i = n_p/n_a;                        % gear ratio
% % Excitation forces 
% Fy=ks*cos(beta)*(y_ac-y_pc-e) + Cs*cos(beta)*2*R_a*speed*2*pi;                %axial dynamic excitation force at the meshing point (N)
% Fz=ks*sin(beta)*(z_ac-z_pc-z) - Cs*sin(beta)*2*tan(beta)*R_a*speed*2*pi;      %circumferential dynamic excitation force at the meshing point (N)
%Time interpolation  
Torque = Torque(t)/1000;
% torque_table = [time', T_a];
% torque = interp1(torque_table(:,1), torque_table(:,2), t, 'linear', 'extrap') / 1000;
Opp_Torque = 68.853/1000;                        % Average torque value
%Driver gear equations
yp = zeros(12,1);                           %vector of 12 equations
ks = 0.9e8 + 20000*sin(2*pi*Freq*t);                   %Shear stiffness 
TE = y(11)*R_p - y(5)*R_a;                   %transmission error
b = 20e-6;                                   %Backlash
if(TE>b)
    h = TE - b;
        KS = h*ks;
elseif(TE < -1*b)
        h = TE + b;
        KS = h*ks;
else 
    h = 0;
   KS = h*ks; 
end
   
yp(1) = y(2); 
yp(2) = (-(Cs*cos(beta)*(R_a*y(6) + R_p*y(12)) - KS*cos(beta)*(R_a*y(5)+R_p*y(11)) - KS*cos(beta)*(e_a-e_p-e)) - k_ay*y(1) - c_ay*y(2))/m_a;  %acceleration in y (up and down) direction (m/s2)
yp(3) = y(4);  
yp(4) = (-(KS*sin(beta)*(z_a*tan(beta) - e_a*tan(beta) - R_a*y(5)*tan(beta) - z_p*tan(beta) - e_p*tan(beta) - R_p*y(11)*tan(beta) - z) + Cs*sin(beta)*(-tan(beta)*R_a*y(6) - tan(beta)*R_p*y(12))) - k_az*y(3) - c_az*y(4))/m_a;  %acceleration in z (side to side) direction (m/s2)
yp(5) = y(6);  
yp(6) = (Torque - Cs*cos(beta)*R_a*(R_a*y(6) + R_p*y(12)) - KS*cos(beta)*R_a*(R_a*y(5)+R_p*y(11)) -KS*cos(beta)*R_a*(e_a-e_p-e))/I_a; %angular acceleration
%Driven gear equations
yp(7) = y(8);  
yp(8) = (-(Cs*cos(beta)*(R_a*y(6) + R_p*y(12)) - KS*cos(beta)*(R_a*y(5)+R_p*y(11)) - KS*cos(beta)*(e_a-e_p-e)) - k_py*y(7) - c_py*y(8))/m_p;            %acceleration in y (up and down) direction (m/s2)
yp(9) = y(10);  
yp(10) = (-(KS*sin(beta)*(z_a*tan(beta) - e_a*tan(beta) - R_a*y(5)*tan(beta) - z_p*tan(beta) - e_p*tan(beta) - R_p*y(11)*tan(beta) - z) + Cs*sin(beta)*(-tan(beta)*R_a*y(6) - tan(beta)*R_p*y(12))) - k_pz*y(9) - c_pz*y(10))/m_p;          %acceleration in y (side to side) direction (m/s2)
yp(11) = y(12);  
yp(12) = (Opp_Torque*i - Cs*cos(beta)*R_p*(R_a*y(6) + R_p*y(12)) - KS*cos(beta)*R_p*(R_a*y(5)+ R_p*y(11)) -KS*cos(beta)*R_p*(e_a-e_p-e))/I_p;                % angular accelration (rad/s2) 
end