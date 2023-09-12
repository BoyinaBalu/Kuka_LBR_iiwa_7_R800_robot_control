clc
clear all
close all
%conversion of Joint angles from degrees to rad

q(1) = deg2rad(-77.26);
q(2) = deg2rad(-38.76);
q(3) = deg2rad(26.22);
q(4) = deg2rad(93.29);
q(5) = deg2rad(-56.69);
q(6) = deg2rad(-59.94);
q(7) = deg2rad(118);

%Transformation Matrices from Base to End effector
T01 = [cos(q(1)) 0 -sin(q(1)) 0;
		sin(q(1)) 0 cos(q(1)) 0;
		0 -1 0 0.34;
		0 0 0 1];
T12 = [cos(q(2)) 0 sin(q(2)) 0;
		sin(q(2)) 0 -cos(q(2)) 0;
		0 1 0 0;
		0 0 0 1];
T23 = [cos(q(3)) 0 sin(q(3)) 0;
		sin(q(3)) 0 -cos(q(3)) 0;
		0 1 0 0.4;
		0 0 0 1];
T34 = [cos(q(4)) 0 -sin(q(4)) 0;
		sin(q(4)) 0 cos(q(4)) 0;
		0 -1 0 0;
		0 0 0 1];
T45 = [cos(q(5)) 0 -sin(q(5)) 0;
		sin(q(5)) 0 cos(q(5)) 0;
		0 -1 0 0.4;
		0 0 0 1];
T56 = [cos(q(6)) 0 sin(q(6)) 0;
		sin(q(6)) 0 -cos(q(6)) 0;
		0 1 0 0;
		0 0 0 1];
T6E = [cos(q(7)) -sin(q(7)) 0 0;
		sin(q(7)) cos(q(7)) 0 0;
		0 0 1 0.126;
		0 0 0 1];

%Final Transformation matrix from Base to End Effector of the robot
T0E = T01*T12*T23*T34*T45*T56*T6E;

%d = 0.0431, theta = -90, a = 0.0662, alpha = 0
%transformation from end effector to camera
TEC = [0 1 0 0; -1 0 0 -0.0662; 0 0 1 0.0431; 0 0 0 1];

% Transformation From camera to Auruko Maker
roll = deg2rad(-172.95718336855933);
pitch = deg2rad(-27.847557028831005);
yaw = deg2rad(68.70697922863141);
euler_angles = [yaw, pitch, roll];
rotation_sequence = 'ZYX';
RCA = eul2rotm(euler_angles, rotation_sequence);
%RCA = eul2rotm([roll pitch yaw],'zyx');
TCA1 = [RCA; 0 0 0];
TCA2 = [-0.14195360128424114; -0.06062556383004704; 0.3528046636209403;1];
TCA = [TCA1, TCA2];


%transformation from Aurukomarker to Target frame
TAT = [1 0 0 0.103975;
	0 1 0 -0.103975;
	0 0 1 0;
	0 0 0 1];

%Transformation from Base frame to Traget Frame
T0Tr = T0E * TEC * TCA * TAT;

%transformation matrix end effector to obj
T_EObj = [0 -1 0 0.0455; -1 0 0 0; 0 0 -1 0.060; 0 0 0 1];

T_TObj = T0Tr * T_EObj;

R0E_fin = T_TObj(1:3,1:3);
euler = rotm2eul(R0E_fin,'zyz');
xpos = T_TObj(1,4);
ypos = T_TObj(2,4);
zpos = T_TObj(3,4);


%desired pose
pd = [xpos; ypos; zpos];
phid = [euler(1); euler(2); euler(3)];

%inverse Kinematics
q1 = deg2rad([58.2686 75.3224 11.7968 45.9029 -22.1081 -31.2831 -42.3712]);

 steps = 1000000;
 gain = 20;
 q = zeros(7, steps);
 e = zeros(6, steps);
 J = zeros(6,7);
 q(:,1) = q1';
 for i = 1:steps
	xe = frwd_kin(q(:,i));
	Ta1 = g_2_an(q(:,i));
    Ja = Jac(q(:,i), pd,Ta1);
    e(:,i) = [pd; phid]-xe;
    psinvjac = pinv(Ja);
    qdot = psinvjac*(gain*e(:,i));
    q(:,i+1) = q(:,i) + qdot*0.01;
    if (max(abs(e(:,i))) < 0.00001)
        break;
    end
 end
 z = q(:,i);
 zdeg = rad2deg(z);
 for y = 1:7
     if zdeg(y)>(180)
         while zdeg(y)>(180)
             zdeg(y) = zdeg(y)-360;
         end
     elseif zdeg(y)<-180
         while zdeg(y)<-180
             zdeg(y) = zdeg(y)+360;
         end
     end
 end
 disp(zdeg);
 zrad= deg2rad(zdeg);
 Z = zrad';
 theta1 = Z(1);
 theta2 = Z(2);
 theta3 = Z(3);
 theta4 = Z(4);
 theta5 = Z(5);
 theta6 = Z(6);
 theta7 = Z(7);

% New_DH parameters
d = [0.340, 0, 0.4, 0, 0.4, 0, 0.126];
theta = [theta1, theta2, theta3, theta4, theta5, theta6, theta7];
a = [0, 0, 0, 0, 0, 0, 0];
alpha = [-pi/2, pi/2, pi/2, -pi/2, -pi/2, pi/2, 0];

T = eye(4); % Transformation matrix initialization

for i = 1:numel(d)
    
    di = d(i);
    thetai = theta(i);
    ai = a(i);
    alphai = alpha(i);
    A = [cos(thetai) -sin(thetai)*cos(alphai) sin(thetai)*sin(alphai) ai*cos(thetai);
         sin(thetai) cos(thetai)*cos(alphai) -cos(thetai)*sin(alphai) ai*sin(thetai);
         0 sin(alphai) cos(alphai) di;
         0 0 0 1];
    T = T * A;
end

disp("Transformation Matrix:");
disp(T);

% Trajectory planning
fileID = fopen('Boyina_BalaSubrahmanyam.txt','w');
fmt = '%.4f %.4f %.4f %.4f %.4f %.4f %.4f\n';
Time=10;
q1=q1';
a0=q1;
a1=0;
a2=(-3*(q1-zrad))/(Time^2);
a3=2*(q1-zrad)/(Time^3);
for t=0:0.005:Time
    qt=(a3)*(t^3)+(a2)*(t^2)+(a1)*(t)+(a0);
    qt=qt';
    fprintf(fileID,fmt,qt);
end
%Txt file of velocities to make sure the are meeting their constraints
fileID2 = fopen('Velocity_limits_Check.txt','w');
fmt = '%.4f %.4f %.4f %.4f %.4f %.4f %.4f\n';
for t=0:0.005:Time
    qv=3*(a3)*(t^2)+2*(a2)*(t)+(a1);
    qv=qv';
    fprintf(fileID2,fmt,qv);
end


  function p= frwd_kin(q)
    T01 = [cos(q(1)) 0 -sin(q(1)) 0; sin(q(1)) 0 cos(q(1)) 0; 0 -1 0 0.34; 0 0 0 1];
    T12 = [cos(q(2)) 0 sin(q(2)) 0; sin(q(2)) 0 -cos(q(2)) 0; 0 1 0 0; 0 0 0 1];
    T23 = [cos(q(3)) 0 sin(q(3)) 0; sin(q(3)) 0 -cos(q(3)) 0; 0 1 0 0.4; 0 0 0 1];
    T34 = [cos(q(4)) 0 -sin(q(4)) 0; sin(q(4)) 0 cos(q(4)) 0; 0 -1 0 0; 0 0 0 1];
    T45 = [cos(q(5)) 0 -sin(q(5)) 0; sin(q(5)) 0 cos(q(5)) 0; 0 -1 0 0.4; 0 0 0 1];
    T56 = [cos(q(6)) 0 sin(q(6)) 0; sin(q(6)) 0 -cos(q(6)) 0; 0 1 0 0; 0 0 0 1];
    T6E = [cos(q(7)) -sin(q(7)) 0 0; sin(q(7)) cos(q(7)) 0 0; 0 0 1 0.126; 0 0 0 1];
    T0E = T01*T12*T23*T34*T45*T56*T6E;
    x = T0E(1,4); y = T0E(2,4); z = T0E(3,4);
    R0E = T0E(1:3,1:3);
    eulerzyz = rotm2eul(R0E,'zyz');
    p= [x; y; z;eulerzyz(1);eulerzyz(2);eulerzyz(3)];

end
%function for Ta matrix
 function Ta = g_2_an(q)
    T01 = [cos(q(1)) 0 -sin(q(1)) 0; sin(q(1)) 0 cos(q(1)) 0; 0 -1 0 0.34; 0 0 0 1];
    T12 = [cos(q(2)) 0 sin(q(2)) 0; sin(q(2)) 0 -cos(q(2)) 0; 0 1 0 0; 0 0 0 1];
    T23 = [cos(q(3)) 0 sin(q(3)) 0; sin(q(3)) 0 -cos(q(3)) 0; 0 1 0 0.4; 0 0 0 1];
    T34 = [cos(q(4)) 0 -sin(q(4)) 0; sin(q(4)) 0 cos(q(4)) 0; 0 -1 0 0; 0 0 0 1];
    T45 = [cos(q(5)) 0 -sin(q(5)) 0; sin(q(5)) 0 cos(q(5)) 0; 0 -1 0 0.4; 0 0 0 1];
    T56 = [cos(q(6)) 0 sin(q(6)) 0; sin(q(6)) 0 -cos(q(6)) 0; 0 1 0 0; 0 0 0 1];
    T6E = [cos(q(7)) -sin(q(7)) 0 0; sin(q(7)) cos(q(7)) 0 0; 0 0 1 0.126; 0 0 0 1];
    T0E = T01*T12*T23*T34*T45*T56*T6E;

   R0E = T0E(1:3,1:3);
   euler1 = rotm2eul(R0E,'zyz');
   Ta = [1 0 0 0 0 0;
       0 1 0 0 0 0;
       0 0 1 0 0 0;
       0 0 0 0 -sin(euler1(1)) cos(euler1(1))*sin(euler1(2));
       0 0 0 0 cos(euler1(1)) sin(euler1(1))*sin(euler1(2));
       0 0 0 1 0 cos(euler1(2))];
 end
 function J = Jac(q, pe,Ta)
   
    T01 = [cos(q(1)) 0 -sin(q(1)) 0; sin(q(1)) 0 cos(q(1)) 0; 0 -1 0 0.34; 0 0 0 1];
    T12 = [cos(q(2)) 0 sin(q(2)) 0; sin(q(2)) 0 -cos(q(2)) 0; 0 1 0 0; 0 0 0 1];
    T02 = T01*T12;
    T23 = [cos(q(3)) 0 sin(q(3)) 0; sin(q(3)) 0 -cos(q(3)) 0; 0 1 0 0.4; 0 0 0 1];
    T03 = T02*T23;
    T34 = [cos(q(4)) 0 -sin(q(4)) 0; sin(q(4)) 0 cos(q(4)) 0; 0 -1 0 0; 0 0 0 1];
    T04 = T03*T34;
    T45 = [cos(q(5)) 0 -sin(q(5)) 0; sin(q(5)) 0 cos(q(5)) 0; 0 -1 0 0.4; 0 0 0 1];
    T05 = T04*T45;
    T56 = [cos(q(6)) 0 sin(q(6)) 0; sin(q(6)) 0 -cos(q(6)) 0; 0 1 0 0; 0 0 0 1];
    T06 = T05*T56;
   
    z0 = [0 0 1]'; p0 = [0 0 0]';
    z1 = T01(1:3,3); p1 = T01(1:3,4);
    z2 = T02(1:3,3); p2 = T02(1:3,4);
    z3 = T03(1:3,3); p3 = T03(1:3,4);
    z4 = T04(1:3,3); p4 = T04(1:3,4);
    z5 = T05(1:3,3); p5 = T05(1:3,4);
    z6 = T06(1:3,3); p6 = T06(1:3,4);
    
    Jg = [cross(z0,pe-p0) cross(z1,pe-p1) cross(z2,pe-p2) cross(z3,pe-p3) cross(z4,pe-p4) cross(z5,pe-p5) cross(z6,pe-p6);
           z0 z1 z2 z3 z4 z5 z6];
       J = inv(Ta)*Jg;
end
