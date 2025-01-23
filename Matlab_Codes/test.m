clc
clear
close all
%%
% robot = loadrobot("universalUR5","DataFormat","row");
robot = loadrobot("universalUR5","DataFormat","row",'Gravity',[0 0 -9.81]);
show(robot)
showdetails(robot)

%%
randConfig = robot.randomConfiguration;

%%
% config = [0 0 0 3*pi/2 0 0 ];
config = [0 0 0 0 0 0 ];

%%
% tform = getTransform(robot,randConfig,'ee_link','base');
tform = getTransform(robot,config,'tool0','base');



% aik = analyticalInverseKinematics(robot);
% tform = [pi/2 0 0 0.5;0 0 0 0.5;0 0 0 0.5;0 0 0 1];

% ik = inverseKinematics('RigidBodyTree',robot)
% tform = [pi/2 0 0 0.5;0 0 0 0.5;0 0 0 0.5;0 0 0 1];

figure
subplot(1,2,1)
show(robot,config);

ik = inverseKinematics('RigidBodyTree',robot);
weights = [0.1 0.25 0.1 1 1 1];
initialguess = robot.homeConfiguration;


[configSoln,solnInfo] = ik('tool0',tform,weights,initialguess);
subplot(1,2,2)
show(robot,configSoln);
% geometricJacobian
%% Panda
robot2 = loadrobot("frankaEmikaPanda","DataFormat","row");
showdetails(robot)

%%
