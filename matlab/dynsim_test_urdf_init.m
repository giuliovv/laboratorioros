clear all
close all

% Create robot object
robot_2link = rigidBodyTree('DataFormat','column','MaxNumBodies',4);

L1 = 0.337; % Arm length [m]
L2 = 0.210; % Forearm length [m]
L3 = 0.268; % Ultimoarm lenght [m]
L4 = 0;
L5 = 0.174;
L6 = 0;

body = rigidBody('edo_link_1');
joint = rigidBodyJoint('edo_joint_1', 'revolute');
setFixedTransform(joint,trvec2tform([0 0 0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
body.Mass = 2;
body.CenterOfMass = [L1/2 0 0];
body.Inertia = [0 0.67 0.67 0 0 0]; % [Ixx Iyy Izz Iyz Ixz Ixy]
addBody(robot_2link, body, 'base');

body = rigidBody('edo_link_2');
joint = rigidBodyJoint('edo_joint_2','revolute');
setFixedTransform(joint, trvec2tform([L1,0,0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
body.Mass = 1;
body.CenterOfMass = [L2/2 0 0];
body.Inertia = [0 0.335 0.335 0 0 0]; % [Ixx Iyy Izz Iyz Ixz Ixy]
addBody(robot_2link, body, 'edo_link_1');

body = rigidBody('edo_link_3');
joint = rigidBodyJoint('edo_joint_3','revolute');
setFixedTransform(joint, trvec2tform([L2,0,0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
body.Mass = 1;
body.CenterOfMass = [L3/2 0 0];
body.Inertia = [0 0.1675 0.1675 0 0 0]; % [Ixx Iyy Izz Iyz Ixz Ixy]
addBody(robot_2link, body, 'edo_link_2');

showdetails(robot_2link)

% Load robot object from URDF
robot_2link_fromURDF = importrobot('../urdf/2link_planar_model.urdf');

showdetails(robot_2link_fromURDF)
