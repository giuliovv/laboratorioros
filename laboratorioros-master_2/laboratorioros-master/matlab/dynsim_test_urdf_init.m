clear all
close all

% Create robot object
robot_2link = rigidBodyTree('DataFormat','column','MaxNumBodies',7);

L1 = 0.337; % Arm length [m]
L2 = 0.210; % Forearm length [m]
L3 = 0.268; % Ultimoarm lenght [m]
L4 = 0;
L5 = 0.174;
L6 = 0;

dhparams = [0.337   	-pi/2	0   	0;
            0           0       0.210   0;
            0           -pi/2	0   	0;
            0.268   	pi/2	0       0;
            0           -pi/2	0   	0;
            0.174       0       0       0];

body = rigidBody('link_1');
joint = rigidBodyJoint('joint_1', 'revolute');
setFixedTransform(joint,dhparams(1,:),'dh');
joint.JointAxis = [0 0 1];
body.Joint = joint;
body.Mass = 2;
body.CenterOfMass = [L1/2 0 0];
body.Inertia = [0 0.67 0.67 0 0 0]; % [Ixx Iyy Izz Iyz Ixz Ixy]
addBody(robot_2link, body, 'base');

body = rigidBody('link_2');
joint = rigidBodyJoint('joint_2','revolute');
setFixedTransform(joint,dhparams(2,:),'dh')
joint.JointAxis = [0 0 1];
body.Joint = joint;
body.Mass = 1;
body.CenterOfMass = [L2/2 0 0];
body.Inertia = [0 0.335 0.335 0 0 0]; % [Ixx Iyy Izz Iyz Ixz Ixy]
addBody(robot_2link, body, 'link_1');

body = rigidBody('link_3');
joint = rigidBodyJoint('joint_3','revolute');
setFixedTransform(joint,dhparams(3,:),'dh')
%joint.JointAxis = [0 0 1];
body.Joint = joint;
body.Mass = 1;
body.CenterOfMass = [L3/2 0 0];
body.Inertia = [0 0.1675 0.1675 0 0 0]; % [Ixx Iyy Izz Iyz Ixz Ixy]
addBody(robot_2link, body, 'link_2');

body = rigidBody('link_4');
joint = rigidBodyJoint('joint_4', 'revolute');
setFixedTransform(joint,dhparams(4,:),'dh')
%joint.JointAxis = [0 0 1];
body.Joint = joint;
body.Mass = 1;
body.CenterOfMass = [L4/2 0 0];
body.Inertia = [0 0.1675 0.1675 0 0 0]; % [Ixx Iyy Izz Iyz Ixz Ixy]
addBody(robot_2link, body, 'link_3');

body = rigidBody('link_5');
joint = rigidBodyJoint('joint_5','revolute');
setFixedTransform(joint,dhparams(5,:),'dh')
%joint.JointAxis = [0 0 1];
body.Joint = joint;
body.Mass = 1;
body.CenterOfMass = [L5/2 0 0];
body.Inertia = [0 0.1675 0.1675 0 0 0]; % [Ixx Iyy Izz Iyz Ixz Ixy]
addBody(robot_2link, body, 'link_4');

body = rigidBody('link_6');
joint = rigidBodyJoint('joint_6','revolute');
setFixedTransform(joint,dhparams(6,:),'dh')
%joint.JointAxis = [0 0 1];
body.Joint = joint;
body.Mass = 1;
body.CenterOfMass = [L6/2 0 0];
body.Inertia = [0 0.1675 0.1675 0 0 0]; % [Ixx Iyy Izz Iyz Ixz Ixy]
addBody(robot_2link, body, 'link_5');

showdetails(robot_2link)

% Load robot object from URDF
robot_2link_fromURDF = importrobot('urdf/2link_planar_model.urdf');

showdetails(robot_2link_fromURDF)
