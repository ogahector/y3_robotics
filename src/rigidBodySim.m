clear;
close all;


rob = rigidBodyTree("DataFormat","row");

link1 = rigidBody('link1');
link2 = rigidBody('link2');
link3 = rigidBody('link3');


jnt1 = rigidBodyJoint('joint1', 'revolute');
jnt2 = rigidBodyJoint('joint2', 'revolute');
jnt3 = rigidBodyJoint('joint3', 'revolute');

jnt1.setFixedTransform(makehgtform("translate", [1 0 0]))
jnt2.setFixedTransform(makehgtform("translate", [0 1 0]))
jnt3.setFixedTransform(makehgtform("translate", [0 0 1]))

link1.Joint = jnt1;
link2.Joint = jnt2;
link3.Joint = jnt3;

rob.addBody(link1, 'base')
rob.addBody(link2, 'link1')
rob.addBody(link3, 'link2')

rob.showdetails;
rob.show;
% gui = interactiveRigidBodyTree(rob, MarkerScaleFactor=0.5);