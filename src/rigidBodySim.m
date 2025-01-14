clear;
close all;


rob = rigidBodyTree();

link1 = rigidBody('link1');
jnt1 = rigidBodyJoint('joint1', 'fixed');
link1.Joint = jnt1;

jnt1.setFixedTransform(makehgtform("translate", [1 0 0]))


rob.addBody(link1, 'base')
rob.showdetails;

% figure(Name= 'Interactive GUI')
% hold on
% gui = interactiveRigidBodyTree(rob, MarkerScaleFactor=0.5);
rob.show;