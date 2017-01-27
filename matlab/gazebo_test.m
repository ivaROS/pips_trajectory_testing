rosinit('localhost');
gazebo = ExampleHelperGazeboCommunicator();

if ismember('/camera/rgb/image_raw', rostopic('list'))
  imsub = rossubscriber('/camera/rgb/image_raw');
  depthsub = rossubscriber('/camera/depth/image_raw');
end

phys = readPhysics(gazebo);

phys;

setPhysics(gazebo,phys);

%resetSim(gazebo);
ind = 0;

for x = 5:8
    
ind = ind+1;
ball = ExampleHelperGazeboModel('Ball');
spherelink = addLink(ball,'sphere',1,'color',[0 0 1 1]);
spawnModel(gazebo,ball,[x,0,1]);

box = ExampleHelperGazeboModel('Box');
boxlink = addLink(box,'box',[.2,.1,.5],'color',[1 0 0 1]);
spawnModel(gazebo,box,[x-1.5,0,.5]);

pause(1);

img = receive(imsub);
figure(ind)
imshow(readImage(img));

ind = ind+1;
img = receive(depthsub);
figure(ind);
imshow(readImage(img));

pauseSim(gazebo);
%[position, orientation, velocity] = getState(ball)

models = getSpawnedModels(gazebo);

pause(2);

if ismember('Ball', getSpawnedModels(gazebo))
        removeModel(gazebo,'Ball');
end
if ismember('Box', getSpawnedModels(gazebo))
        removeModel(gazebo,'Box');
end

resumeSim(gazebo);

end


clear;


rosshutdown;