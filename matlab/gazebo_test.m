close all
clear all

WORLD_NUM = 10
OBJECT_TYPE = 'Cylinder'
OBJECT_NUM = 5
MAX_DEPTH_RANGE = 8

%% initial gazebo communicator
% rosinit('localhost');
% rosinit
gazebo = ExampleHelperGazeboCommunicator();

if ismember('/camera/rgb/image_raw', rostopic('list'))
  imsub = rossubscriber('/camera/rgb/image_raw');
  depthsub = rossubscriber('/camera/depth/image_raw');
end

phys = readPhysics(gazebo);

phys;

setPhysics(gazebo,phys);

%% create random world
%resetSim(gazebo);
for iter = 1:WORLD_NUM
  
  % spawn random object
  switch OBJECT_TYPE
    case 'Ball'
      for ii=1:OBJECT_NUM
        obj_model{ii} = ExampleHelperGazeboModel('Ball');
        %
        radius = rand([1 1]) * 0.2;
        addLink(obj_model{ii},'sphere',radius,'color',[0 0 1 1]);
        %
        p = rand([2 1]) * 2;
        r = rand([1 1]) * pi;
        spawnModel(gazebo,obj_model{ii},[p(1),p(2),0], [0,0,r(1)]);
      end
    case 'Box'
      for ii=1:OBJECT_NUM
        obj_model{ii} = ExampleHelperGazeboModel('Box');
        %
        % TODO fix the issue of size adjustment
        % For now we simply avoid creating box object to the world
        sz = rand([1 1]) * 0.2;
        addLink(obj_model{ii},'box',sz,'color',[0 0 1 1]);
        %
        p = rand([2 1]) * 2;
        r = rand([1 1]) * pi;
        spawnModel(gazebo,obj_model{ii},[p(1),p(2),0], [0,0,r(1)]);
      end
    case 'Cylinder'
      for ii=1:OBJECT_NUM
        obj_model{ii} = ExampleHelperGazeboModel('Cylinder');
        %
        length = rand([1 1]) * 0.5;
        radius = rand([1 1]) * 0.2;
        addLink(obj_model{ii},'cylinder',[length radius],'color',[0 0 1 1]);
        %
        p = rand([2 1]) * 2;
        r = rand([1 1]) * pi;
        spawnModel(gazebo,obj_model{ii},[p(1),p(2),0], [0,0,r(1)]);
      end
    otherwise
        %
  end
  
  pause(3)
  
  img = receive(imsub);
  figure(1)
  imshow(readImage(img),'DisplayRange',[0,MAX_DEPTH_RANGE]);
  
  img = receive(depthsub);
  figure(2);
  imshow(readImage(img),'DisplayRange',[0,MAX_DEPTH_RANGE]);
  
  pauseSim(gazebo);
  %[position, orientation, velocity] = getState(ball)
  
%   models = getSpawnedModels(gazebo);
  
  for ii=1:OBJECT_NUM
    if ismember('Ball', getSpawnedModels(gazebo))
      removeModel(gazebo, 'Ball');
    end
    if ismember(['Ball_' num2str(ii-1)], getSpawnedModels(gazebo))
      removeModel(gazebo, ['Ball_' num2str(ii-1)]);
    end
    %
    if ismember('Box', getSpawnedModels(gazebo))
      removeModel(gazebo, 'Box');
    end
    if ismember(['Box_' num2str(ii-1)], getSpawnedModels(gazebo))
      removeModel(gazebo, ['Box_' num2str(ii-1)]);
    end
    %
    if ismember('Cylinder', getSpawnedModels(gazebo))
      removeModel(gazebo, 'Cylinder');
    end
    if ismember(['Cylinder_' num2str(ii-1)], getSpawnedModels(gazebo))
      removeModel(gazebo, ['Cylinder_' num2str(ii-1)]);
    end
  end
  
  
  resumeSim(gazebo);
  
end

clear;
rosshutdown;