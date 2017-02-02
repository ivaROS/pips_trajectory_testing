close all
clear all

% bash: roslaunch pips_trajectory_testing gazebo_empty.launch
% matlab: rosinit('localhost');
%
WORLD_NUM = 10
WORLD_RANGE_X = [0.5 8]
WORLD_RANGE_Y = [-1 1]
%
OBJECT_TYPE = 'Cylinder'
OBJECT_NUM = 5
OBJECT_RADIUS = [0.1 0.5]
OBJECT_HEIGHT = [0.1 0.5]
%
MAX_DEPTH_RANGE = 8
SCALE_FACTOR = 1000

%% initial gazebo communicator
gazebo = ExampleHelperGazeboCommunicator();

if ismember('/camera/rgb/image_raw', rostopic('list'))
  imsub = rossubscriber('/camera/rgb/image_raw');
  depthsub = rossubscriber('/camera/depth/image_raw');
  pcdsub = rossubscriber('/camera/depth/points');
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
        radius = OBJECT_RADIUS(1) + rand([1 1]) * (OBJECT_RADIUS(2) - OBJECT_RADIUS(1));
        addLink(obj_model{ii},'sphere',radius,'color',[0 0 1 1]);
        %
        pose = zeros([2 1]);
        pose(1) = WORLD_RANGE_X(1) + rand([1 1]) * (WORLD_RANGE_X(2) - WORLD_RANGE_X(1));
        pose(2) = WORLD_RANGE_Y(1) + rand([1 1]) * (WORLD_RANGE_Y(2) - WORLD_RANGE_Y(1));
        orient = rand([1 1]) * pi;
        spawnModel(gazebo,obj_model{ii},[pose(1),pose(2),0], [0,0,orient(1)]);
      end
    case 'Box'
      for ii=1:OBJECT_NUM
        obj_model{ii} = ExampleHelperGazeboModel('Box');
        %
        % TODO fix the issue of size adjustment
        % For now we simply avoid creating box object to the world
        sz = OBJECT_RADIUS(1) + rand([1 1]) * (OBJECT_RADIUS(2) - OBJECT_RADIUS(1));
        addLink(obj_model{ii},'box',sz,'color',[0 0 1 1]);
        %
        pose = zeros([2 1]);
        pose(1) = WORLD_RANGE_X(1) + rand([1 1]) * (WORLD_RANGE_X(2) - WORLD_RANGE_X(1));
        pose(2) = WORLD_RANGE_Y(1) + rand([1 1]) * (WORLD_RANGE_Y(2) - WORLD_RANGE_Y(1));
        orient = rand([1 1]) * pi;
        spawnModel(gazebo,obj_model{ii},[pose(1),pose(2),0], [0,0,orient(1)]);
      end
    case 'Cylinder'
      for ii=1:OBJECT_NUM
        obj_model{ii} = ExampleHelperGazeboModel('Cylinder');
        %
        length = OBJECT_HEIGHT(1) + rand([1 1]) * (OBJECT_HEIGHT(2) - OBJECT_HEIGHT(1));
        radius = OBJECT_RADIUS(1) + rand([1 1]) * (OBJECT_RADIUS(2) - OBJECT_RADIUS(1));
        addLink(obj_model{ii},'cylinder',[length radius],'color',[0 0 1 1]);
        %
        pose = zeros([2 1]);
        pose(1) = WORLD_RANGE_X(1) + rand([1 1]) * (WORLD_RANGE_X(2) - WORLD_RANGE_X(1));
        pose(2) = WORLD_RANGE_Y(1) + rand([1 1]) * (WORLD_RANGE_Y(2) - WORLD_RANGE_Y(1));
        orient = rand([1 1]) * pi;
        spawnModel(gazebo,obj_model{ii},[pose(1),pose(2),0], [0,0,orient(1)]);
      end
    otherwise
      %
  end
  
  pause(3)
  
  color_img = readImage(receive(imsub));
  figure(1)
  imshow(color_img,'DisplayRange',[0,MAX_DEPTH_RANGE]);
  imwrite(color_img, ['./output/color_world_' num2str(iter)], 'PNG');
  
  depth_img = readImage(receive(depthsub));
  depth_img(isnan(depth_img)) = 0;
  figure(2);
  imshow(depth_img,'DisplayRange',[0,MAX_DEPTH_RANGE]);
  imwrite(uint16(depth_img * SCALE_FACTOR), ['./output/depth_world_' num2str(iter)], 'PNG');
  
%   depth_load = imread(['./output/depth_world_' num2str(iter)]);
%   [pcd_loaded, ~] = depth_png_to_pcd( depth_load );
%   pcd_loaded = pointCloud(pcd_loaded);
%   
%   pcd_sensed = pointCloud(readXYZ(receive(pcdsub)));
%   figure(3)
%   pcshowpair(pcd_sensed,pcd_loaded);
  
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