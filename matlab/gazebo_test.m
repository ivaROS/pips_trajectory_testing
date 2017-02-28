close all
clear all

% bash: roslaunch pips_trajectory_testing gazebo_empty.launch
% matlab: rosinit('localhost');
%
WORLD_NUM = 500
WORLD_RANGE_X = [0.5 8]
WORLD_RANGE_Y = [-1 1]
%
OBJECT_TYPE = 'Cylinder'
OBJECT_NUM = 5 % 3 % 
OBJECT_RADIUS = [0.1 0.5]
OBJECT_HEIGHT = [0.1 0.5]
%
MAX_DEPTH_RANGE = 8
SCALE_FACTOR = 1000

%
do_viz = true

%% initial gazebo communicator
gazebo = ExampleHelperGazeboCommunicator();

% subscribe for camera info
if ismember('/camera/rgb/image_raw', rostopic('list'))
  imsub = rossubscriber('/camera/rgb/image_raw');
  depthsub = rossubscriber('/camera/depth/image_raw');
  pcdsub = rossubscriber('/camera/depth/points');
end

% subscribe for tf info
if ismember('/tf', rostopic('list'))
  tfsub = rossubscriber('/tf');
  tfstatsub = rossubscriber('/tf_static');
end

phys = readPhysics(gazebo);

phys;

setPhysics(gazebo,phys);

%% create random world
%resetSim(gazebo);
for iter = 1:WORLD_NUM
  
  %% [1]== Spawn random object in the world
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
  %
  pause(3)
  
  %% [2]== Grab color & depth image captured at the simulated scenario
  color_img = readImage(receive(imsub));
  if do_viz
    figure(1)
    imshow(color_img,'DisplayRange',[0,MAX_DEPTH_RANGE]);
  end
  imwrite(color_img, ['./output/color_world_' num2str(iter) '.png'], 'PNG');
  
  depth_img = readImage(receive(depthsub));
  depth_img(isnan(depth_img)) = 0;
  if do_viz
    figure(2);
    imshow(depth_img,'DisplayRange',[0,MAX_DEPTH_RANGE]);
  end
  imwrite(uint16(depth_img * SCALE_FACTOR), ['./output/depth_world_' num2str(iter) '.png'], 'PNG');
  
  % SANITY CHECK OF DEPTH IMAGE
  %
  %   depth_load = imread(['./output/depth_world_' num2str(iter)]);
  %   [pcd_loaded, ~] = Depth_PNG_To_PCD( depth_load );
  %   pcd_loaded = pointCloud(pcd_loaded);
  %
  %   pcd_sensed = pointCloud(readXYZ(receive(pcdsub)));
  %   figure(3)
  %   pcshowpair(pcd_sensed,pcd_loaded);
  
  pcd_sensed = pointCloud(readXYZ(receive(pcdsub)));
  
  pauseSim(gazebo);
  
  %% [3]== Perform collision check on the simulated scenario
  %
  % Ideally this should be done on robot frame; however I don't know how to
  % convert from sensor frame to robot frame yet. As a simple approximation,
  % I perform the checking on the X-Z plane of sensor frame
  %
  %   receive(tfsub)
  %   receive(tfstatsub)
  
  pcd_is_obs = pcd_sensed.Location(:, 2) < 0.3;
  
  % define the parm of descrete ogm
  ogm_scl = [5; 5];
  ogm_res = [0.02; 0.02];
  ogm_sz = uint16(ogm_scl ./ ogm_res);
  ogm_proj = zeros(ogm_sz');
  
  % convert from X-Z plane to ogm frame
  filled_idx = PCD_to_OGM(pcd_sensed, pcd_is_obs, ogm_sz, ogm_res);
  linear_idx = sub2ind(ogm_sz, filled_idx(1, :), filled_idx(2, :));
  ogm_proj(linear_idx) = 1;
  ogm_proj = logical(ogm_proj);
  
  % perform dilate on a descrete ogm with predined size and resolution
  robot_radius = 0.18;
  se = strel('disk', robot_radius / ogm_res(1), 0);
  ogm_eroded = imdilate(gpuArray(ogm_proj), se);
  
  if do_viz
    figure(3)
    subplot(1,2,1)
    imshow(ogm_proj)
    subplot(1,2,2)
    imshow(ogm_eroded)
  end
  
  % convert from the eroded ogm to sensor frame
  disp 'Time cost of ray tracing:'
  tic
  collide_rng = OGM_To_Depth_PNG(ogm_eroded, ogm_sz, ogm_res);
  toc
  if do_viz
    figure(4)
    plot(collide_rng, '--o')
  end
  save(['./output/collide_dist_' num2str(iter) '.mat'], 'collide_rng');
  
  %% [4]== Recycle the world for next run of simulation
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