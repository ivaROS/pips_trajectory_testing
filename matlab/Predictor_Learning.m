close all
clear all

WORLD_NUM = 500
MAX_DEPTH_RANGE = 8
SCALE_FACTOR = 1000
%
do_viz = true

%% load the ground truth data
for iter = 1:WORLD_NUM
  %
  color_img{iter} = imread(['./output/color_world_' num2str(iter) '.png']);
  depth_img{iter} = imread(['./output/depth_world_' num2str(iter) '.png']);
  load(['./output/collide_dist_' num2str(iter) '.mat']);
  collide{iter} = collide_rng;
  %
  [pcd_loaded, ~] = Depth_PNG_To_PCD( depth_img{iter} );
  pcd_loaded = pointCloud(pcd_loaded);
  %
  if do_viz
    figure(1)
    subplot(2,2,1)
    imshow(color_img{iter},'DisplayRange',[0,MAX_DEPTH_RANGE]);
    subplot(2,2,2)
    imshow(double(depth_img{iter}) / SCALE_FACTOR,'DisplayRange',[0,MAX_DEPTH_RANGE]);
    subplot(2,2,3)
    pcshow(pcd_loaded)
    subplot(2,2,4)
    plot(collide{iter}, '--o')
    xlim([0 640])
  end
end
