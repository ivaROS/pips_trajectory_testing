close all
% clear all

WORLD_NUM = 266 % 500
MAX_DEPTH_RANGE = 8
SCALE_FACTOR = 1000
%
SLIDING_WINDOW_SZ = 160;
%
do_viz = true

%% Load the ground truth data
load '/mnt/BACK/GoogleDrive/Projects/Collision_Prediction/regressor_full';

for iter = 1:WORLD_NUM
  %
  disp(['start extracting features for simulate world ' num2str(iter)])

  color_img = imread(['./output/color_world_' num2str(iter) '.png']);
  depth_img = imread(['./output/depth_world_' num2str(iter) '.png']);
  depth_img = double(depth_img) / SCALE_FACTOR;
  invalid_idx = find(depth_img == 0);
  depth_img(invalid_idx) = nan;
  %
  load(['./output/collide_dist_' num2str(iter) '.mat']);
  collide = collide_rng;
  %
  [pcd_loaded, ~] = Depth_PNG_To_PCD( depth_img );
  pcd_loaded = pointCloud(pcd_loaded);
  %
  if do_viz
    h = figure(1)
    %
    subplot(2,2,1)
    imshow(color_img,'DisplayRange',[0,MAX_DEPTH_RANGE]);
    hold on
    scatter(1:640, repmat(240.5, 1, 640), ones(640, 1), collide)
    hold off
    %
    subplot(2,2,2)
    imshow(depth_img,'DisplayRange',[0,MAX_DEPTH_RANGE]);
    hold on
    scatter(1:640, repmat(240.5, 1, 640), ones(640, 1), collide)
    hold off
    %
    subplot(2,2,3)
    pcshow(pcd_loaded)
    subplot(2,2,4)
    plot(collide, '--o')
    xlim([1 640])
    ylim([0 MAX_DEPTH_RANGE])
  end

  %% Feature extraction
  % [1]== project the depth image to x-axis
  %   depth_min = min(double(depth_img{iter}) / SCALE_FACTOR, 1);
  % [2]== compute basic statistics along x-axis
  labelr = collide;
  featr = zeros(640, 241);
  for sldwin = 1:640
    if mod(sldwin, 100) == 0
      disp(['sliding window centered at column ' num2str(sldwin)])
    end
    %
    depth_win = depth_img(:, max(1, sldwin-SLIDING_WINDOW_SZ/2) : min(640, sldwin + SLIDING_WINDOW_SZ/2));
    %
    featr_tmp_1 = zeros(1, 48);
    featr_tmp_2 = zeros(1, 48);
    featr_tmp_3 = zeros(1, 48);
    featr_tmp_4 = zeros(1, 48);
    featr_tmp_5 = zeros(1, 48);
    %
    parfor blk_no = 1:48
      depth_arr = depth_win(10*(blk_no-1)+1:10*blk_no, :);
      %       featr(sldwin, 5*(blk_no-1)+1:5*blk_no) = [
      %       nanmedian(depth_arr(:));
      %       nanmean(depth_arr(:));
      %       nanstd(depth_arr(:));
      %       nanmax(depth_arr(:));
      %       nanmin(depth_arr(:));
      %       ];
      featr_tmp_1(blk_no) = nanmedian(depth_arr(:));
      featr_tmp_2(blk_no) = nanmean(depth_arr(:));
      featr_tmp_3(blk_no) = nanstd(depth_arr(:));
      featr_tmp_4(blk_no) = nanmax(depth_arr(:));
      featr_tmp_5(blk_no) = nanmin(depth_arr(:));
    end
    %
    featr(sldwin, :) = [sldwin featr_tmp_1 featr_tmp_2 featr_tmp_3 featr_tmp_4 featr_tmp_5];
    %     if do_viz
    %       figure(2)
    %       subplot(1,2,1)
    %       imagesc(depth_win)
    %       subplot(1,2,2)
    %       plot(featr(sldwin, :))
    %     end
    %
  end
  % [3]== Perform prediction with the loaded regressor
  predictr = predict(ensemble_cpt, featr(:, selectedIdx));
  if do_viz
    figure(h)
    %
    subplot(2,2,1)
    hold on
    scatter(1:640, repmat(245.5, 1, 640), ones(640, 1), predictr)
    hold off
    %
    subplot(2,2,2)
    hold on
    scatter(1:640, repmat(245.5, 1, 640), ones(640, 1), predictr)
    hold off
    %
    subplot(2,2,4)
    hold on
    plot(predictr, '--x')
    hold off
    xlim([1 640])
    ylim([0 MAX_DEPTH_RANGE])
  end
  %
  export_fig(h, ['./output/predict_collide_' num2str(iter) '.png']);
  close all
end