close all
clear all

WORLD_NUM = 266 % 500
MAX_DEPTH_RANGE = 8
SCALE_FACTOR = 1000
%
SLIDING_WINDOW_SZ = 160;
%
do_viz = false

%% Load the ground truth data
X = zeros(WORLD_NUM * 640, 2400);
Y = zeros(WORLD_NUM * 640, 1);
for iter = 1:WORLD_NUM
  %
  color_img{iter} = imread(['./output/color_world_' num2str(iter) '.png']);
  depth_img{iter} = imread(['./output/depth_world_' num2str(iter) '.png']);
  depth_img{iter} = double(depth_img{iter}) / SCALE_FACTOR;
  invalid_idx = find(depth_img{iter} == 0);
  depth_img{iter}(invalid_idx) = nan;
  %
  load(['./output/collide_dist_' num2str(iter) '.mat']);
  collide{iter} = collide_rng;
  %
  [pcd_loaded, ~] = Depth_PNG_To_PCD( depth_img{iter} );
  pcd_loaded = pointCloud(pcd_loaded);
  %
  if do_viz
    figure(1)
    %
    subplot(2,2,1)
    imshow(color_img{iter},'DisplayRange',[0,MAX_DEPTH_RANGE]);
    hold on
    scatter(1:640, repmat(240.5, 1, 640), ones(640, 1), collide{iter})
    hold off
    %
    subplot(2,2,2)
    imshow(depth_img{iter},'DisplayRange',[0,MAX_DEPTH_RANGE]);
    hold on
    scatter(1:640, repmat(240.5, 1, 640), ones(640, 1), collide{iter})
    hold off
    %
    subplot(2,2,3)
    pcshow(pcd_loaded)
    subplot(2,2,4)
    plot(collide{iter}, '--o')
    xlim([1 640])
    ylim([0 MAX_DEPTH_RANGE])
  end
  
  %% Feature extraction
  % [1]== project the depth image to x-axis
  %   depth_min = min(double(depth_img{iter}) / SCALE_FACTOR, 1);
  % [2]== compute basic statistics along x-axis
  labelr = collide{iter};
  featr = zeros(640, 2400);
  for sldwin = 1:640
    depth_win = depth_img{iter}(:, max(1, sldwin-SLIDING_WINDOW_SZ/2) : min(640, sldwin + SLIDING_WINDOW_SZ/2));
    featr(sldwin, :) = [
      nanmedian(depth_win, 2);
      nanmean(depth_win, 2);
      nanstd(depth_win, [], 2);
      nanmax(depth_win, [], 2);
      nanmin(depth_win, [], 2);
      ];
    %     if do_viz
    %       figure(2)
    %       subplot(1,2,1)
    %       imagesc(depth_win)
    %       subplot(1,2,2)
    %       plot(featr(sldwin, :))
    %     end
    %
  end
  % [3]== Concatenate the feature array into matrix
  X((iter-1)*640+1 : iter*640, :) = featr;
  Y((iter-1)*640+1 : iter*640) = labelr;
  %
end

%% Training
for regressor_type = 1:2
  
  switch regressor_type
    case 1
      % General linear regression
      opts = statset('display','iter');
      fun = @(x0,y0,x1,y1) norm(y1-x1*(x0\y0))^2;  % residual sum of squares
      [in,history] = sequentialfs(fun,X,Y,'cv',5, 'options',opts)
      
      beta_final = regress(Y, X(:,in));
    case 2
      % Ensemble
      t = templateTree('Surrogate','on');
      ensemble_final = fitensemble(X, Y, 'Bag', 200, ...
        t, 'NPrint', 50, 'Type', 'Classification');
      figure;
      clf
      plot(loss(ensemble_final, X, Y, 'mode', 'cumulative'));
      xlabel('Number of trees');
      ylabel('Component classification error');
      
%     case 3
%       % High-dimension data linear regression (SVM + LASSO)
%       Lambda = logspace(-5,-1,15);
%       CVMdl = fitrlinear(X',Y,'ObservationsIn','columns','KFold',5,'Lambda',Lambda,...
%         'Learner','leastsquares','Solver','sparsa','Regularization','lasso','Verbose',2);
%       
%       numCLModels = numel(CVMdl.Trained)
%       
%       Mdl1 = CVMdl.Trained{1}
%       mse = kfoldLoss(CVMdl);
%       
%       % manual search for the best lambda
%       Mdl = fitrlinear(X',Y,'ObservationsIn','columns','Lambda',Lambda,...
%         'Learner','leastsquares','Solver','sparsa','Regularization','lasso','Verbose',2);
%       numNZCoeff = sum(Mdl.Beta~=0);
%       
%       figure;
%       [h,hL1,hL2] = plotyy(log10(Lambda),log10(mse),...
%         log10(Lambda),log10(numNZCoeff));
%       hL1.Marker = 'o';
%       hL2.Marker = 'o';
%       ylabel(h(1),'log_{10} MSE')
%       ylabel(h(2),'log_{10} nonzero-coefficient frequency')
%       xlabel('log_{10} Lambda')
%       hold off
%       
%       idxFinal = 10;
%       MdlFinal = selectModels(Mdl,idxFinal)
%       idxNZCoeff = find(MdlFinal.Beta~=0)
%       beta_f = Mdl.Beta(idxNZCoeff)
%     case 4
%       % auto optimize
%       hyperopts = struct('AcquisitionFunctionName','expected-improvement-plus');
%       [Mdl,FitInfo,HyperparameterOptimizationResults] = fitrlinear(X,Y,...
%         'OptimizeHyperparameters','auto',...
%         'HyperparameterOptimizationOptions',hyperopts)
%       
%       beta_HDLR_Manual = Mdl.Beta
  end
  
end

save('./output/regressor_debug.mat', 'beta_final', 'ensemble_final', 'X', 'Y');