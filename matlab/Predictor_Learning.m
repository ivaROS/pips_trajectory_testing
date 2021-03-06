close all
clear all

WORLD_NUM = 266 % 500
MAX_DEPTH_RANGE = 8
SCALE_FACTOR = 1000
%
SLIDING_WINDOW_SZ = 160;
%
do_viz = false

% %% Load the ground truth data
% X = zeros(WORLD_NUM * 640, 240);
% Y = zeros(WORLD_NUM * 640, 1);
% for iter = 1:WORLD_NUM
%   %
%   disp(['start extracting features for simulate world ' num2str(iter)])
%
%   color_img = imread(['./output/color_world_' num2str(iter) '.png']);
%   depth_img = imread(['./output/depth_world_' num2str(iter) '.png']);
%   depth_img = double(depth_img) / SCALE_FACTOR;
%   invalid_idx = find(depth_img == 0);
%   depth_img(invalid_idx) = nan;
%   %
%   load(['./output/collide_dist_' num2str(iter) '.mat']);
%   collide = collide_rng;
%   %
%   [pcd_loaded, ~] = Depth_PNG_To_PCD( depth_img );
%   pcd_loaded = pointCloud(pcd_loaded);
%   %
%   if do_viz
%     figure(1)
%     %
%     subplot(2,2,1)
%     imshow(color_img,'DisplayRange',[0,MAX_DEPTH_RANGE]);
%     hold on
%     scatter(1:640, repmat(240.5, 1, 640), ones(640, 1), collide)
%     hold off
%     %
%     subplot(2,2,2)
%     imshow(depth_img,'DisplayRange',[0,MAX_DEPTH_RANGE]);
%     hold on
%     scatter(1:640, repmat(240.5, 1, 640), ones(640, 1), collide)
%     hold off
%     %
%     subplot(2,2,3)
%     pcshow(pcd_loaded)
%     subplot(2,2,4)
%     plot(collide, '--o')
%     xlim([1 640])
%     ylim([0 MAX_DEPTH_RANGE])
%   end
%
%   %% Feature extraction
%   % [1]== project the depth image to x-axis
%   %   depth_min = min(double(depth_img{iter}) / SCALE_FACTOR, 1);
%   % [2]== compute basic statistics along x-axis
%   labelr = collide;
%   featr = zeros(640, 240);
%   for sldwin = 1:640
%     if mod(sldwin, 100) == 0
%       disp(['sliding window centered at column ' num2str(sldwin)])
%     end
%     %
%     depth_win = depth_img(:, max(1, sldwin-SLIDING_WINDOW_SZ/2) : min(640, sldwin + SLIDING_WINDOW_SZ/2));
%     %
%     featr_tmp_1 = zeros(1, 48);
%     featr_tmp_2 = zeros(1, 48);
%     featr_tmp_3 = zeros(1, 48);
%     featr_tmp_4 = zeros(1, 48);
%     featr_tmp_5 = zeros(1, 48);
%     %
%     parfor blk_no = 1:48
%       depth_arr = depth_win(10*(blk_no-1)+1:10*blk_no, :);
%       %       featr(sldwin, 5*(blk_no-1)+1:5*blk_no) = [
%       %       nanmedian(depth_arr(:));
%       %       nanmean(depth_arr(:));
%       %       nanstd(depth_arr(:));
%       %       nanmax(depth_arr(:));
%       %       nanmin(depth_arr(:));
%       %       ];
%       featr_tmp_1(blk_no) = nanmedian(depth_arr(:));
%       featr_tmp_2(blk_no) = nanmean(depth_arr(:));
%       featr_tmp_3(blk_no) = nanstd(depth_arr(:));
%       featr_tmp_4(blk_no) = nanmax(depth_arr(:));
%       featr_tmp_5(blk_no) = nanmin(depth_arr(:));
%     end
%     %
%     featr(sldwin, :) = [featr_tmp_1 featr_tmp_2 featr_tmp_3 featr_tmp_4 featr_tmp_5];
%     %     if do_viz
%     %       figure(2)
%     %       subplot(1,2,1)
%     %       imagesc(depth_win)
%     %       subplot(1,2,2)
%     %       plot(featr(sldwin, :))
%     %     end
%     %
%   end
%   % [3]== Concatenate the feature array into matrix
%   X((iter-1)*640+1 : iter*640, :) = featr;
%   Y((iter-1)*640+1 : iter*640) = labelr;
%   %
% end
%
% save 'data_loaded'

%% Training
load '/mnt/BACK/GoogleDrive/Projects/Collision_Prediction/data_loaded'

% enhance X with the column idx in depth image
X = [repmat(1:640, 1, WORLD_NUM)', X];

% get rid of X rows with All NaN measurement
validIdx = any(~isnan(X(:, 2:end)), 2);
X = X(validIdx, :);
Y = Y(validIdx);

% randomnize the data for future training
rd_idx = randperm(length(Y));
X = X(rd_idx, :);
Y = Y(rd_idx);

% search for features that contribute most to regression
%
% c = cvpartition(Y,'k',10);
% opts = statset('display','iter');
% [fs,history] = sequentialfs(critfun,X,Y,'cv',c,'options',opts)
%
% compute mutual information between each predictor & regression value
for pn=1:size(X, 2)
  [~, MI_prob] = get_histogram(X(:, pn), Y, 512);
  [miArr(pn), ~] = get_NMI(MI_prob);
end
figure;plot(miArr, '--o')
%
[miArr_sorted, miIdx] = sort(miArr, 'descend');
figure;plot(miArr_sorted, '--o')
%
selectedIdx = miIdx(1:150);
X = X(:, selectedIdx);
% map Y into discrete value?
% discreteFac = 10;
% Y = round(Y*discreteFac)/discreteFac;

regressor_type = 2

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
    %     ensemble_final = fitensemble(X(1:30000, :), Y(1:30000), 'Bag', 200, ...
    %       t, 'NPrint', 50, 'Type', 'regression');
    ensemble_final = fitensemble(X, Y, 'Bag', 500, ...
      t, 'NPrint', 50, 'Type', 'regression');
    
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

save('/mnt/BACK/GoogleDrive/Projects/Collision_Prediction/regressor_full.mat', ...
  'ensemble_final', 'selectedIdx');
%
ensemble_cpt = compact(ensemble_final);
save('/mnt/BACK/GoogleDrive/Projects/Collision_Prediction/regressor_compact.mat', ...
  'ensemble_cpt', 'selectedIdx');

figure;
clf
plot(loss(ensemble_final, X, Y, 'mode', 'cumulative', 'lossfun', 'mse'));
xlabel('Number of trees');
ylabel('Regression loss');