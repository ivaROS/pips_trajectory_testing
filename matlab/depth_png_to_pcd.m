% Converts kinect depth data (11-bit measurements) to 3-D point cloud
%   Note(s):
%       1. Converts 11-bit depth measurements to meters/millimeters
%       2. Applies kinect depth camera intrinsic params to map each
%           measurement to 3-D coordinate (in camera frame)
%       3. Filters out invalid depth measurements (ie. 2047's)
%
%   Input(s):
%       m_depth_img - MxN matrix of 11-bit depth measurements
%
%   Output(s):
%       point_cloud_filt = Px3 matrix of 3-D coordinates (millimeters)
%                           (P = M*N - # invalid depth measurements)
%       point_cloud = 480x640x3 matrix mapping image positions to 3-D
%                           coordinates (in camera frame)
%
%   Note(s): 
%       - Invalid depth measurements constitute locations with depth value,
%           2047 (ie. max 11-bit value)
% 
function [ point_cloud_filt , valid_inds ] = depth_png_to_pcd( depth_data )
% 11-bit Depth to Meter Conversion:
%                       1.0 / (depthValue * -0.0030711016 + 3.3309495161)
%                       1054 -> 10.64 meters
% depth_data = 1.0 ./ (double(depth_png) * -0.0030711016 + 3.3309495161);
depth_data = double(depth_data) / 1000; % mm to meter

% Kinect depth camera intrinsic parameter(s):
% fx = 579.83; % in axis x
% fy = 586.73; % in axis y
% cx_img_coord = 321.55;  % optical center x
% cy_img_coord = 235.01;  % optical center y
fx = 554.25469; % in axis x
fy = 554.25469; % in axis y
cx_img_coord = 320.5;  % optical center x
cy_img_coord = 240.5;  % optical center y

x_img_coord = repmat((1:640), 480, 1);
y_img_coord = repmat((1:480)', 1, 640);

point_cloud = zeros(480, 640, 3);   % x, y, z
point_cloud(:, :, 1) = (x_img_coord - cx_img_coord) .* depth_data / fx;     % x-coord
point_cloud(:, :, 2) = (y_img_coord - cy_img_coord) .* depth_data / fy;     % y-coord
point_cloud(:, :, 3) = depth_data;                                          % z-coord

% Filter out 2047 (invalid) depth measurements
valid_inds = find(depth_data ~= 0 & ~isnan(depth_data));

valid_depth = depth_data(valid_inds);
valid_x = point_cloud(:, :, 1); valid_x = valid_x(valid_inds);
valid_y = point_cloud(:, :, 2); valid_y = valid_y(valid_inds);

point_cloud_filt = [reshape(valid_x, length(valid_inds), 1) ...
                    reshape(valid_y, length(valid_inds), 1) ...
                    reshape(valid_depth, length(valid_inds), 1) ];
end