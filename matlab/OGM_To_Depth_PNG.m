function [ depth_data ] = OGM_To_Depth_PNG( ogm_data, ogm_sz, ogm_res )

MAX_DEPTH = 8.0;

% Kinect depth camera intrinsic parameter(s):
% fx = 579.83; % in axis x
% fy = 586.73; % in axis y
% cx_img_coord = 321.55;  % optical center x
% cy_img_coord = 235.01;  % optical center y
fx = 554.25469; % in axis x
fy = 554.25469; % in axis y
cx = 320.5;  % optical center x
cy = 240.5;  % optical center y

%
% since the FOV of kinect v1 & v2 are both within 90 deg, we only need to
% trace the ray along each z grid (y in ogm)
%
sx = double(ogm_sz(1))/2;
sy = 0;

depth_data = ones(640, 1) * MAX_DEPTH;
parfor ix = 1:640
  for iy = 240.5 % 1:480
    ray = [(ix - cx) / fx; (iy - cy) / fy; 1];
%     ray = ray / norm(ray, 2);
%     %
%     % NOTE idealy we need to do one more transfrom from sensor frame to
%     % robot frame (ogm frame); however at the moment I don't have access to
%     % these extrinsic parameters.
%     % Will get back to it later
%     %
%     
%     % perform ray tracing on ogm
%     % define the search interval according to the ray orientation
%     if abs(ray(3)) < eps
%       continue ;
%     end
    check_intv = [ray(1) / ray(3); 1];
    check_cell = [sx; sy];
    collide_dist = MAX_DEPTH;
    % iteratively search through all cells
    while Within_Range(check_cell(1), 1, ogm_sz(1)) && check_cell(2) <= ogm_sz(2)
%       check_cell
      if check_cell(2) >= 1 && ...
          ogm_data(round(check_cell(1)), round(check_cell(2))) == 1
        % collision detected
        collide_dist = norm((round(check_cell) - [sx; sy]) .* ogm_res, 2);
        break ;
      else
        % free space detected
        check_cell = check_cell + check_intv;
      end
    end
    %
    %     depth_data(ix, iy) = collide_dist;
    depth_data(ix) = collide_dist;
  end
end

end