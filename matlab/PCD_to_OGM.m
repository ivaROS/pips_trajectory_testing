function ogm_idx = PCD_to_OGM(pcd, pcd_idx, ogm_sz, ogm_res)
%%

pcd_proj = pcd.Location(pcd_idx, [1,3])';

ogm_idx = round(pcd_proj ./ repmat(ogm_res, 1, size(pcd_proj,2)) + ...
  repmat([double(ogm_sz(1))/2; 0], 1, size(pcd_proj,2)));

end