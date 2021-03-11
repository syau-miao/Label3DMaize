function [  normals ] = find_normal_data( pts )
%FIND_NORMAL_DATA 此处显示有关此函数的摘要
%   此处显示详细说明
  ptCloud = pointCloud(pts);
  normals = pcnormals(ptCloud,16);

%   figure
%   pcshow(ptCloud)
%   title('Estimated Normals of Point Cloud')
%   hold on
x = ptCloud.Location(1:end,1);
y = ptCloud.Location(1:end,2);
z = ptCloud.Location(1:end,3);
u = normals(1:end,1);
v = normals(1:end,2);
w = normals(1:end,3);
%Plot the normal vectors.



end

