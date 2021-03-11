function [ indices ] = findNeighborId(points,pt,r)
%FINDNEIGHBORID 此处显示有关此函数的摘要
%   此处显示详细说明
  ptCloud=pointCloud(points);
  pt=pt(1,:);
  [indices,~]=findNeighborsInRadius(ptCloud,pt,r);

end

