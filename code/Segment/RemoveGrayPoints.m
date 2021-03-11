function [ points_,colors_] = RemoveGrayPoints(points, colors )
%REMOVEGRAYPOINTS 此处显示有关此函数的摘要
%   此处显示详细说明
   hsv=rgb2hsv(colors);
   indices=find(hsv(:,2)>0.05);
   points_=points(indices,:);
   colors_=colors(indices,:);
  
end

