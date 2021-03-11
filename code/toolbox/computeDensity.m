function [ density ] = computeDensity(points)
%COMPUTEDENSITY 此处显示有关此函数的摘要
%   此处显示详细说明
 PtsSz = length(points);
 kdtreeobj= KDTreeSearcher(points,'distance','euclidean');
% [Adj,dis ] = rangesearch(kdtreeobj,points,searchR);
 [Adj,dis ] = knnsearch(kdtreeobj,points,'k',2);
 density=sum(dis(:,2))/size(points,1);
end

