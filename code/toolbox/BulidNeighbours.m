function [ n ] = BulidNeighbours(Pts,numNeighbours)
%BULIDNEIGHBOURS 此处显示有关此函数的摘要
%   此处显示详细说明
PtsSz = length(Pts);
 kdtreeobj= KDTreeSearcher(Pts,'distance','euclidean');
%[n,~ ] = rangesearch(kdtreeobj,Pts,20);
[n,dis] = knnsearch(kdtreeobj,Pts,'k',(numNeighbours+1));

end

