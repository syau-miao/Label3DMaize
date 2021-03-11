function [ density ] = computeDensity(points)
%COMPUTEDENSITY �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
 PtsSz = length(points);
 kdtreeobj= KDTreeSearcher(points,'distance','euclidean');
% [Adj,dis ] = rangesearch(kdtreeobj,points,searchR);
 [Adj,dis ] = knnsearch(kdtreeobj,points,'k',2);
 density=sum(dis(:,2))/size(points,1);
end

