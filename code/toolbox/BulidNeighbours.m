function [ n ] = BulidNeighbours(Pts,numNeighbours)
%BULIDNEIGHBOURS �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
PtsSz = length(Pts);
 kdtreeobj= KDTreeSearcher(Pts,'distance','euclidean');
%[n,~ ] = rangesearch(kdtreeobj,Pts,20);
[n,dis] = knnsearch(kdtreeobj,Pts,'k',(numNeighbours+1));

end

