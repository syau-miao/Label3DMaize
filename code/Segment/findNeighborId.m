function [ indices ] = findNeighborId(points,pt,r)
%FINDNEIGHBORID �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
  ptCloud=pointCloud(points);
  pt=pt(1,:);
  [indices,~]=findNeighborsInRadius(ptCloud,pt,r);

end

