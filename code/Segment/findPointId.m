function [ id ] = findPointId( points,pt )
%FINDPOINTID find the index of pt from points
%   �˴���ʾ��ϸ˵��
 index=ismember(points,pt,'rows');
 id=find(index==1);
 id=id(1);
end

