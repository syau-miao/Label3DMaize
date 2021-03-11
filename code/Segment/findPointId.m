function [ id ] = findPointId( points,pt )
%FINDPOINTID find the index of pt from points
%   此处显示详细说明
 index=ismember(points,pt,'rows');
 id=find(index==1);
 id=id(1);
end

