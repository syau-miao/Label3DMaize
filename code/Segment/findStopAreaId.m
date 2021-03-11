function [ ids ] = findStopAreaId( points,unSegIds,pts,r )
%FINDSTOPAREAID 此处显示有关此函数的摘要
%   此处显示详细说明
  unSegPts=points(unSegIds,:);
   kdtreeobj= KDTreeSearcher(unSegPts,'distance','euclidean');
 [Adj,dis ] = rangesearch(kdtreeobj,pts,r);
 ids=zeros(length(unSegIds),1);
 for i=1:length(Adj)
    id=Adj{i};
    ids(id)=1;
 end
 ids=unSegIds(find(ids==1));
 
 % [Adj,dis ] = knnsearch(kdtreeobj,pts,'k',2);

%   ptCloud=pointCloud(unSegPts);
%   [indices,~]=findNeighborsInRadius(ptCloud,pts,r);
  
end

