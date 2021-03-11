function [ output_args ] = SegmetLeaf(points,unSegId,StemId,searchR)
%SEGMETLEAF 此处显示有关此函数的摘要
%   此处显示详细说明
  %%%从unSegId中提取与StemId有相邻的点%%%%%%%%%%
     stemPts=points(StemId,:);
     stemCloud = pointCloud(stemPts);  
     Candidates=zeros(size(points,1),1); 
     for i=1:length(unSegId)
       upt=points(unSegId(i),:);
       [Ids,~]= findNeighborsInRadius(stemCloud,upt,searchR);
       if(length(Ids)==0)
          continue;
       end 
       Candidates(unSegId(i))=1;
     end
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  k=1;
  while(~isempty(unSegId))
     cand=find(Candidates==1);
     if(length(cand)>0)
     unSegPts=points(cand,:); 
     [minX Ids]=min(unSegPts(:,1));
     SeedPoint=unSegPts(Ids,:);
     else
     unSegPts=points(unSegId,:); 
     [minX Ids]=min(unSegPts(:,1));
     SeedPoint=unSegPts(Ids,:);    
     end
     [leafId,unSegId]=RegionGrowingSegment(points,unSegId,SeedPoint,searchR,0);
     LeafId{k}=leafId;
     C=intersect(cand,leafId);
     Candidates(C)=0;
     k=k+1;
  end
   
 
 figure('Name','growing','NumberTitle','off');set(gcf,'color','white');movegui('southwest');
 scatter3(points(:,1),points(:,2),points(:,3),5,[0 0 0], 'filled');
 hold on
 scatter3(points(StemId,1),points(StemId,2),points(StemId,3),5,[1 0 0], 'filled');
 hold on
 for i=1:size(LeafId,2)
  unSegId=LeafId{i};
  color=[rand(1,1) rand(1,1) rand(1,1)];
 scatter3(points(unSegId,1),points(unSegId,2),points(unSegId,3),5,color, 'filled');
 hold on
 end
 axis off; axis equal; camorbit(0,0,'camera'); axis vis3d; view(0,0);view3d ZOOM;
  
  
  
end

