function [SegId,unSegId] = RegionGrowingSegmentLeaf(points,normals,unSegId,SeedPoint,StopIds,SearchR,ProjdT,CosT,varargin)
%REGIONGROWINGSEGMENT_STEM_MT 此处显示有关此函数的摘要
%   此处显示详细说明
   SegId=[];  
   %%%%%%根据SeedPoint获得初始的种子点SeedIds%%%%%%%%%%%%%%%
   unSegPts=points(unSegId,:);%unSegId是针对points的索引
   unSegPtCloud = pointCloud(unSegPts);  
   [Ids,~]= findNearestNeighbors(unSegPtCloud,SeedPoint,8);
   SeedIds=unSegId(Ids);
   unSegId(Ids)=[];
   SegId=[SegId;SeedIds];
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   while(~isempty(SeedIds))
   %%%%%%%%%%%%%计算最近邻，放到SegId中%%%%%%%%%%%%%%%
     unSegPts=points(unSegId,:);%unSegId是针对points的索引
     unSegPtCloud = pointCloud(unSegPts);  
     spts=points(SeedIds,:);
     newSeedIds=[];
     deleteIds=[];
     spns=normals(SeedIds,:);
     %%%%%%%%%%%%%%拟合局部平面%%%%%%%%%%%%%%%%%
     [dirs ,~,~]= pca(spts); 
     plane=false;
     if(size(dirs,2)==3)
       plane_norml=dirs(:,3)';
       plane_center=mean(spts);
       plane=true;
     end
     for i=1:length(SeedIds)
       spt=spts(i,:);
       spn=spns(i,:);
       [Ids,~]= findNearestNeighbors(unSegPtCloud,spt,SearchR);
       if(length(Ids)==0)
         continue;
       end
       segId=unSegId(Ids);%Ids是针对UnSegId里的索引
       segPts=points(segId,:);%segId是针对points的索引
       segNormals=normals(segId,:);
       %dirs=segPts-spt;
       
       %%%%%%%%%%平面约束%%%%%%%%%%%%%%%%%%%%%
       if(plane)
          pjs=find_ProjCoord_mt(plane_norml,plane_center,segPts);
          v=pjs-segPts;
          ProjMatrix=sqrt(sum(v.*v,2));
       else
          ProjMatrix=pdist2(spt,segPts);
       end
      
       %%%%%%%%%normal 约束%%%%%%%%%%%
       CosMatrix=sum(segNormals.*spn,2);
       Ids1=Ids(find(CosMatrix>CosT));
       Ids2=Ids(find(ProjMatrix<ProjdT));
       Ids=intersect(Ids1,Ids2);
       
       

       
       
       
       segId=unSegId(Ids);
       StopId=intersect(segId,StopIds);
       if(~isempty(StopId))
          break;
       end
       deleteIds=[deleteIds;Ids];
     %  SegId=[SegId;segId];%%SegId是针对points的索引
       newSeedIds=[newSeedIds;segId];  
     end
     SeedIds=unique(newSeedIds);
     SegId=[SegId;SeedIds];
     deleteIds=unique(deleteIds);
     unSegId(deleteIds)=[];
   %%%%%%%%%%分类stem
 
   if(false)
    close all;
    figure('Name','regiongrowthsegment','NumberTitle','off');set(gcf,'color','white');movegui('southwest');
    scatter3(points(:,1),points(:,2),points(:,3),5,[0 0 0], 'filled');
    hold on
    scatter3(points(unSegId,1),points(unSegId,2),points(unSegId,3),5,[1 0 0], 'filled');
     hold on
%     scatter3(SeedPoint(1),SeedPoint(2),SeedPoint(3),10,[0 0 1], 'filled');
%     hold on
    scatter3(points(SegId,1),points(SegId,2),points(SegId,3),5,[0 1 0], 'filled');
    hold on;
%     scatter3(segPts(:,1),segPts(:,2),segPts(:,3),5,[1 1 0],'filled');
%     hold on
    axis off; axis equal; camorbit(0,0,'camera'); axis vis3d; view(0,0);view3d ZOOM;
   end
   
end

