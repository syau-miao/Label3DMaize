function [SegId,unSegId] = RegionGrowingSegment_GUI(points,unSegId,SeedPoint,SearchR,StopPoint)
%REGIONGROWINGSEGMENT_STEM_MT 此处显示有关此函数的摘要
%   此处显示详细说明
 SegId=[];
  % DebugShow=true;
 PreDir=StopPoint-SeedPoint;
 PreDir=PreDir./norm(PreDir);
 newDir=PreDir;
 LineO=SeedPoint;
 [ ~,stopT ] = P2LineDistance(LineO,PreDir,StopPoint);
 
  for i=1:inf
     unSegPts=points(unSegId,:);%unSegId是针对points的索引
     unSegPtCloud = pointCloud(unSegPts);  
     [Ids,~]= findNeighborsInRadius(unSegPtCloud,SeedPoint,SearchR);
     if(isempty(Ids))
       newDir=StopPoint-SeedPoint;
       newDir=newDir./norm(newDir);
       SeedPoint=SeedPoint+newDir*SearchR;  
       continue;   
     end
      segId=unSegId(Ids);
      segPts=points(segId,:);%segId是针对points的索引 
      [ ~,t1 ] = P2LineDistance(LineO,PreDir,segPts);
      ids1=find(t1<stopT);
      if(isempty(ids1))
         break;
      end
     segId=unSegId(Ids(ids1));
     unSegId(Ids(ids1))=[];
     SegId=[SegId;segId];
     segPts=points(segId,:);
     segDir=(segPts-SeedPoint); 
     for i=1:size(segDir,1)
       if(norm(segDir(i,:))==0)
         continue;  
       end
       segDir(i,:)=segDir(i,:)./norm(segDir(i,:));
     end
     newDir=median(segDir);
     if(norm(newDir)==0)
        break; 
     end
     newDir=newDir./norm(newDir);
     adjustDir=StopPoint-SeedPoint;
     adjustDir=adjustDir./norm(adjustDir);
     newDir=0.1*newDir+0.9*adjustDir;
     newDir=newDir./norm(newDir);
     SeedPoint=SeedPoint+newDir*SearchR;
  end
   %%%%%%%%%%%%%计算最近邻，放到SegId中%%%%%%%%%%%%%%%
    

     %Ids是针对UnSegId里的索引
    
    % Len=norm(SeedPoint-StopPoint);
     

   %%%%%%%%%%分类stem
 
   if(nargin==6)
   % close all;
    figure('Name','regiongrowthsegment','NumberTitle','off');set(gcf,'color','white');movegui('southwest');
    scatter3(points(:,1),points(:,2),points(:,3),5,[0 0 0], 'filled');
    hold on
    scatter3(points(unSegId,1),points(unSegId,2),points(unSegId,3),5,[1 0 0], 'filled');
     hold on
     scatter3(SeedPoint(1),SeedPoint(2),SeedPoint(3),20,[0 0 1], 'filled');
%     hold on
    scatter3(points(SegId,1),points(SegId,2),points(SegId,3),5,[0 1 0], 'filled');
    hold on;
%     scatter3(segPts(:,1),segPts(:,2),segPts(:,3),5,[1 1 0],'filled');
%     hold on
    axis off; axis equal; camorbit(0,0,'camera'); axis vis3d; view(0,0);view3d ZOOM;
   end
 
   
   %%删除茎中叶片点
%    if(bStem)
%     stemPts= points(SegId,:);
%     [res,dir_]=fitline(stemPts);
%     center=[res(1) res(2) res(3)]; 
%     dir=[dir_(1) dir_(2) dir_(3)];
%     total=[];
%     tvec=[];
%     for i=1:size(stemPts,1)
%     pt=stemPts(i,:);
%     [dis,t]=P2LineDistance(center,dir,pt);
%     total=[total;dis];
%     tvec=[tvec;t];
%     end
%     %%%%%沿着茎tvec进行分割%%%%%%%%%%%%%%
%     mint=min(tvec); maxt=max(tvec);
%     IntNum=1;
%     stept=(maxt-mint)/IntNum;
%     %%%%%%%每一段的茎中，去掉大于median R%%%%%%%%%%%%%%
%     for i=1:IntNum
%        t1=mint+stept*(i-1);t2=t1+stept;
%        indices=find(tvec>=t1&tvec<=t2);%% indices 针对茎的索引
%        ds=total(indices);%%indices 针对点云的索引
%        mds=median(ds);
%        ids=indices(find(ds>mds));%%ids针对茎的
%        if(isempty(ids))continue;end
%        uIds=SegId(ids);%%%uIds针对点云的
%        unSegId=[unSegId;uIds];
%     end
%     C=intersect(unSegId,SegId);
%     SegId=setdiff(SegId,C);
  
      if(nargin==6)
   % close all;
    figure('Name','AfterMedian_Stem','NumberTitle','off');set(gcf,'color','white');movegui('southwest');
    scatter3(points(:,1),points(:,2),points(:,3),5,[0 0 0], 'filled');
    hold on
    scatter3(points(unSegId,1),points(unSegId,2),points(unSegId,3),5,[1 0 0], 'filled');
     hold on
     scatter3(SeedPoint(1),SeedPoint(2),SeedPoint(3),20,[0 0 1], 'filled');
%     hold on
    scatter3(points(SegId,1),points(SegId,2),points(SegId,3),5,[0 1 0], 'filled');
    hold on;
%     scatter3(segPts(:,1),segPts(:,2),segPts(:,3),5,[1 1 0],'filled');
%     hold on
    axis off; axis equal; camorbit(0,0,'camera'); axis vis3d; view(0,0);view3d ZOOM;
   end

