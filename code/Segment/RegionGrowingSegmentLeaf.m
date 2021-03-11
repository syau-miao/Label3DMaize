function [SegId,unSegId] = RegionGrowingSegmentLeaf(points,normals,unSegId,SeedPoint,StopIds,SearchR,ProjdT,CosT,varargin)
%REGIONGROWINGSEGMENT_STEM_MT �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
   SegId=[];  
   %%%%%%����SeedPoint��ó�ʼ�����ӵ�SeedIds%%%%%%%%%%%%%%%
   unSegPts=points(unSegId,:);%unSegId�����points������
   unSegPtCloud = pointCloud(unSegPts);  
   [Ids,~]= findNearestNeighbors(unSegPtCloud,SeedPoint,8);
   SeedIds=unSegId(Ids);
   unSegId(Ids)=[];
   SegId=[SegId;SeedIds];
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   while(~isempty(SeedIds))
   %%%%%%%%%%%%%��������ڣ��ŵ�SegId��%%%%%%%%%%%%%%%
     unSegPts=points(unSegId,:);%unSegId�����points������
     unSegPtCloud = pointCloud(unSegPts);  
     spts=points(SeedIds,:);
     newSeedIds=[];
     deleteIds=[];
     spns=normals(SeedIds,:);
     %%%%%%%%%%%%%%��Ͼֲ�ƽ��%%%%%%%%%%%%%%%%%
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
       segId=unSegId(Ids);%Ids�����UnSegId�������
       segPts=points(segId,:);%segId�����points������
       segNormals=normals(segId,:);
       %dirs=segPts-spt;
       
       %%%%%%%%%%ƽ��Լ��%%%%%%%%%%%%%%%%%%%%%
       if(plane)
          pjs=find_ProjCoord_mt(plane_norml,plane_center,segPts);
          v=pjs-segPts;
          ProjMatrix=sqrt(sum(v.*v,2));
       else
          ProjMatrix=pdist2(spt,segPts);
       end
      
       %%%%%%%%%normal Լ��%%%%%%%%%%%
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
     %  SegId=[SegId;segId];%%SegId�����points������
       newSeedIds=[newSeedIds;segId];  
     end
     SeedIds=unique(newSeedIds);
     SegId=[SegId;SeedIds];
     deleteIds=unique(deleteIds);
     unSegId(deleteIds)=[];
   %%%%%%%%%%����stem
 
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

