function [ Regions,Regions2 ] = classifyLeafPoints_EMD( points,EMD,leafSeeds,unSegment ,SampleNum,SampleNum2,a3,times)
%CLASSIFYLEAFPOINTS_MT 此处显示有关此函数的摘要
%   此处显示详细说明
   leafNum=size(leafSeeds,1);
   Regions2=cell(leafNum,1);
   leafSegment=cell(leafNum,1);
   UnSegPts=points(unSegment,:);
   %UnSegNormal=normals(unSegment,:);
   UnSegPts_z=UnSegPts(:,1);
   UnSegPts_x=UnSegPts(:,2);
   UnSegPts_y=UnSegPts(:,3);
   EFF=UnSegPts_z+0*UnSegPts_x+0*UnSegPts_y;
   [sortZ, SortIndices]=sort(EFF,'descend');
   if(false)
   figure('Name','RoughSegment','NumberTitle','off');set(gcf,'color','white');movegui('southwest');
   color=[1 0 0;0 0 1;0 1 0;1 1 0;1 0 1; 0 1 1;0.5 0 0;0 0.5 0;0 0 0.5;0.5 0.5 0;0.5 0 0.5;0 0.5 0.5;0.25 0 0;0 0.25 0;0 0 0.25;0.25 0.25 0];
   scatter3(points(:,1),points(:,2),points(:,3),5,[0.0 0.0 0], 'filled');
   hold on;
    for i=2:size(leafSeeds,1)
    I1=leafSeeds{i};
    scatter3(points(I1,1),points(I1,2),points(I1,3),5,[rand(1,1) rand(1,1) rand(1,1)], 'filled');
    hold on;
    end
   axis off; axis equal; camorbit(0,0,'camera'); axis vis3d; view(0,0);view3d ZOOM;
   end 




   for i=1:size(SortIndices,1)
       index_u=SortIndices(i); 
       index=unSegment(index_u);
    %   pt=UnSegPts(index,:);
      % pn=UnSegNormal(index,:);
       min_dis=inf;
       min_id=-1;
       Dis1=zeros(leafNum,1); Dis2=zeros(leafNum,1);
       a1=1.0;a2=1.0;%SampleNum=5;SampleNum2=5;
       %a3=0.5;index
      % emd=EMD(index,:);
      % [emd_,~]=sort(emd_);
       for j=times:leafNum
          ids=leafSeeds{j};
          Dis1(j)=caldis1(points,EMD,index,ids,SampleNum); 
       end
       [sortDis1,SortIndices1]=sort(Dis1,'ascend');
       s1=SortIndices1(1); s2=SortIndices1(2);
       d1=sortDis1(1);     d2=sortDis1(2);
   %  d1_2=Dis2(s1);       d2_2=Dis2(s2);
%         if(s1==1)
%            min_id=s1;
%         else
%         if(abs(d1-d2)/d1>a3)
%            min_id=s1; 
%         else
           ids1=leafSeeds{s1};
           leafPts1=points(ids1,:);
           ids2=leafSeeds{s2};
           leafPts2=points(ids2,:);
            if(length(ids1)<6||length(ids2)<6||a3==0)
               d1_2=0;
               d2_2=0;
            else
               d1_2=caldis2(points,EMD,index,ids1,SampleNum2);
               d2_2=caldis2(points,EMD,index,ids2,SampleNum2);
            end
            sd1=d1+a3*d1_2;  sd2=d2+a3*d2_2;
            if(sd1>sd2)min_id=s2;else min_id=s1;end
      %  end
       
       
     temp=leafSeeds{min_id};
     temp=[temp;index];
     leafSeeds{min_id}=temp;
     Regions2{min_id}=[Regions2{min_id};index];
     newleaf=index;
    %leafSeeds{min_id};

%    if(mod(i,1000)==0) 
%    figure('Name','LeafClassify','NumberTitle','off');set(gcf,'color','white');movegui('southwest');
%    color=[1 0 0;0 0 1;0 1 0;1 1 0;1 0 1; 0 1 1;0.5 0 0;0 0.5 0;0 0 0.5;0.5 0.5 0;0.5 0 0.5;0 0.5 0.5;0.25 0 0;0 0.25 0;0 0 0.25;0.25 0.25 0];
%    scatter3(points(:,1),points(:,2),points(:,3),5,[0 0.0 0.0], 'filled');
%    hold on;
%    scatter3(UnSegPts(:,1),UnSegPts(:,2),UnSegPts(:,3),5,[0 0 0], 'filled');
%    hold on; 
% %    scatter3(points(newleaf,1),points(newleaf,2),points(newleaf,3),40,[0 0 1], 'filled');
% %    hold on;
%     for i=1:size(leafSeeds,1)
%     I1=leafSeeds{i};
%     scatter3(points(I1,1),points(I1,2),points(I1,3),5,color(i,:), 'filled');
%     hold on;
%     end
%    axis off; axis equal; camorbit(0,0,'camera'); axis vis3d; view(0,0);view3d ZOOM;
%    end
   end
    Regions=leafSeeds';
%     if(true)
%    figure('Name','LeafClassify','NumberTitle','off');set(gcf,'color','white');movegui('southwest');
%    color=[1 0 0;0 0 1;0 1 0;1 1 0;1 0 1; 0 1 1;0.5 0 0;0 0.5 0;0 0 0.5;0.5 0.5 0;0.5 0 0.5;0 0.5 0.5;0.25 0 0;0 0.25 0;0 0 0.25;0.25 0.25 0];
%    scatter3(points(:,1),points(:,2),points(:,3),5,[0 0 0], 'filled');
%    hold on;
%     for i=1:size(leafSeeds,1)
%     I1=leafSeeds{i};
%     scatter3(points(I1,1),points(I1,2),points(I1,3),5,[rand(1,1) rand(1,1) rand(1,1)], 'filled');
%     hold on;
%     end
%    axis off; axis equal; camorbit(0,0,'camera'); axis vis3d; view(0,0);view3d ZOOM;
%    end 

end

function [min_dis]= caldis2(points,EMD,index,indices,SampleNum2)
   ids=findNNInRadius_EMD(points,EMD,index,indices,SampleNum2);
   if(length(ids)<6)
       min_dis=-1;
       return;
   end
  % rad=abs(vs(:,3))+1.5*abs(vs(:,1))+1.5*abs(vs(:,2));
   samplePts=points(ids,:);
   pt=points(index,:);
   [coeff,~,~]=pca(samplePts);
    projd=inf;
    if(size(coeff,2)==3)
    normal=coeff(:,3)';
    Spt=mean(samplePts);
    pjs=find_ProjCoord_mt(normal,Spt,pt);
    v=pjs-pt;projd=sqrt(v(1)*v(1)+v(2)*v(2)+v(3)*v(3));
    projd=abs(projd);
    end
   min_dis=projd;
end



function [min_dis]= caldis1(points,EMD,index,ids,SampleNum)
    nn=findNN_EMD(EMD,index,ids,SampleNum);
    pt=points(index,:);
    pts=points(nn,:);
    vs=pts-pt;
    rad=sqrt(sum(vs.*vs,2)); 
    min_dis=mean(rad);
end


function [nn]= findNN_EMD(EMD,index,indices,SampleNum)
    emd=EMD(index,:);
    ids=indices;
    emd_=emd(ids);
    [~,emdIndices] =sort(emd_,'descend');
    num=length(emdIndices);
    if(num>SampleNum)
         num=SampleNum;
    end
    nn=ids(emdIndices(1:num));
end
function [nn]= findNNInRadius_EMD(points,EMD,index,indices,SampleNum2)
    emd=EMD(index,:);
    ids=indices;
    emd_=emd(ids);
    [~,emdIndices] =sort(emd_,'descend');
    pts=points(ids(emdIndices),:);
    pt=points(index,:);
    vs=pts-pt;
    rad=sqrt(sum(vs.*vs,2)); 
    nn=ids(emdIndices(rad<SampleNum2));
    
  %  nn=ids(emdIndices(1:num));
end
