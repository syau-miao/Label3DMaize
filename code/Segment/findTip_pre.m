function [ Tip ] = findTip( points,searchR,thr,debug)
%FINDTIP 此处显示有关此函数的摘要
%   此处显示详细说明
%  Adj= BulidNeighbours(points,num_neighb);

 PtsSz = length(points);
 kdtreeobj= KDTreeSearcher(points,'distance','euclidean');
 [Adj,~ ] = rangesearch(kdtreeobj,points,searchR);
  Tip=[];
  TipCos=[];
  for i=1:size(points,1)
     pt=points(i,:);
     nei=Adj{i};
     npts=points(nei,:);
     dir=(npts-pt);
     for j=2:length(nei)
        dir(j,:)=dir(j,:)./norm(dir(j,:)); 
     end
     cosM=dir*dir'; 
     cosM(:,1)=[];
     cosM(1,:)=[];
     maxCos=max(cosM(:));
     minCos=min(cosM(:));
     if(minCos>thr)
         Tip=[Tip;i];
         TipCos=[TipCos;minCos];
     end
  end
  [~, I]=sort(TipCos,'descend');
  Tip=Tip(I);
  if(nargin==4)
    figure('Name','Tip','NumberTitle','off');set(gcf,'color','white');movegui('southwest');
    scatter3(points(:,1),points(:,2),points(:,3),5,[0 0 0], 'filled');
    hold on
    num=length(Tip);
    r=1:-0.7/num:0.3; r(end)=[];
    g=zeros(1,num);
    b=zeros(1,num);
    color=[r',g',b'];
    scatter3(points(Tip,1),points(Tip,2),points(Tip,3),50,color, 'filled');
    hold on
    axis off; axis equal; camorbit(0,0,'camera'); axis vis3d; view(0,0);view3d ZOOM;
  end
end

