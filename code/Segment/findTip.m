function [ Tip ] = findTip( points,searchR,thr,sampleId,debug)
%FINDTIP 此处显示有关此函数的摘要
%   此处显示详细说明
%  Adj= BulidNeighbours(points,num_neighb);
  ptCloudIn=pointCloud(points);
 %[~,~,outlierIndices] = pcdenoise(ptCloudIn);
 fprintf('KDTREE time');
 tic
 kdtreeobj= KDTreeSearcher(points,'distance','euclidean');
 [Adj,~] = rangesearch(kdtreeobj,points,searchR);
 toc
  TipCos=zeros(size(points,1),1);
  fprintf('cos time');
  tic;
  for i=1:size(points,1)
     if(~isempty(sampleId))
        if(sampleId(i)==0)
           TipCos(i)=-Inf;
           continue;
        end
     end
     pt=points(i,:);
     nei=Adj{i};
     npts=points(nei,:);
     indices=find(npts(:,1)>pt(1));
     if(isempty(indices))
     TipCos(i)=inf;
     else
      TipCos(i)=-inf;   
     end
  end
  Tip=find(TipCos==inf);
  %Tip=setdiff(Tip,outlierIndices);
%  [dis, I]=sort(TipCos,'descend');
  %Tip=I(1:thr);
  %Tip(find(Tip)<0)=[];
  %%%%%%如果两个tip的点比较接近，则合并两个点%%%%
  TipPts=points(Tip,:);
  %Dist=pdist2(TipPts,TipPts);
%   del=[];
%    for i=1:thr-1
%     if(~isempty(find(del==i)))
%         continue;
%     end
%     pt1=TipPts(i,:);
%     for j=i+1:thr
%       pt2=TipPts(j,:);
%       v=pt1-pt2;
%       dis=norm(v);
%       if(dis<2*searchR)
%          del=[del;j]; 
%       end
%     end
%    end
%    Tip(del)=[];
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  toc
%   if(nargin==5)
%     figure('Name','Tip','NumberTitle','off');set(gcf,'color','white');movegui('southwest');
%     scatter3(points(:,1),points(:,2),points(:,3),5,[0 0 0], 'filled');
%     hold on
%     num=length(Tip);
%     r=1:-0.7/num:0.3; r(end)=[];
%     g=zeros(1,num);
%     b=zeros(1,num);
%     color=[r',g',b'];
%     scatter3(points(Tip,1),points(Tip,2),points(Tip,3),50,color, 'filled');
%     hold on
%     axis off; axis equal; camorbit(0,0,'camera'); axis vis3d; view(0,0);view3d ZOOM;
%   end
end

