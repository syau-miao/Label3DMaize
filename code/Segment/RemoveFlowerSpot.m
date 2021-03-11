function [ output_args ] = RemoveFlowerSpot(points,colors)
%REMOVEFLOWERSPOT 此处显示有关此函数的摘要
%   此处显示详细说明
%调用Kmeans函数
%X N*P的数据矩阵
%Idx N*1的向量,存储的是每个点的聚类标号
%Ctrs K*P的矩阵,存储的是K个聚类质心位置
%SumD 1*K的和向量,存储的是类间所有点与该类质心点距离之和
%D N*K的矩阵，存储的是每个点与所有质心的距离;
  hsv=rgb2hsv(colors);
  KNum=32;
  [Idx,Ctrs,SumD,Data] = kmeans(hsv,2);
  pnum=length(colors);
%  kdtree = kdtree_build(points);% kdtree,用来找k近邻
 % Indices = zeros(pnum, KNum);
   kdtreeobj= KDTreeSearcher(points,'distance','euclidean');
   %[n,~ ] = rangesearch(kdtreeobj,Pts,20);
  Indices = knnsearch(kdtreeobj,points,'k',KNum);
%   for i=1:pnum
%   Indices(i,:)= kdtree_k_nearest_neighbors(kdtree,points(i,:),KNum)';
%   end
  nb = sparse(pnum,pnum);
  edgeNum=0;
 
  for i=1:pnum
     indices=Indices(i,:);
     ids=indices(find(indices>i));
     if(length(ids)==0)continue;end;
     nb(i,ids)=0.5;
     edgeNum=edgeNum+length(ids);
  end
 hinc = BK_Create(pnum,edgeNum);
 BK_SetUnary(hinc,Data'); 
 BK_SetNeighbors(hinc,nb);
 e_inc = BK_Minimize(hinc);
 label = BK_GetLabeling(hinc);
 if(true)
   figure('Name','removespot','NumberTitle','off');set(gcf,'color','white');movegui('southwest');
   indices1=find(label==1);
   scatter3(points(indices1,1),points(indices1,2),points(indices1,3),10,[1 0 0], 'filled');
   hold on;
   indices1=find(label==2);
   scatter3(points(indices1,1),points(indices1,2),points(indices1,3),10,[0 0 1], 'filled');
   hold on;
   axis off; axis equal; camorbit(0,0,'camera'); axis vis3d; view(-90,0);view3d rot;
 end 
  
end

