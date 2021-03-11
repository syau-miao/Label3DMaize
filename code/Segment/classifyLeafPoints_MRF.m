function [ FinalRegions ] = classifyLeafPoints_MRF(points,normals,Regions,UnSegIds,lamada,KNum,len_pow,ang_pow)
%REMOVEFLOWERSPOT 此处显示有关此函数的摘要
  pts=points(UnSegIds,:);
  nrms=normals(UnSegIds,:);
%   kdtreeobj0= KDTreeSearcher(pts,'distance','euclidean');
%   Indices=knnsearch(kdtreeobj0,pts,'k',KNum);
%   pnum=length(UnSegIds);
%   nb = sparse(pnum,pnum);
  [gr, point_adj, max_elen, outliers] =build_graph(pts,KNum);
 % weight = 0.1; % Weight assigned to make data cost bigger/smaller than smoothness cost
 % len_pow = 1; % Power to weight the edge length term
 % ang_pow = 1; % Power to weight the edge angle term
   norm_avg = 0; % Use average of neighbors as point normal
gc_adjacency = build_adjacency(pts, nrms, gr, len_pow, ang_pow, norm_avg, point_adj, max_elen);

    l=length(Regions);
   smoothness = zeros(l, l);
    for i = 1:l
        for j = 1:l
            smoothness(i, j) = (i ~= j);
        end
    end
  
  datacost=zeros(length(pts),length(Regions));
  for i=1:length(Regions)
     region= Regions{i};
     pts1=points(region,:);
     kdtreeobj1= KDTreeSearcher(pts1,'distance','euclidean');
     [~,D1]=knnsearch(kdtreeobj1,pts,'k',1);
     datacost(:,i)=D1;
  end
  datacost=datacost';
  disp('Running graph cut'); drawnow('update');
  gch = GraphCut('open', lamada*datacost, smoothness, gc_adjacency);
 [gch graphcut_idx] = GraphCut('expand', gch);
  gch = GraphCut('close', gch);
 graphcut_idx = graphcut_idx';
 graphcut_idx = double(graphcut_idx)+1;
  for i=1:length(Regions)
     FinalRegions{i}= UnSegIds(graphcut_idx==i);
  end
end

