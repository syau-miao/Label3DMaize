function [features] = computeMyFeature_mt(InputPts,searchR,varargin)
%COMPUTEMYFEATURE_MT 此处显示有关此函数的摘要
   ptCloud = pointCloud(InputPts);
   features=zeros(size(InputPts,1),3);
   kdtreeobj= KDTreeSearcher(InputPts,'distance','euclidean');
   [Adj,dis] = rangesearch(kdtreeobj,InputPts,searchR);
   for i=1:size(InputPts,1)
     indices=Adj{i};
     n_pts=InputPts(indices,:);
     [~ ,~,coffes]= pca(n_pts);
     if(length(coffes)<3)
       features(i,:)=[0 0 0];  
       continue;
     end
     feat=[coffes(1)/sum(coffes) coffes(2)/sum(coffes) coffes(3)/sum(coffes)];
     features(i,:)=[feat(3)/feat(2) feat(3) feat(2)];   
   end   
    if(nargin>2)
     figure('Name','feature','NumberTitle','off');set(gcf,'color','white');movegui('southwest');
     scatter3(InputPts(:,1),InputPts(:,2),InputPts(:,3),5,features, 'filled');
     axis off; axis equal; camorbit(0,0,'camera'); axis vis3d; view(-90,0);view3d rot;      
    end
end

