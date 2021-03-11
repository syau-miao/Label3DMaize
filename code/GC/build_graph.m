
%  Copyright (c) 2013
%      Noa Fish <noafish@post.tau.ac.il>

%%% build KNN graph

%%% input:
% points: 3D point set
% k: number of neighbors

%%% output:
% gr: KNN graph
% adj: point adjacency matrix
% max_dist: maximum distance between neighbors
% outliers: outlier points

function [gr, adj, max_dist, outliers] = build_graph(points, k)

nump = size(points,1);
[IDX, dists] = knnsearch(points,points,'K',k);
IDX = IDX(:,2:end);
dists = dists(:,2:end);

p = 1:nump;
x = kron(p,ones(k-1,1));
x = reshape(x, size(x,1)*size(x,2),1);
y = reshape(IDX',size(IDX,1)*size(IDX,2),1);
d = reshape(dists',size(dists,1)*size(dists,2),1);
tmp = [x y];
tmp = sort(tmp,2);
[tmp2, i, ~] = unique(tmp,'rows');
d = d(i);
D = sparse(tmp2(:,1),tmp2(:,2),d,nump,nump,2*length(d));
adj = sparse(x,y,ones(size(x)),nump,nump,2*length(x));

adj = adj | adj';

[nzx nzy nzv] = find(D);
symind = sub2ind(size(D), nzy, nzx);
D(symind) = nzv;

max_dist = max(max(D));

% build gr - a cell array containing the closest points to each point, in
% order of ascending distance
gr = cell(size(points,1),1);

for i=1:size(points,1)
    nz = find(D(i,:));
    dsts = D(i,nz);
    [~, sidx] = sort(dsts);
    gr{i} = nz(sidx);   
end

% calc outliers
outliers = [];

for i=1:size(points,1)
    
    neigh = gr{i};
    closest_neigh = neigh(1);
    nneigh = gr{closest_neigh};
    % If the closest neighbor of point i doesn't have point i as one of its
    % top 5 closest neighbors then point i is an outlier
%     if(isempty(find(nneigh(1:5) == i)))
%         outliers = [outliers; i];
%     end
end


end


