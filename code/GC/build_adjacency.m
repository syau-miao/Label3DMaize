
%  Copyright (c) 2013
%      Oliver van Kaick <ovankaic@gmail.com>

%
% Build adjacency matrix for graph cuts labeling of a point cloud
%
% Input:
%   - points: <n x 3> matrix of points
%   - normals: <n x 3> matrix of normals (one corresponding to each point)
%   - gr: neighborhood relations, where gr{i} is a list of the neighbors
%   of point 'i', as returned by build_graph
%   - len_pow: power to weight length term
%   - ang_pow: power to weight angle term
%   - norm_avg: if 1, the normal of a point is given by the average of
%   the normals of its neighbors. If 0, the normal of the point is taken
%   directly from 'normals'
%
% Output:
%   - sp: sparse matrix representing the adjacency graph of the point
%   cloud.  Each entry is the edge length * the angle between the point
%   normals, weighted by len_pow and ang_pow
%
function sp = build_adjacency(points, normals, gr, len_pow, ang_pow, norm_avg, point_adj, max_elen)

    % Build a sparse adjacency matrix based on edge length and angle
    % There could be a faster way of doing this
    %sp = sparse(n, n);
    
    sp = double(point_adj);
    n = size(points, 1);
    
    for i = 1:n
        % Initialize sparse matrix with 1 in all entries
        for j = 1:length(gr{i})
            % Edge between p1 and p2
            p1 = i;
            p2 = gr{i}(j);
            % Compute edge length
            elen = norm(points(p1, :) - points(p2, :), 2);
            elen = elen / max_elen;
            % Compute angle between normals
            % Select the normals
            if norm_avg
                vec1 = mean(normals([p1 gr{p1}], :));
                vec2 = mean(normals([p2 gr{p2}], :));
            else
                vec1 = normals(p1, :);
                vec2 = normals(p2, :);
            end
            % Normalize the vectors
            vec1 = vec1 ./ norm(vec1, 2);
            vec2 = vec2 ./ norm(vec2, 2);
            dp = sum(vec1 .* vec2);
            % Avoid complex result
            if dp > 1.0
                dp = 1.0;
            elseif dp < -1.0
                dp = -1.0;
            end
            eang = acos(dp); % Angle between normals
            eang = rectify_angle2(eang, points(p1, :), points(p2, :), vec1, vec2);
            % Use a simpler angle term
            ang = eang / (2*pi());
            
            if(ang > 0.5)
                ang = 1;
            else if(ang == 0.5)
                    ang = 0.01;
                else
                    ang = (0.5-ang) .^ 6;
                end
            end
            % Set graph
            sp(p1, p2) = (elen^len_pow)*(ang^ang_pow);
        end
    end
end
