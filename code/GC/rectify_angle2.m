
%  Copyright (c) 2013
%      Oliver van Kaick <ovankaic@gmail.com>

%
% Rectify angle according to triangle normals and vertices
%
function [angle, inconsistency] = rectify_angle2(angle, o1, o2, norm1, norm2)

    % Rectify angle: acos() always returns an angle betwen [0, pi],
    % so we have to find out when we have an angle > pi
    %
    % The test to rectify the angle is the following: imagine the
    % edge 'e' seen from the side (the edge is pointing into the
    % monitor). The edge has two neighboring faces 'f1' and 'f2'
    % also seen from the side. These two faces have two vertices
    % 'o1' and 'o2' that they don't share with 'e'. So, we use the
    % vector from 'o1' and 'o2' and compute its angle with the
    % normals of 'f1' and 'f2' which are 'norm1' and 'norm2'. We can
    % have two cases:
    %       
    % __    __ Outside
    %   \  /   Inside
    %    \/
    %
    % o1 ----> o2
    %    \  /
    %  f1 \/ f2
    %     e
    % Case 1: cos(o2-o1, norm1) > 0 (angle less than pi) and
    % cos(o2-o1, norm2) < 0 (angle more than pi)
    % Result: angle between 'norm1' and 'norm2' is fine
    %
    %    /\
    % __/  \__ Outside
    %          Inside
    %
    %     e 
    %  f1 /\ f2
    %    /  \
    % o1 ----> o2
    %
    % Case 2: cos(o2-o1, norm1) < 0 and cos(o2-o1, norm2) > 0
    % Result: angle between 'norm1' and 'norm2' is actually an
    % obtuse angle, so we have to increment it by pi

    % Compute direction vector for opposed vertices
    opdir = o2 - o1;

    % Compute dotr product between direction and normals
    dp1 = sum(norm1.*opdir); % dot product
    dp2 = sum(norm2.*opdir); % dot product

    % Now, rectify angle based on dot products 1 and 2
    if dp1 > 0 && dp2 < 0
        ; % Angle is ok
    elseif dp1 < 0 && dp2 > 0
        %angle = angle * -1; % Invert angle
        angle = angle + pi(); % Add pi() to make it an obtuse angle 
    else
        %angle = angle + pi(); % Add pi() to make it an obtuse angle 
        % If we have both dp1 and dp2 of the same sign, there
        % must be an inconsistency in the mesh
        inconsistency = 1;
        angle = pi();
    end
end
