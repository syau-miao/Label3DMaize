function mesh = pointCloud2rawMesh(data, stdTol, correctNormals)

% mesh = pointCloud2rawMesh(data, stdTol, correctNormals)
% 
%
% Author : Ajmal Saeed Mian {ajmal@csse.uwa.edu.au}
%           Computer Science. Univ of Western Australia
%
% This function is a cut down version of pointCloud2mesh.m  
%
% Arguments : data - Nx3 array of x,y,z coordinates
%             stdTol - (optional) stdDev tolerance to filter edges
%                       default is 0.6
%             correctNormals - (optional) set it to 1 if you want all 
%                           triangles to face towards the sensor
%
% Return : mesh - mesh data structure with 
%               mesh.vertices
%               mesh.triangles
%               mesh.resolution
%               mesh.stdeviation
%          [optionally] mesh.triangleNormals
%
% Copyright : This code is written by Ajmal Saeed Mian {ajmal.mian@uwa.edu.au}
%              Computer Science, The University of Western Australia. The code
%              may be used, modified and distributed for research purposes with
%              acknowledgement of the author and inclusion this copyright information.
%
% Disclaimer : This code is provided as is without any warrantly.

%data = data*princomp(data);
 

tri = delaunay(data(:,1),data(:,2));
tri(:,4) = 0; % initialize 4th column to store maximum edge length

edgeLength = [sqrt(sum((data(tri(:,1),:) - data(tri(:,2),:)).^2,2)),...
        sqrt(sum((data(tri(:,2),:) - data(tri(:,3),:)).^2,2)),...
        sqrt(sum((data(tri(:,3),:) - data(tri(:,1),:)).^2,2))];

tri(:,4) = max(edgeLength,[],2);

if nargin<2
    stdTol = 0.6;
end

if stdTol > 0
    resolution = mean(edgeLength(:));
    stdeviation = std(edgeLength(:));
    filtLimit = resolution + stdTol*stdeviation;
    
    bigTriangles = find(tri(:,4) > filtLimit); %find index numbers of triagles with edgelength more than filtLimit
    tri(bigTriangles,:) = []; % remove all triangles with edgelength more than filtlimit
    tri(:,4) = []; % remove the max edgeLength column    
    
    edgeLength(bigTriangles,:) = []; % remove edges belonging to triangles which are removed       
end

resolution = mean(edgeLength(:)); % find the mean of the remaining edges
stdeviation = std(edgeLength(:));

mesh.vertices = data;  
mesh.triangles = tri;
mesh.resolution = resolution;
mesh.stdeviation = stdeviation;

if nargin < 3 
    return;
end

if ~correctNormals
    return;
end
% correct normals of polygons
refNormal = [0 0 1];
for ii = 1:length(mesh.triangles)
    %indices of the points from which the polygon is made
    pointIndex1 = mesh.triangles(ii,1);
    pointIndex2 = mesh.triangles(ii,2);
    pointIndex3 = mesh.triangles(ii,3);
    
    %coordinates of the points
    point1 = mesh.vertices(pointIndex1,:);
    point2 = mesh.vertices(pointIndex2,:);
    point3 = mesh.vertices(pointIndex3,:);
    
    vector1 = point2 - point1;
    vector2 = point3 - point2;
    
    normal = cross(vector1,vector2);
    normal = normal / norm(normal);
    
    theta = acos(dot(refNormal, normal));
    if theta > pi/2
        normal = normal * (-1);
        a = mesh.triangles(ii,2);
        mesh.triangles(ii,2) = mesh.triangles(ii,1);
        mesh.triangles(ii,1) = a;
    end    
    mesh.triangleNormals(ii,:)=normal;        
end