function [ AXIS_POINTS ] = PlantCoordinate( points,stemIds,v )
%PLANTCOORDINATE 此处显示有关此函数的摘要
%   此处显示详细说明
StemPoints=points(stemIds,:);
 [StemDirs ,~,~]= pca(StemPoints);  
 StemDir=StemDirs(:,1)';
 if(dot(v,StemDir)<0)
     StemDir=-StemDir;
 end
 StemCenter=mean(StemPoints,1);
 AXIS=find_AxisByPrincipalDir_mt(points,StemDir,StemCenter,false);
 [AXIS_POINTS ]=Transfer_XYZ2AXIS_mt(points,AXIS);
end

