function [ points,Obb ] = computeOBB( points )
%COMPUTEOBB �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
  [globalDirs ,~,~]= pca(points,'Algorithm','eig');  
  gDir1=globalDirs(:,1)';
  gDir2=globalDirs(:,2)';
  gDir3=globalDirs(:,3)'; 
  center=mean(points,1);
  Axis=[center;gDir1;gDir2;gDir3];
  points=Transfer_XYZ2AXIS_mt(points,Axis);
  maxX=max(points(:,1));minX=min(points(:,1));
  maxY=max(points(:,2));minY=min(points(:,2));
  maxZ=max(points(:,3));minZ=min(points(:,3));
  Obb=[minX maxX minY maxY minZ maxZ];
end

