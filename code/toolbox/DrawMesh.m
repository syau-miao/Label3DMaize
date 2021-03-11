function  DrawMesh( newPts,tri )
%DRAWMESH 此处显示有关此函数的摘要
%   此处显示详细说明
figure;
scatter3(newPts(:,1),newPts(:,2),newPts(:,3),50,[0 0 1], 'filled');
   hold on;
   
for i=1:size(tri,1)
   ids =tri(i,:); 
   ids=ids;
   %ids=ids+[1 1 1];
   color=[1 0 0];
   idx=[ids(1);ids(2)];
   line( newPts(idx,1),newPts(idx,2),newPts(idx,3), 'LineWidth', 2, 'Color', color); 
   idx=[ids(1);ids(3)];
   line( newPts(idx,1),newPts(idx,2),newPts(idx,3), 'LineWidth', 2, 'Color', color);
   idx=[ids(2);ids(3)];
   line( newPts(idx,1),newPts(idx,2),newPts(idx,3), 'LineWidth', 2, 'Color', color);
end
axis off; axis equal; camorbit(0,0,'camera'); axis vis3d; view(0,0);view3d ZOOM;

end

