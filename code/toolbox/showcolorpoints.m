function showcolorpoints( points,colors )
%SHOWCOLORPOINTS �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
     figure('Name','points and colors','NumberTitle','off');set(gcf,'color','white');movegui('southwest');
     scatter3(points(:,1),points(:,2),points(:,3),10,colors, 'filled');
     axis off; axis equal; camorbit(0,0,'camera'); axis vis3d; view(-90,0);view3d rot;
   
end

