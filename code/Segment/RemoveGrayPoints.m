function [ points_,colors_] = RemoveGrayPoints(points, colors )
%REMOVEGRAYPOINTS �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
   hsv=rgb2hsv(colors);
   indices=find(hsv(:,2)>0.05);
   points_=points(indices,:);
   colors_=colors(indices,:);
  
end

