function [ SeedId ,StopId] = GetStemGUI(AXIS_POINTS,SeedId)
%GETTIPGUI �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
%%%%%%%%������ʼ��%%%%%%%%%%%%%%%%%%%
 StopPoints=getPointInFigure(AXIS_POINTS,SeedId,'SelectBottomStem');
 %SeedId=Tip;
 if(~isempty(StopPoints))
 index=ismember(AXIS_POINTS,StopPoints,'rows');
 SeedId=find(index==1);
 end
 %%%%%%%%�õ���ֹ��%find(index==1)%%%%%%%%%%%%%%%%%% 
 StopPoints=getPointInFigure(AXIS_POINTS,[],'SelectTopStem');
 StopId=SeedId;
 if(~isempty(StopPoints))
 index=ismember(AXIS_POINTS,StopPoints,'rows');
 StopId=find(index==1);
 end
 
%  if(true)
%   figure('Name','tip result','NumberTitle','off');
%   set(gcf,'color','white');
%   %movegui('southwest');
%   scatter3(AXIS_POINTS(:,1),AXIS_POINTS(:,2),AXIS_POINTS(:,3),10,[0 0 0],'filled');
%   hold on;
%   scatter3(AXIS_POINTS(Tip,1),AXIS_POINTS(Tip,2),AXIS_POINTS(Tip,3),30,[1 0 0],'filled');
%   hold on;
%   axis off; axis equal; camorbit(0,0,'camera'); axis vis3d; view(90,90);view3d ZOOM;
%  end
end

