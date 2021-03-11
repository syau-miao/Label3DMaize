function [ Tip ] = GetTipGUI(AXIS_POINTS,Tip)
%GETTIPGUI 此处显示有关此函数的摘要
%   此处显示详细说明
%%%%%%%%增加新的tip点%%%%%%%%%%%%%%%%%%%
 StopPoints=getPointsInFigure(AXIS_POINTS,Tip,'SelectNewTipPoints');
 if(~isempty(StopPoints))
 index=ismember(AXIS_POINTS,StopPoints,'rows');
 index=find(index==1);
 Tip=[Tip;index];
 Tip=unique(Tip);
 end
 %%%%%%%%删除错误的tip点%%%%%%%%%%%%%%%%%%% 
 StopPoints=getPointsInFigure(AXIS_POINTS,Tip,'DeleteTipPoints');
 if(~isempty(StopPoints))
 StopPoints=unique(StopPoints,'rows');
 TipPoints =AXIS_POINTS(Tip,:);
 [Dis,I]=pdist2(TipPoints,StopPoints,'euclidean','Smallest',1);
 I=unique(I);
 Tip(I)=[];
 end
 
 if(true)
  figure('Name','tip result','NumberTitle','off');
  set(gcf,'color','white');
  %movegui('southwest');
  scatter3(AXIS_POINTS(:,1),AXIS_POINTS(:,2),AXIS_POINTS(:,3),10,[0 0 0],'filled');
  hold on;
  scatter3(AXIS_POINTS(Tip,1),AXIS_POINTS(Tip,2),AXIS_POINTS(Tip,3),30,[1 0 0],'filled');
  hold on;
  axis off; axis equal; camorbit(0,0,'camera'); axis vis3d; view(90,90);view3d ZOOM;
 end
end

