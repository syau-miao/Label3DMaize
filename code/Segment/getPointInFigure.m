function [ pos ] = getPointInFigure(data,tip,figureName)
%GETPOINTINFIGURE 此处显示有关此函数的摘要
%   此处显示详细说明
  hFigure= figure('Name',figureName,'NumberTitle','off');
  set(gcf,'color','white');
  %movegui('southwest');
  scatter3(data(:,1),data(:,2),data(:,3),10,[0 0 0],'filled');
  hold on;
  scatter3(data(tip,1),data(tip,2),data(tip,3),30,[1 0 0],'filled');
  hold on;
  axis off; axis equal; 
  %camorbit(0,0,'camera'); axis vis3d; view(90,90);view3d ZOOM;
   set(hFigure,'windowkeyreleasefcn',@keyreleasefcn);
  datacursormode on;
  dcm_obj = datacursormode(hFigure);
  pause;
  pos=[];
  while(1)
    if strcmpi(get(gcf,'CurrentCharacter'),'q')
       close(hFigure);
       break;
    end
    pause;
    c_info = getCursorInfo(dcm_obj);
    if(~isempty(c_info))
       pos=c_info.Position;
       scatter3(pos(:,1),pos(:,2),pos(:,3),35,[0 0 1],'filled');
       hold on;
    end
  end
end
 function keyreleasefcn(h,evt)
        if(evt.Character=='o')
           datacursormode;
        end
end

