function [ Pos ] = getPointsInFigure(data,tip,figureName)
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
 % camorbit(0,0,'camera'); axis vis3d; view(90,90);view3d ZOOM;
  
  %set(fig,'windowkeypressfcn',@keypressfcn);
  set(hFigure,'windowkeyreleasefcn',@keyreleasefcn);
  datacursormode on;
  dcm_obj = datacursormode(hFigure);
 % dcm_obj.Enable='off';
  Pos=[];
while(1)
    if strcmpi(get(gcf,'CurrentCharacter'),'q')
       close(hFigure);
       break;
    end
  pause;
  c_info = getCursorInfo(dcm_obj);
  if(~isempty(c_info))
  pos=c_info.Position;
  Pos=[Pos;pos];
  scatter3(Pos(:,1),Pos(:,2),Pos(:,3),35,[0 0 1],'filled');
  hold on;
   end
  end
end
 function keyreleasefcn(h,evt)
       % fprintf('************release \n');
        if(evt.Character=='o')
          % fprintf('oooooo\n'); 
           datacursormode;
        end
       % fprintf('************ \n');
    end





