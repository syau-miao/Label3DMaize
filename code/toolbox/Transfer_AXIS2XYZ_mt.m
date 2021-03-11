function [ Axis_Coords] = Transfer_AXIS2XYZ_mt( XYZs_Coords,XYZ_Axis)
%% TRANSFER_XYZ2AXIS_MT 从XYZ坐标系转换到AXIS坐标系  dir1 对应AXIS-Z， dir2 对应AXIS-X dir3对应AXIS-Y
%% method=0,则XYZs_Coords为点，如果method=1非0,则为向量
 PNum=size(XYZs_Coords,1);
 Dir1=repmat(XYZ_Axis(2:4,1),[1 PNum]);
 Dir2=repmat(XYZ_Axis(2:4,2),[1 PNum]);
 Dir3=repmat(XYZ_Axis(2:4,3),[1 PNum]);
 Displace=XYZs_Coords;
 V1=dot(Displace',Dir1);
 V2=dot(Displace',Dir2);
 V3=dot(Displace',Dir3);
 Axis_Coords=[ V1' V2' V3'];
 Axis_Coords=Axis_Coords+XYZ_Axis(1,:);
 %Axis_normals=find_normal_data(Axis_Coords,NerNum);
 return
  figure('Name','AXIS COORDINATE','NumberTitle','off');set(gcf,'color','white');movegui('southwest'); 
    scatter3(Axis_Coords(:,1),Axis_Coords(:,2),Axis_Coords(:,3),5,[0 0 0], 'filled');
    hold on;
    quiver3(Axis_Coords(:,1),Axis_Coords(:,2),Axis_Coords(:,3),Axis_normals(:,1),Axis_normals(:,2),Axis_normals(:,3));
    hold on;
    center=[0 0 0];
    dir1=[0 0 1];
    dir2=[1 0 0];
    dir3=[0 1 0];
    showLine(center,dir1,'r'); 
    hold on
    showLine(center,dir2,'g'); 
    hold on
    showLine(center,dir3,'b'); 
    hold on
 
    axis off; axis equal; camorbit(0,0,'camera'); axis vis3d; view(-90,0);view3d rot;
    %showLine(xyz,dir,'r');  
end

