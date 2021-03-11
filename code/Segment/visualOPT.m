clear all; close all;
% [filename,pathname] = uigetfile('.ply','Select the ply point cloud file');  
% if(filename==0)
%    return; 
% end
%points;
 points=load('Ê¾Àý¡ª¡ªÏ¸·Ö¸îlabel.txt');
% pccloud=pcread([pathname filename]);
% points=double(pccloud.Location);
 points=points(:,1:3);
EMD=computeEMD(points);
index=12193;
%index=500;

neiNum=32;
nn1=findNN_emd(EMD,index,neiNum);
nn=BulidNeighbours(points,neiNum);
nn2=nn(index,2:neiNum+1);
nn3=intersect(nn1,nn2);
figure;
scatter3(points(:,1),points(:,2),points(:,3),30,[0 0 0], 'filled');
hold on;
scatter3(points(index,1),points(index,2),points(index,3),30,[1 0 0], 'filled');
hold on;
scatter3(points(nn1,1),points(nn1,2),points(nn1,3),30,[0 1 0], 'filled');
hold on;
scatter3(points(nn2,1),points(nn2,2),points(nn2,3),30,[0 0 1], 'filled');
hold on;
scatter3(points(nn3,1),points(nn3,2),points(nn3,3),30,[1 0 1], 'filled');
hold on;
axis equal; camorbit(0,0,'camera'); axis vis3d; view(0,0);view3d ZOOM;



function [nn]= findNN_emd(EMD,index,SampleNum)
    emd=EMD(index,:);
    emd_=emd;
    [~,emdIndices] =sort(emd_,'descend');
    num=length(emdIndices);
    if(num>SampleNum)
         num=SampleNum;
    end
    nn=emdIndices(2:num+1);
end




