function [ Regions ] = classifyLeafPoints_mt3( points,normals,leafSeeds,unSegment ,SampleNum,SampleNum2,a3,times)
%CLASSIFYLEAFPOINTS_MT 此处显示有关此函数的摘要
%   此处显示详细说明
   leafNum=size(leafSeeds,1);
   leafSegment=cell(leafNum,1);
   UnSegPts=points(unSegment,:);
   UnSegNormal=normals(unSegment,:);
   UnSegPts_z=UnSegPts(:,1);
   UnSegPts_x=UnSegPts(:,2);
   UnSegPts_y=UnSegPts(:,3);
   EFF=UnSegPts_z+0*UnSegPts_x+0*UnSegPts_y;
   [sortZ, SortIndices]=sort(EFF,'descend');
   if(false)
   figure('Name','RoughSegment','NumberTitle','off');set(gcf,'color','white');movegui('southwest');
   color=[1 0 0;0 0 1;0 1 0;1 1 0;1 0 1; 0 1 1;0.5 0 0;0 0.5 0;0 0 0.5;0.5 0.5 0;0.5 0 0.5;0 0.5 0.5;0.25 0 0;0 0.25 0;0 0 0.25;0.25 0.25 0];
   scatter3(points(:,1),points(:,2),points(:,3),5,[0.0 0.0 0], 'filled');
   hold on;
    for i=2:size(leafSeeds,1)
    I1=leafSeeds{i};
    scatter3(points(I1,1),points(I1,2),points(I1,3),5,[rand(1,1) rand(1,1) rand(1,1)], 'filled');
    hold on;
    end
   axis off; axis equal; camorbit(0,0,'camera'); axis vis3d; view(0,0);view3d ZOOM;
   end 




   for i=1:size(SortIndices,1)
       index=SortIndices(i); 
       pt=UnSegPts(index,:);
       pn=UnSegNormal(index,:);
       min_dis=inf;
       min_id=-1;
       Dis1=zeros(leafNum,1); Dis2=zeros(leafNum,1);
       a1=1.0;a2=1.0;%SampleNum=5;SampleNum2=5;
       a3=0.0;
       for j=times:leafNum
          ids=leafSeeds{j};
          leafPts=points(ids,:);
       %   leafNormals=normals(ids,:);        
          Dis1(j)=caldis1(leafPts,pt,SampleNum); 
      end
     [sortDis1,SortIndices1]=sort(Dis1,'ascend');
     s1=SortIndices1(1); s2=SortIndices1(2);
     d1=sortDis1(1);     d2=sortDis1(2);
   %  d1_2=Dis2(s1);       d2_2=Dis2(s2);
%         if(s1==1)
%            min_id=s1;
%         else
%         if(abs(d1-d2)/d1>a3)
%            min_id=s1; 
%         else
           ids1=leafSeeds{s1};
           leafPts1=points(ids1,:);
           ids2=leafSeeds{s2};
           leafPts2=points(ids2,:);
%             if(size(leafPts2,1)<4||size(leafPts2,1)<4)
%                d1_2=0;
%                d2_2=0;
%             else
%              d1_2=caldis2(leafPts1,pt,SampleNum2);
%              d2_2=caldis2(leafPts2,pt,SampleNum2);
%             end
            d1_2=caldis3(normals(ids1,:),leafPts1,pn,pt,SampleNum*5);
            d2_2=caldis3(normals(ids2,:),leafPts2,pn,pt,SampleNum*5);
            a1=1.0; a2=a3;
           % sd1=a1*d1+a2*d1_2;  sd2=a1*d2+a2*d2_2;
            sd1=d1*(1+d1_2);  sd2=d2*(1+d2_2);
            if(sd1>sd2)min_id=s2;else min_id=s1;end
     %   end
      
     temp=leafSeeds{min_id};
     temp=[temp;unSegment(index)];
     leafSeeds{min_id}=temp;
     newleaf=unSegment(index);
    %leafSeeds{min_id};

%    if(mod(i,1000)==0) 
%    figure('Name','LeafClassify','NumberTitle','off');set(gcf,'color','white');movegui('southwest');
%    color=[1 0 0;0 0 1;0 1 0;1 1 0;1 0 1; 0 1 1;0.5 0 0;0 0.5 0;0 0 0.5;0.5 0.5 0;0.5 0 0.5;0 0.5 0.5;0.25 0 0;0 0.25 0;0 0 0.25;0.25 0.25 0];
%    scatter3(points(:,1),points(:,2),points(:,3),5,[0 0.0 0.0], 'filled');
%    hold on;
%    scatter3(UnSegPts(:,1),UnSegPts(:,2),UnSegPts(:,3),5,[0 0 0], 'filled');
%    hold on; 
% %    scatter3(points(newleaf,1),points(newleaf,2),points(newleaf,3),40,[0 0 1], 'filled');
% %    hold on;
%     for i=1:size(leafSeeds,1)
%     I1=leafSeeds{i};
%     scatter3(points(I1,1),points(I1,2),points(I1,3),5,color(i,:), 'filled');
%     hold on;
%     end
%    axis off; axis equal; camorbit(0,0,'camera'); axis vis3d; view(0,0);view3d ZOOM;
%    end
   end
    Regions=leafSeeds';
%     
%     for i=1:size(leafSeeds,1)
%        indices=leafSeeds{i};
%        findNoiseInLeaf_mt( points(indices,:));
%     end
   


%%%%%%%%%%%%%%%%%debug for rough jieguo%%%%%%%%%%%%%%
   








    
%     if(true)
%    figure('Name','LeafClassify','NumberTitle','off');set(gcf,'color','white');movegui('southwest');
%    color=[1 0 0;0 0 1;0 1 0;1 1 0;1 0 1; 0 1 1;0.5 0 0;0 0.5 0;0 0 0.5;0.5 0.5 0;0.5 0 0.5;0 0.5 0.5;0.25 0 0;0 0.25 0;0 0 0.25;0.25 0.25 0];
%    scatter3(points(:,1),points(:,2),points(:,3),5,[0 0 0], 'filled');
%    hold on;
%     for i=1:size(leafSeeds,1)
%     I1=leafSeeds{i};
%     scatter3(points(I1,1),points(I1,2),points(I1,3),5,[rand(1,1) rand(1,1) rand(1,1)], 'filled');
%     hold on;
%     end
%    axis off; axis equal; camorbit(0,0,'camera'); axis vis3d; view(0,0);view3d ZOOM;
%    end 

end



% function [min_dis]= caldis2(points,pt,SampleNum)
%    vs=points-pt;
%    rad=sqrt(sum(vs.*vs,2));
%   % rad=abs(vs(:,3))+1.5*abs(vs(:,1))+1.5*abs(vs(:,2));
%    [sortr,sortIndices]=sort(rad);
%    num=size(sortr,1);
%    sampleNum=SampleNum;
%    if(num<sampleNum)
%        sampleNum=num;
%    end
%    indices=sortIndices(1:sampleNum);
%    samplePts=points(indices,:);
%    [coeff,~,~]=pca(samplePts);
%     projd=10;
%     if(size(coeff,2)==3)
%     normal=coeff(:,3)';
%     Spt=mean(samplePts);
%     pjs=find_ProjCoord_mt(normal,Spt,pt);
%   %  pjs2=find_ProjCoord_mt(normal,Spt,samplePts);
%   %  vs=pjs2-samplePts;
%   %  projds=sqrt(sum(vs.*vs,2));
%   %  mprojds=mean(projds);
%     v=pjs-pt;projd=sqrt(v(1)*v(1)+v(2)*v(2)+v(3)*v(3));
%   %  projd=abs(projd-mprojds);
%     projd=abs(projd);
%     end
%    min_dis=projd;
% end

function [min_dis]= caldis2(points,pt,searchR)
   vs=points-pt;
   rad=sqrt(sum(vs.*vs,2));
   ids=find(rad<searchR);
   if(length(ids)<6)
       min_dis=-1;
       return;
   end
  % rad=abs(vs(:,3))+1.5*abs(vs(:,1))+1.5*abs(vs(:,2));
   samplePts=points(ids,:);
   [coeff,~,~]=pca(samplePts);
    projd=inf;
    if(size(coeff,2)==3)
    normal=coeff(:,3)';
    Spt=mean(samplePts);
    pjs=find_ProjCoord_mt(normal,Spt,pt);
    v=pjs-pt;projd=sqrt(v(1)*v(1)+v(2)*v(2)+v(3)*v(3));
    projd=abs(projd);
    end
   min_dis=projd;
end



function [min_dis]= caldis1(points,pt,SampleNum)
   vs=points-pt;
   rad=sqrt(sum(vs.*vs,2));
  % rad=abs(vs(:,3))+1.5*abs(vs(:,1))+1.5*abs(vs(:,2));
   [sortr,sortIndices]=sort(rad);
   num=size(sortr,1);
   sampleNum=SampleNum;
   if(num>sampleNum)
   min_dis=mean(sortr(1:sampleNum));
   else
   min_dis=mean(sortr(1:num)); 
   end
end


function [sum_ang]= caldis3(normals,points,pn,pt,SampleNum)
   vs=points-pt;
   rad=sqrt(sum(vs.*vs,2));
  % rad=abs(vs(:,3))+1.5*abs(vs(:,1))+1.5*abs(vs(:,2));
   [sortr,sortIndices]=sort(rad);
   num=size(sortr,1);
   sampleNum=SampleNum;
   ids=[];
   if(num>sampleNum)
   ids=sortIndices(1:sampleNum);
   else
   ids=sortIndices(1:num);
   end;
   sum_ang=0;
   for j=1:length(ids)
     vec1 = normals(ids(j), :);
     vec2 = pn;
     vec1 = vec1 ./ norm(vec1, 2);
     vec2 = vec2 ./ norm(vec2, 2);
     dp = sum(vec1 .* vec2);
            % Avoid complex result
     if dp > 1.0
         dp = 1.0;
     elseif dp < -1.0
         dp = -1.0;
     end
     eang = acos(dp); % Angle between normals
   %  eang = rectify_angle2(eang, points(ids(j), :), pt, vec1, vec2);
            % Use a simpler angle term
     ang = eang / (2*pi());
     
%      if(ang > 0.5)
%         ang = 1;
%      elseif(ang == 0.5)
%           ang = 0.01;
%      else
%           ang = (0.5-ang) .^ 6;
%       end
           
     
     sum_ang=sum_ang+ang;
   end
   sum_ang=sum_ang./length(ids);
end




% function [min_dis]= caldis2(leafPts,pt,SampleNum)
%  LX=leafPts(:,1); LY=leafPts(:,2);
% % [coeff,~,~]=pca(samplePts);  
%  figure('Name','Leafcurve','NumberTitle','off');set(gcf,'color','white');movegui('southwest');
%     scatter(LX,LY,5,[1 0 0], 'filled');
%     hold on;
%    axis off; axis equal; camorbit(0,0,'camera'); axis vis3d; view(0,0);view3d ZOOM;   
%    %min_dis=mean(sortr(1:10));
%    min_dis=-1;
% end

function [xc,yc,R,a] = circfit(x,y)
%圆拟合函数
%CIRCFIT Fits a circle in x,y plane
% [XC, YC, R, A] = CIRCFIT(X,Y)
% Result is center point (yc,xc) and radius R.A is an
% optional output describing the circle’s equation:
%
% x^2+y^2+a(1)*x+a(2)*y+a(3)=0
% by Bucher izhak 25/oct/1991
n=length(x); xx=x.*x; yy=y.*y; xy=x.*y;
A=[sum(x) sum(y) n;sum(xy) sum(yy)...
sum(y);sum(xx) sum(xy) sum(x)];
B=[-sum(xx+yy) ; -sum(xx.*y+yy.*y) ; -sum(xx.*x+xy.*y)];
a=A\B;            %x = A\B 用来求解线性方程 A*x = B.  A 和 B 的行数一致.
xc = -.5*a(1);
yc = -.5*a(2);
R = sqrt((a(1)^2+a(2)^2)/4-a(3));
 theta=0:0.1:2*pi;  
    Circle1=xc+R*cos(theta);  
    Circle2=yc+R*sin(theta);  
    figure;
    plot(Circle1,Circle2,'g','linewidth',1); 
    hold on;
    scatter(x,y);
    axis equal  
end 




function [min_dis]= calDir(points,pt,dir)
   ndotls=zeros(size(points,1));
   for i=1:size(points,1)
     vs=pt-points(i,:);vs=vs./norm(vs);
     ndotl=dot(vs,dir);
     ndotls(i)=ndotl;
   end
    [sortr,~]=sort(ndotls);
   num=size(sortr,1);
   if(num>5)
   min_dis=mean(sortr(1:5));
   elseif(num>4)
   min_dis=mean(sortr(1:4));  
   elseif(num>3)
   min_dis=mean(sortr(1:3));  
    elseif(num>2)
   min_dis=mean(sortr(1:2));  
   else
   min_dis=mean(sortr(1));
   end

end

function [Angles]=calAngle(points,normals,pt,pn,SampleNum)
   vs=points-pt;
   rad=sum(vs.*vs,2);
   [sortr,sortIndices]=sort(rad);
   num=size(sortr,1);
   sampleNum=SampleNum;
   if(num<SampleNum)sampleNum=num;end
   min_dis=mean(sortr(1:sampleNum));
   ptns=normals(sortIndices,:);
   pts=points(sortIndices,:);
   pts=pts(1:sampleNum,:);
   ptns=ptns(1:sampleNum,:);
   Angles=0;
   for i=1:size(pts,1)
     n1=ptns(i,:);
     n2=pn;
     p1=pts(i,:);
     p2=pt;
     ang=calc_angle(p1,p2,n1,n2);
     Angles=Angles+ang;
   end
   Angles=min_dis*Angles/sampleNum;
   %Angles=min_dis*;
end

function [min_dis]= calProjectRadius(points,pt)
  

end

