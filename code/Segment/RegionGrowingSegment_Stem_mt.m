function [ StemIndices ] = RegionGrowingSegment_Stem_mt( points,tensors,UnSegmentIndices,SeedIndices,SearchR,ndotl)
%REGIONGROWINGSEGMENT_STEM_MT 此处显示有关此函数的摘要
%   此处显示详细说明
      SegmentIndices=[];
     USegIndices=UnSegmentIndices;
     TempPoints=points(SeedIndices,:);
     Y=TempPoints(:,2);
     Z=TempPoints(:,3);
     maxY=max(Y);minY=min(Y);maxZ=max(Z);minZ=min(Z);
     th_Y=1.5*abs(maxY-abs(minY));
     th_Z=1.5*abs(maxZ-abs(minZ));
     Radius=sqrt(Y.*Y+Z.*Z);
     minR=min(Radius);
     SearchR=max(Radius);
%     TempTensors=tensors(SeedIndices,:);
%     zv=repmat([1 0 0],size(TempTensors,1),1);
 %    Dot=abs(TempTensors.*zv);
    % ndotl=1.2*mean(Dot(:,1),1);
     SeedPt=median(points(SeedIndices,:),1);
    % SeedPt(1)=0;SeedSearchRPt(2)=0;
     LastV=[1 0 0];
     newDir=[1,0,0];
     LastSeedIndices=SeedIndices;
    % SeedPt=SeedPt+SearchR/2*newDir;
     while(true)
        UnSegmentPts=points(USegIndices,:);
        ptCloud = pointCloud(UnSegmentPts);  
        [indices,~]= findNeighborsInRadius(ptCloud,SeedPt,SearchR);
        if(length(indices)==0)
            break;
        end
        
        Varray=[];
        Iarray=[];
       % Iarray2=[];
       pts=UnSegmentPts(indices,:);
%        minY=min(pts(:,2));
%        maxY=max(pts(:,2));
%        minZ=min(pts(:,3));
%        maxZ=max(pts(:,3));
%        yratio=0.0;zratio=0.0;
%        if(abs(maxY)>abs(minY))
%        yratio=abs(minY)/abs(maxY);
%        else
%        yratio=abs(maxY)/abs(minY);
%        end;
%        if(abs(maxZ)>abs(minZ))
%        zratio=abs(minZ)/abs(maxZ);
%        else
%        zratio=abs(maxZ)/abs(minZ);
%        end;
%        sym=0.3;
%        if(yratio<sym||zratio<sym)
%         %   break;
%        end;
           
        for i=1:length(indices)
          pt=UnSegmentPts(indices(i),:);
%          tensor=tensors(indices(i),:);
          dir=(pt-SeedPt)./norm(pt-SeedPt);
        %  dis=sqrt(pt(2)*pt(2)+pt(3)*pt(3));
%           if(pt(2)>maxY||pt(2)<minY||pt(3)>maxZ||pt(3)<minZ)
%             %  continue;
%           end;
          if(dot(dir,[1 0 0])>0.5)
       %     if(abs(dot(tensor,[1 0 0]))<ndotl)  
%               if(dis>minR)
              Varray=[Varray;dir];
%              Iarray2=[Iarray2;indices(i)];
%               end
             
       %     end
          end;
          Iarray=[Iarray;indices(i)];
        end
        neindices=USegIndices(Iarray);
        
         if(length(Varray)==0)
            break;
         end
      
      
%         if(false)
%               local_pts=UnSegmentPts(Iarray2,:);
%         close all;
%         figure('Name','2d pic','NumberTitle','off');
%         scatter(local_pts(:,2), local_pts(:,3),5,[0 0 0], 'filled');     
%        end
         
        
        newDir=median(Varray,1);
        newDir=newDir./norm(newDir);
        newDir=(newDir+LastV)/2;
        newDir=newDir./norm(newDir);
        angle=acos(abs(dot(newDir,LastV)));
        if(angle>ndotl)
           break; 
        end;
        LastV=newDir;
%         angle=acos(abs(dot(newDir,[1 0 0])));
%         if(angle>ndotl)
%            break; 
%         end;
        
        
        if(false)
        close all;
        figure('Name','regiongrowthsegmentStem','NumberTitle','off');set(gcf,'color','white');movegui('southwest');
        scatter3(points(:,1),points(:,2),points(:,3),5,[0 0 0], 'filled');
        hold on
        scatter3(SeedPt(:,1),SeedPt(:,2),SeedPt(:,3),20,[0 1 0], 'filled');
        hold on
        scatter3(points(neindices,1),points(neindices,2),points(neindices,3),10,[1 1 0], 'filled');
        hold on
        axis off; axis equal; camorbit(0,0,'camera'); axis vis3d; view(-90,0);view3d rot;            
       
        figure('Name','2d pic','NumberTitle','off');
         scatter(points(neindices,2), points(neindices,3),5,[0 0 0], 'filled');  
        end
      
        SeedPt=SeedPt+SearchR*newDir;
        SegmentIndices=[SegmentIndices;USegIndices(Iarray)];
        USegIndices=setdiff(USegIndices,USegIndices(Iarray)); 
     end
     
    % StemIndices=UnSegmentIndices(SegmentIndices);
     StemIndices=[SegmentIndices;SeedIndices];
     
     
     
     if(true)
        figure('Name','regiongrowthsegment','NumberTitle','off');set(gcf,'color','white');movegui('southwest');

     scatter3(points(:,1),points(:,2),points(:,3),5,[0 0 0], 'filled');
    hold on
    scatter3(points(StemIndices,1),points(StemIndices,2),points(StemIndices,3),5,[0 0 1], 'filled');
    hold on
  %  showLine(mean(StemPoints,1),StemDir,'red');
   axis off; axis equal; camorbit(0,0,'camera'); axis vis3d; view(-90,0);view3d rot;  
         
         
     end

end

