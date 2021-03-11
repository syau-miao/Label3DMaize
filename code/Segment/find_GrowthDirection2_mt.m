function [ GrowthDir, opp ] = find_GrowthDirection2_mt(  SegmentPoints,SegmentType,Dir,center )
%FIND_GROWTHDIRECTION2_MT find_GrowthDirection函数判断不出dir的时候，用
%  该函数进行处理，该函数不会单独调用，由find_GrowthDirection函数调用
  Num=size(SegmentPoints,2);
  %将所有joint_joint合在一起，放入jjpoints中
  jjpoints=zeros(0,3);
  for i=1:Num
   % if(SegmentType(i)==myGlobal.ROOT_JOINT)
   %    continue;
   % end
    Seg=SegmentPoints(i);
    segPoint=LinkSegmentPoints_mt(Seg,2);
    jjpoints=[jjpoints;segPoint];
  end 
  %删除掉
  ptCloudIn = pointCloud(jjpoints);
  ptCloudOut = pcdenoise(ptCloudIn,'NumNeighbors',32,'Threshold',0.8);
  jjpoints_=ptCloudOut.Location;
  point_num=size(jjpoints_,1);
  Radius=zeros(point_num,1);
  T=zeros(point_num,1);
  for i=1:point_num
   [Radius(i) T(i)]= P2LineDistance(center,Dir,jjpoints_(i,:));   
  end
  [T_ I_]=sort(T);
  Radius_=Radius(I_);
  middleT=(T_(end)+T_(1))/2;
  %StepT=(T_(end)-T_(1))/2;
  %[T_1]=find(T_>T_(end)-StepT);
  %[T_2]=find(T_<T_(1)+StepT);
  [T_1]=find(T_>middleT);
  [T_2]=find(T_<middleT);
  Radius_1=Radius_(T_1);
  Radius_2=Radius_(T_2);
  max_r_1=max((Radius_1));
  max_r_2=max((Radius_2));
  if(max_r_1>max_r_2)
      GrowthDir=Dir;
      opp=true;
  else
      GrowthDir=-Dir;
      opp=false;
  end
end

