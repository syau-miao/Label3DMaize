function [ SegId,unSegId ] = MedianStem( points,SegId,unSegId,IntNum )
%MEDIANSTEM 此处显示有关此函数的摘要
%   此处显示详细说明
    stemPts= points(SegId,:);
    [res,dir_]=fitline(stemPts);
    center=[res(1) res(2) res(3)]; 
    dir=[dir_(1) dir_(2) dir_(3)];
    total=[];
    tvec=[];
    for i=1:size(stemPts,1)
    pt=stemPts(i,:);
    [dis,t]=P2LineDistance(center,dir,pt);
    total=[total;dis];
    tvec=[tvec;t];
    end
    %%%%%沿着茎tvec进行分割%%%%%%%%%%%%%%
    mint=min(tvec); maxt=max(tvec);
    %IntNum=1;
    stept=(maxt-mint)/IntNum;
    %%%%%%%每一段的茎中，去掉大于median R%%%%%%%%%%%%%%
    for i=1:IntNum
       t1=mint+stept*(i-1);t2=t1+stept;
       indices=find(tvec>=t1&tvec<=t2);%% indices 针对茎的索引
       ds=total(indices);%%indices 针对点云的索引
       mds=median(ds);
       ids=indices(find(ds>mds));%%ids针对茎的
       if(isempty(ids))continue;end
       uIds=SegId(ids);%%%uIds针对点云的
       unSegId=[unSegId;uIds];
    end
    C=intersect(unSegId,SegId);
    SegId=setdiff(SegId,C);

end

