function [ SegId,unSegId ] = MedianStem( points,SegId,unSegId,IntNum )
%MEDIANSTEM �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
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
    %%%%%���ž�tvec���зָ�%%%%%%%%%%%%%%
    mint=min(tvec); maxt=max(tvec);
    %IntNum=1;
    stept=(maxt-mint)/IntNum;
    %%%%%%%ÿһ�εľ��У�ȥ������median R%%%%%%%%%%%%%%
    for i=1:IntNum
       t1=mint+stept*(i-1);t2=t1+stept;
       indices=find(tvec>=t1&tvec<=t2);%% indices ��Ծ�������
       ds=total(indices);%%indices ��Ե��Ƶ�����
       mds=median(ds);
       ids=indices(find(ds>mds));%%ids��Ծ���
       if(isempty(ids))continue;end
       uIds=SegId(ids);%%%uIds��Ե��Ƶ�
       unSegId=[unSegId;uIds];
    end
    C=intersect(unSegId,SegId);
    SegId=setdiff(SegId,C);

end

