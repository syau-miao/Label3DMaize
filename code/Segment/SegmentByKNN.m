function [ dstLabels ] = SegmentByKNN( srcPts,dstPts,srcLabels,number )
%SEGMENTBYKNN �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
  kdtreeobj= KDTreeSearcher(srcPts,'distance','euclidean');
   %[n,~ ] = rangesearch(kdtreeobj,Pts,20);
  indices = knnsearch(kdtreeobj,dstPts,'k',number);
  dstLabels=zeros(length(dstPts),1);
 for i=1:size(dstPts,1)
   ids=indices(i,:);
   labels=srcLabels(ids);
  %labels=unique(labels);
   table = tabulate(labels);
   [maxCount,idx] = max(table(:,2));
   dstLabels(i)=table(idx);
 end
end

