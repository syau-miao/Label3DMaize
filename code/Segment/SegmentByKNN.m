function [ dstLabels ] = SegmentByKNN( srcPts,dstPts,srcLabels,number )
%SEGMENTBYKNN 此处显示有关此函数的摘要
%   此处显示详细说明
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

