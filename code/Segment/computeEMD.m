function [ EMD ] = computeEMD( points)
%COMPUTEEMD 此处显示有关此函数的摘要
%   此处显示详细说明
spoints=points;
d1=length(points);
d2=d1;
N=1;
iter=0;
nspoints=[];
d1=length(points);
d2=size(spoints,1);
M=pdist2(points,spoints); % normalize to get unit median.
M=M./max(M(:));
lambda=5;
K=exp(-lambda*M);

% in practical situations it might be a good idea to do the following:
K(K<1e-100)=1e-100;
U=K.*M;
a=zeros(d1,1);
a(a==0)=1/d1;
b=zeros(d2,1);
b(b==0)=1/d2;
[D,lowerEMD,l,m]=sinkhornTransport(a,b,K,U,lambda,[],[],[],[],1); % running with VERBOSE
i=1;
EMD=bsxfun(@times,m(:,i)',(bsxfun(@times,l(:,i),K)));
% meanE=mean(EMD);
% EMD(EMD<meanE)=0;
%EMD=1./EMD;
end

