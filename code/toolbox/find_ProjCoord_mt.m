function [ PCoord ] = find_ProjCoord_mt(dir1,center,v,varargin)
%FIND_PROJCOORD_MT 计算点V到面P的投影点，P的法向量为dir1，P上一点为center
%%  垂直于dir1的面K公式为   (x-center)*dir1=dir1*x-dir*center
%%  点P按dir1方向到面K的投影为  p'= p+dir1*t;
%%  t的值为
% TEMP=(dir1.*center-dir1.*v);
% Temp2=(dir1(1)*dir1(1)+dir1(2)*dir1(2)+dir1(3)*dir1(3));
% t= (dir1(1)*center(1)+dir1(2)*center(2)+dir1(3)*center(3)-dir1(1)*v(:,1)-dir1(2)*v(:,2)-dir1(3)*v(:,3))./(dir1(1)*dir1(1)+dir1(2)*dir1(2)+dir1(3)*dir1(3));
% =v+dir1.*t;
A=dir1(1);B=dir1(2);C=dir1(3);D=-1*(dir1(1)*center(1)+dir1(2)*center(2)+dir1(3)*center(3));
PCoord(:,1)=(B*B+C*C).*v(:,1)-A*(B*v(:,2)+C.*v(:,3)+D);
PCoord(:,2)=(A*A+C*C).*v(:,2)-B*(A*v(:,1)+C.*v(:,3)+D);
PCoord(:,3)=(A*A+B*B).*v(:,3)-C*(A*v(:,1)+B.*v(:,2)+D);
if(numel(varargin)>0)
figure
scatter3(v(:,1),v(:,2),v(:,3),'r');
hold on;
scatter3(center(1),center(2),center(3),'b');
hold on;
scatter3(PCoord(:,1),PCoord(:,2),PCoord(:,3),'g');
hold on;
X=PCoord(:,1);
Y=PCoord(:,2);
xfit = min(X):0.001:max(X);
yfit = min(Y):0.001:max(Y);

[XFIT,YFIT]= meshgrid (xfit,yfit);
d=dir1(1)*center(1)+dir1(2)*center(2)+dir1(3)*center(3);
ZFIT = -(-1*d + dir1(1) * XFIT + dir1(2) * YFIT)/dir1(3);

mesh(XFIT,YFIT,ZFIT);

end
end

