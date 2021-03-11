function [area]= Point2Mesh(points,smooth,edgelen)
%POINT2MESH �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
%%%%%%%%%���ǽ�����CGAL���ɵ�exe�ļ�����ģ������
%%%%%%%%%�ȱ����һ����ʱ��.ply�����ļ���Ȼ������
%%%%%%%%%%��.off��ģ���ļ�
% if(nargin<3)
%     edgelen=10;
% end
% if(nargin<2)
%    smooth=5; edgelen=10;
% end
filter = {'*.off'};
[fileName, Path] = uiputfile(filter);
MeshName=[Path fileName];
PtName=[Path 'temp.ply'];
ptCloud=pointCloud(points);
pcwrite(ptCloud,PtName);    
arg=['P2M\P2M.exe' 32 PtName 32 MeshName 32 num2str(smooth) 32 num2str(edgelen)];
system(arg);
[vertex,face]=read_off(MeshName);
vertex=vertex';
face=face';
area=0;
for i=1:size(face,2)
   id1=face(1,i);
   id2=face(2,i);
   id3=face(3,i);
   v0=vertex(:,id1);
   v1=vertex(:,id2);
   v2=vertex(:,id3);
   len1=norm(v1-v0);   
   len2=norm(v2-v0);
   len3=norm(v2-v1);
   q=(len1+len2+len3)/2;
   area=area+sqrt(q*(q-len1)*(q-len2)*(q-len3));
end

DrawMesh(vertex',face');
end

