function [xyz0 ,direction ] = fitline( lineData)
%FITLINE �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
xyz0=mean(lineData,1);
% xyz0=(lineData(1,:)),
% Э�����������任�������ƽ�治ͬ����
% ����ֱ�ߵķ���ʵ�������������ֵ��Ӧ������������ͬ
centeredLine=bsxfun(@minus,lineData,xyz0);
[U,S,V]=svd(centeredLine);
direction=V(:,1)';
%h=plot3(NewLineData(:,1),NewLineData(:,2),NewLineData(:,3),'r');
%h.LineWidth = 2;

%hold on;


end

