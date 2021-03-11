function [ tri ] = SomMesh( num1,num2 )
%SOMMESH 此处显示有关此函数的摘要
%   此处显示详细说明
    tri=[];
%     for i=1:num1-1
%        for j=1:num2-1   
%         p1=(j-1)*num1+i;
%         p2=(j-1)*num1+i+1;
%         p3=j*num1+i;
%         p4=j*num1+i+1;
%         t=[p1 p2 p3];
%         tri=[tri;t];
%         t=[p2 p4 p3];
%         tri=[tri;t];
%        end
%     end
 for i=1:num1-1
       for j=1:num2-1   
        p1=(j-1)*num1+i;
        p2=(j-1)*num1+i+1;
        p3=j*num1+i;
        p4=j*num1+i+1;
        t=[p1 p2 p3];
        tri=[tri;t];
        t=[p2 p4 p3];
        tri=[tri;t];
       end
    end
end

