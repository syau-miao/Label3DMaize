% v = regexp(version(),'\.','split');
% mj = str2double(v{1});
% mn = str2double(v{2});
% % v = version();
% % di = find(v=='.');
% % mj= str2double(v(1:di(1)-1));
% % mn= str2double(v(di(1)+1 :di(2)-1));

if strfind(computer(),'64')
    defs = '-DA64BITS '; % for 64bit machines - define pointer type
else
    defs = '';
end
% if mj < 7 || (mj==7 && mn < 3)
if verLessThan('matlab','7.3')    
    defs = [defs, '-DmwIndex=int -DmwSize=size_t '];
end

cmd = sprintf('mex -O -largeArrayDims %s GraphCutMex.cpp graph.cpp GCoptimization.cpp GraphCut.cpp LinkedBlockList.cpp maxflow.cpp', defs);
eval(cmd);
cmd = sprintf('mex -O -largeArrayDims %s GraphCut3dConstr.cpp graph.cpp GCoptimization.cpp GraphCut.cpp LinkedBlockList.cpp maxflow.cpp', defs);
eval(cmd);
cmd = sprintf('mex -O -largeArrayDims %s GraphCutConstrSparse.cpp graph.cpp GCoptimization.cpp GraphCut.cpp LinkedBlockList.cpp maxflow.cpp', defs);
eval(cmd);
cmd = sprintf('mex -O -largeArrayDims %s GraphCutConstr.cpp graph.cpp GCoptimization.cpp GraphCut.cpp LinkedBlockList.cpp maxflow.cpp', defs);
eval(cmd);
clear cmd mj mn v di defs


% if strcmp(computer(),'GLNXA64')
%     mex -g  -DA64BITS GraphCutMex.cpp graph.cpp GCoptimization.cpp GraphCut.cpp LinkedBlockList.cpp maxflow.cpp
%     mex -g  -DA64BITS GraphCut3dConstr.cpp graph.cpp GCoptimization.cpp GraphCut.cpp LinkedBlockList.cpp maxflow.cpp
%     if v >= 7.3
%         mex -g  -largeArrayDims -DMAT73 -DA64BITS GraphCutConstrSparse.cpp graph.cpp GCoptimization.cpp GraphCut.cpp LinkedBlockList.cpp maxflow.cpp
%     else
%         mex -g  -DA64BITS GraphCutConstrSparse.cpp graph.cpp GCoptimization.cpp GraphCut.cpp LinkedBlockList.cpp maxflow.cpp
%     end
%     mex -g -DA64BITS GraphCutConstr.cpp graph.cpp GCoptimization.cpp GraphCut.cpp LinkedBlockList.cpp maxflow.cpp
% else
%     mex -g GraphCutMex.cpp graph.cpp GCoptimization.cpp GraphCut.cpp LinkedBlockList.cpp maxflow.cpp
%     mex -g  GraphCut3dConstr.cpp graph.cpp GCoptimization.cpp GraphCut.cpp LinkedBlockList.cpp maxflow.cpp
%     if v >= 7.3
%         mex -g  -largeArrayDims -DMAT73 GraphCutConstrSparse.cpp graph.cpp GCoptimization.cpp GraphCut.cpp LinkedBlockList.cpp maxflow.cpp
%     else
%         mex -g  GraphCutConstrSparse.cpp graph.cpp GCoptimization.cpp GraphCut.cpp LinkedBlockList.cpp maxflow.cpp
%     end
%     mex -g  GraphCutConstr.cpp graph.cpp GCoptimization.cpp GraphCut.cpp LinkedBlockList.cpp maxflow.cpp
% end    
