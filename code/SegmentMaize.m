function varargout = SegmentMaize(varargin)
% SEGMENTMAIZE MATLAB code for SegmentMaize.fig
%      SEGMENTMAIZE, by itself, creates a new SEGMENTMAIZE or raises the existing
%      singleton*.
%rrrrrrr the existing singleton*.
%
%      SEGMENTMAIZE('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SEGMENTMAIZE.M with the given input arguments.
%
%      SEGMENTMAIZE('Property','Value',...) creates a new SEGMENTMAIZE or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before SegmentMaize_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to SegmentMaize_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help SegmentMaize

% Last Modified by GUIDE v2.5 11-Jul-2020 18:56:42

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @SegmentMaize_OpeningFcn, ...
                   'gui_OutputFcn',  @SegmentMaize_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before SegmentMaize is made visible.
function SegmentMaize_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to SegmentMaize (see VARARGIN)

% Choose default command line output for SegmentMaize
handles.output = hObject;
handles.process= 0;
set(handles.gridStep,'string',1); 
global g_SelectIndices;
g_SelectIndices=[];
I = imread('icon.png');
javaImage = im2java(I);%①
newIcon = javax.swing.ImageIcon(javaImage);
figFrame = get(handles.figure1,'JavaFrame'); %取得Figure的JavaFrame。
figFrame.setFigureIcon(newIcon); %修改图标
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes SegmentMaize wait for user response (see UIRESUME)
 %uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = SegmentMaize_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in OpenPlyFile.
function OpenPlyFile_Callback(hObject, eventdata, handles)
% hObject    handle to OpenPlyFile (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[filename,pathname] = uigetfile('.ply','Select the ply point cloud file');  
if(filename==0)
   return; 
end

global g_regions;
g_regions=[];
global points;
global original_points;
%points=[];
 pccloud=pcread([pathname filename]);
 points=double(pccloud.Location);
 points=points(:,1:3);
%%[points, ~]=loadSegmentFile2([pathname '\']);
original_points=points;
handles.file_points=points;
global g_regions;
g_regions=[];
if(size(points,1)==0)
   return; 
end
global g_regions;
global unSegId;
g_regions=[];
unSegId=[];
% setappdata(handles.axes1,'points',points);
%setappdata(0,'points',points);
handles.process=1;
density=computeDensity(points);
set(handles.PointNumber,'string',length(points));
set(handles.PointCloudSpace,'string',density);
if(length(points)>10000)
    msgbox('It is strongly recommended that you reduce the number of point clouds to less than 10000, otherwise the segmentation will be very slow. You can use the "downsample" function for point cloud sampling. Our software supports the segmentation of different resolution point clouds by using the segmentation results of specific resolution point clouds.', 'warning'); 
end
guidata(hObject, handles);
ShowResults(handles);

% --- Executes on button press in StemTools.
function StemTools_Callback(hObject, eventdata, handles)
% hObject    handle to StemTools (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%getappdata(handles.axes1,'points');
%points=getappdata(0,'points');
if(handles.process<1)
   msgbox('please open a model first', 'warning'); 
   return;
end
guidata(hObject, handles);
   global points;
   [~,OBB]=computeOBB(points);
   LenY=(OBB(4)-OBB(3))/3;
[stemIds,stemRadius]=StemTool(LenY);
if(isempty(stemIds))
   msgbox('stem segmentation error', 'warning'); 
   return; 
end
global g_regions;
global g_stemradius;
g_stemradius=stemRadius;
g_regions{1}=stemIds;
handles.stemIds=stemIds;
handles.spotIds=[];
handles.process=2;
handles.stemRadius=stemRadius;
guidata(hObject, handles);
UpdateResults(handles);



function ShowResults(handles)
global points;
global g_regions;
axes(handles.axes1);
% cameraPos=get(handles.axes1,'CameraPosition');
% cameraTarget=get(handles.axes1,'CameraTarget');
% cameraUpVector=get(handles.axes1,'CameraUpVector');
cla;
scatter3(points(:,1),points(:,2),points(:,3),5,[0 0 0], 'filled');
hold on;
color=MyGS.MYCOLOR;
if(~isempty(g_regions))
  for i=1:size(g_regions,1)
  indices=g_regions{i};
  scatter3(points(indices,1),points(indices,2),points(indices,3),5,color(i,:), 'filled');
  hold on;
  end
end
axis off; axis equal; camorbit(0,0,'camera'); axis vis3d; view(0,0);view3d ZOOM;
set(handles.axes1,'Color',[1 1 1]);
% set(handles.axes1,'CameraPosition',cameraPos);
% set(handles.axes1,'CameraTarget',cameraTarget);
% set(handles.axes1,'CameraUpVector',cameraUpVector);


function UpdateResults(handles)
global points;
global g_regions;
axes(handles.axes1);
%  cameraPos=get(handles.axes1,'CameraPosition');
%  cameraTarget=get(handles.axes1,'CameraTarget');
%  cameraUpVector=get(handles.axes1,'CameraUpVector');
 cla;
color=MyGS.MYCOLOR;
 
 
scatter3(points(:,1),points(:,2),points(:,3),5,[0 0 0], 'filled');
hold on;
if(~isempty(g_regions))
  for i=1:length(g_regions)
  indices=g_regions{i};
  scatter3(points(indices,1),points(indices,2),points(indices,3),5,color(i,:), 'filled');
  hold on;
  end
end
axis off; axis equal; camorbit(0,0,'camera'); axis vis3d; view(0,0);view3d ZOOM;
set(handles.axes1,'Color',[1 1 1.0]);
%  set(handles.axes1,'CameraPosition',cameraPos);
%  set(handles.axes1,'CameraTarget',cameraTarget);
%  set(handles.axes1,'CameraUpVector',cameraUpVector);


% --- Executes on button press in LeafTool.
function LeafTool_Callback(hObject, eventdata, handles)
% hObject    handle to LeafTool (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if(handles.process<2)
   msgbox('please segment stem first', 'warning'); 
   return;
end
global g_regions;
temp=g_regions{1};
g_regions=[];
g_regions{1}=temp;
global points;
unSegId=(1:size(points,1))';
unSegId=setdiff(unSegId,handles.stemIds);
unSegId=setdiff(unSegId,handles.spotIds);
if(LeafTool(handles.stemRadius,handles.stemIds,handles.spotIds,unSegId)==false)
   return; 
end
%UpdateResults(handles);
handles.process=3;
guidata(hObject, handles);
UpdateResults(handles);


% --- Executes on button press in OrganTool.
function OrganTool_Callback(hObject, eventdata, handles)
% hObject    handle to OrganTool (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if(handles.process<3)
   msgbox('please segment leaf first', 'warning'); 
   return;
end
OrganTool();
%UpdateResults(handles);
handles.process=4;
guidata(hObject, handles);
UpdateResults(handles);

% --- Executes on button press in OpenTxtFile.
function OpenTxtFile_Callback(hObject, eventdata, handles)
% hObject    handle to OpenTxtFile (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[filename,pathname] = uigetfile('.txt','Select the txt point cloud file');  
if(filename==0)
   return; 
end
% fileFolder=fullfile([pathname '\']);
% dirOutput=dir(fullfile(fileFolder,'*.txt'));
% FileNames=dirOutput.name;   % 获取所提取数据文件的个数
% K_Trace =strcat(fileFolder, FileNames);
K_Trace=[pathname  filename]; 
global g_regions;
g_regions=[];
global points;
global points_colors;
global g_pointlabels;
points=[];
points=load(K_Trace);
if(size(points,1)==0)
   return; 
end
handles.file_colors=[];
if(size(points,2)==6)
    showcolorpoints(points(:,1:3),points(:,4:6));
end
points=points(:,1:3);

global original_points;
original_points=points;
handles.file_points=points;

global g_regions;
global unSegId;
g_regions=[];
unSegId=[];
g_regions=[];
density=computeDensity(points);
set(handles.PointNumber,'string',length(points));
set(handles.PointCloudSpace,'string',density);
if(length(points)>10000)
    msgbox('It is strongly recommended that you reduce the number of point clouds to less than 10000, otherwise the segmentation will be very slow. You can use the "downsample" function for point cloud sampling. Our software supports the segmentation of different resolution point clouds by using the segmentation results of specific resolution point clouds.', 'warning'); 
end
% setappdata(handles.axes1,'points',points);
%setappdata(0,'points',points);
handles.process=1;
guidata(hObject, handles);
ShowResults(handles);


% --- Executes on button press in DownSample.
function DownSample_Callback(hObject, eventdata, handles)
% hObject    handle to DownSample (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
gridstep = str2num(get(handles.gridStep,'string'));
global points;
global original_points;
global points_colors;
points_temp=points;
ptCloudIn=pointCloud(points);
%ptCloudOut=pcdownsample(ptCloudIn, 'random', 0.25);
ptCloudOut=pcdownsample(ptCloudIn, 'gridAverage', gridstep);
points=ptCloudOut.Location;
original_points=points;
density=computeDensity(points);
set(handles.PointNumber,'string',length(points));
set(handles.PointCloudSpace,'string',density);
% if(length(points_colors)>0)
%  %  colors_temp=points;
%    kdtreeobj= KDTreeSearcher(points_temp,'distance','euclidean');
%    %[n,~ ] = rangesearch(kdtreeobj,Pts,20);
%   indices = knnsearch(kdtreeobj,points,'k',1);
%   colors_temp=points_colors(indices,:);
%   points_colors=colors_temp;
%   showcolorpoints(points,points_colors);
% end

UpdateResults(handles);

function gridStep_Callback(hObject, eventdata, handles)
% hObject    handle to gridStep (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of gridStep as text
%        str2double(get(hObject,'String')) returns contents of gridStep as a double


% --- Executes during object creation, after setting all properties.
function gridStep_CreateFcn(hObject, eventdata, handles)
% hObject    handle to gridStep (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in SegmentByLabel.
function SegmentByLabel_Callback(hObject, eventdata, handles)
% hObject    handle to SegmentByLabel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if(handles.process<1)
   msgbox('please load point cloud first', 'warning'); 
   return;
end
global points;
global original_points;
points=original_points;
handles.stemRadius=SegmentByLabel();
% if(handles.stemRadius==0)
%     return; 
% end 
handles.process=3;
guidata(hObject, handles);
UpdateResults(handles);


% --- Executes on button press in FineSegmentation.
function FineSegmentation_Callback(hObject, eventdata, handles)
% hObject    handle to FineSegmentation (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if(handles.process<3)
   msgbox('please segment leaf first', 'warning'); 
   return;
end
radius=handles.stemRadius;
% if(handles.stemRadius==0)
%     global points;
%     [~,OBB]=computeOBB(points);
%     LenY=(OBB(4)-OBB(3)); 
% end
RefineSegmentation(radius);
%UpdateResults(handles);
guidata(hObject, handles);
UpdateResults(handles);


% --- Executes on button press in Reset.
function Reset_Callback(hObject, eventdata, handles)
% hObject    handle to Reset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if(handles.process<1)
   msgbox('please load point cloud first', 'warning'); 
   return;
end
global g_regions;
g_regions=[];
global points;
global original_points;
points=handles.file_points;
original_points=handles.file_points;
density=computeDensity(points);
set(handles.PointNumber,'string',length(points));
set(handles.PointCloudSpace,'string',density);
guidata(hObject, handles);
UpdateResults(handles);


% --- Executes on button press in pushbutton11.
function pushbutton11_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
UpdateResults(handles);


% --- Executes on button press in RemoveFlowerPot.
function RemoveFlowerPot_Callback(hObject, eventdata, handles)
% hObject    handle to RemoveFlowerPot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global points_colors;
global points;
if(length(points_colors)==0)
   msgbox('No color information for segmentation', 'warning'); 
   return;
end
RemoveFlowerSpot(points,points_colors);


% --- Executes on button press in pushbutton13.
function pushbutton13_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global points_colors;
global points;
if(length(points_colors)==0)
   msgbox('No color information for removing gray points', 'warning'); 
   return;
end
[points,points_colors]=RemoveGrayPoints(points,points_colors);
showcolorpoints(points,points_colors);


% --- Executes on button press in UnderStem.
function UnderStem_Callback(hObject, eventdata, handles)
% hObject    handle to UnderStem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if(handles.process<2)
   msgbox('please segment stem first', 'warning'); 
   return;
end
global g_regions;
global points;
stemPoints=points(handles.stemIds,:);
xMin=min(stemPoints(:,1));
handles.spotIds=find(points(:,1)<xMin);
g_regions{2}=handles.spotIds;
guidata(hObject, handles);
UpdateResults(handles);


% --- Executes on button press in OpenLabelFile.
function OpenLabelFile_Callback(hObject, eventdata, handles)
% hObject    handle to OpenLabelFile (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[filename,pathname] = uigetfile('.txt','Select the label txt  file');  
if(filename==0)
   return; 
end

K_Trace=[pathname  filename]; 
global g_regions;
g_regions=[];
global points;
global points_colors;
global g_pointlabels;
points=[];
points=load(K_Trace);
if(size(points,2)~=4)
   return; 
end
labels=points(:,4);
points=points(:,1:3);

global original_points;
original_points=points;
handles.file_points=points;

global g_regions;
global unSegId;
unSegId=[];
labelNum=max(labels);
global g_pointlabels;
for i=1:labelNum
   g_regions{i}=find(labels==i);
    g_pointlabels(g_regions{i})=i;
end


density=computeDensity(points);
set(handles.PointNumber,'string',length(points));
set(handles.PointCloudSpace,'string',density);

handles.process=3;
guidata(hObject, handles);
UpdateResults(handles);
