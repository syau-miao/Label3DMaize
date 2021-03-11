function varargout = StemTool(varargin)
% STEMTOOL MATLAB code for StemTool.fig
%      STEMTOOL, by itself, creates a new STEMTOOL or raises the existing
%      singleton*.
%
%      H = STEMTOOL returns the handle to a new STEMTOOL or the handle to
%      the existing singleton*.
%
%      STEMTOOL('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in STEMTOOL.M with the given input arguments.
%
%      STEMTOOL('Property','Value',...) creates a new STEMTOOL or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before StemTool_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to StemTool_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help StemTool

% Last Modified by GUIDE v2.5 03-May-2020 11:00:13

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @StemTool_OpeningFcn, ...
                   'gui_OutputFcn',  @StemTool_OutputFcn, ...
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


% --- Executes just before StemTool is made visible.
function StemTool_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to StemTool (see VARARGIN)

% Choose default command line output for StemTool
handles.output = hObject;
handles.select=0;
handles.Ids.bottomId=[];
handles.Ids.topId=[];
handles.Ids.stemIds=[];
handles.Ids.bottomNeighbors=[];
handles.Ids.topNeighbors=[];
handles.MaxLen=varargin{1};
handles.StemRadius=varargin{1}/5;
set(handles.View,'value',1);
set(handles.SelectBottomStem,'value',0);
set(handles.SelectTopStem,'value',0);
set(handles.edit1,'string',1); 
set(handles.RadiusSlider,'Max',varargin{1});
set(handles.RadiusSlider,'Min',0);
%set(handles.RadiusSlider,'SliderStep',varargin{1}/100);
set(handles.RadiusSlider,'Value',handles.StemRadius);
guidata(hObject, handles);
findStemNeighbors(hObject,handles)
showResults(handles);
datacursormode off;
I = imread('icon.png');
javaImage = im2java(I);%
newIcon = javax.swing.ImageIcon(javaImage);
figFrame = get(handles.figure2,'JavaFrame'); %取得Figure的JavaFrame。
figFrame.setFigureIcon(newIcon); %修改图标
uiwait(handles.figure2);

% --- Outputs from this function are returned to the command line.
function varargout = StemTool_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.Ids.stemIds;
varargout{2} = handles.StemRadius;

% --- Executes on button press in SelectBottomStem.
function SelectBottomStem_Callback(hObject, eventdata, handles)
% hObject    handle to SelectBottomStem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of SelectBottomStem
set(handles.SelectBottomStem,'value',1);
set(handles.SelectTopStem,'value',0);
set(handles.View,'value',0);
%global select;
handles.select=1;
guidata(hObject, handles);
datacursormode on;
dcm_obj = datacursormode(handles.figure2);
dcm_obj.DisplayStyle='window';
%set(gcf,'WindowButtonDownFcn',@figure2_ButtonDownFcn);
%handles.select=1;
% function ButttonDownFcn(src,event)
%  if 
%   dcm_obj = datacursormode(handles.figure2);
%    c_info = getCursorInfo(dcm_obj);
%    global points;
%    if(~isempty(c_info))
%        pos=c_info.Position;
%        handles.Ids.bottomId=findPointId(points,pos);
%        guidata(hObject, handles);
%   end  
%dcm_obj = datacursormode(handles.figure2);
% --- Executes on button press in SelectTopStem.
function SelectTopStem_Callback(hObject, eventdata, handles)
% hObject    handle to SelectTopStem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of SelectTopStem
set(handles.SelectBottomStem,'value',0);
set(handles.SelectTopStem,'value',1);
set(handles.View,'value',0);
%set(handles.select,'value',2);
%global select;
handles.select=2;
guidata(hObject, handles);
datacursormode on;
dcm_obj = datacursormode(handles.figure2);
dcm_obj.DisplayStyle='window';
%dcm_obj = datacursormode(handles.figure2);

% --- Executes on button press in StemSegmentation.
function StemSegmentation_Callback(hObject, eventdata, handles)
% hObject    handle to StemSegmentation (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global points;
global unSegId;
unSegId=(1:size(points,1))';
if(isempty(handles.Ids.bottomId)||isempty(handles.Ids.topId))
   msgbox('please select stem bottom and top points', 'warning'); 
   return;  
end

SeedPt=points(handles.Ids.bottomId,:);
SeedPt=median(SeedPt,1);
StopPt=points(handles.Ids.topId,:);
StopPt=median(StopPt,1);
searchR=handles.StemRadius;
[Id,unSegId]=RegionGrowingSegment_GUI(points,unSegId,SeedPt,searchR,StopPt);
handles.Ids.stemIds=Id;
guidata(hObject, handles);
UpdateResults(handles);
msgbox('finish'); 
% --- Executes on slider movement.

function RadiusSlider_Callback(hObject, eventdata, handles)
% hObject    handle to RadiusSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
%  r=get(handles.RadiusSlider,'value');
%  handles.StemRadius=r;
findStemNeighbors(hObject,handles);
% guidata(hObject, handles);
% UpdateResults(handles);

% --- Executes during object creation, after setting all properties.
function RadiusSlider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to RadiusSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in Done.
function Done_Callback(hObject, eventdata, handles)
% hObject    handle to Done (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if(~isempty(handles.Ids.bottomId)&&~isempty(handles.Ids.topId))
global points;
bottomPt=median(points(handles.Ids.bottomId,:),1);
topPt=median(points(handles.Ids.topId,:),1);
v=topPt-bottomPt;
points=PlantCoordinate(points,handles.Ids.stemIds,v);
global normals;
normals=find_normal_data(points); 
end

uiresume(gcbf);

% --- Executes on button press in View.
function View_Callback(hObject, eventdata, handles)
% hObject    handle to View (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of View
set(handles.View,'value',1);
set(handles.SelectBottomStem,'value',0);
set(handles.SelectTopStem,'value',0);
%set(handles,'select',0);
%global select;
handles.select=0;
guidata(hObject, handles);
UpdateResults(handles);
datacursormode  off;



function showResults(handles)
global points;
cla;
Ids=handles.Ids;
axes(handles.axes1);

%%%%%%画球bottomId%%%%
% rad=handles.StemRadius;
% [x,y,z]=sphere;
% C=repmat([1 0 0],[length(x),1]);
% x0=points(Ids.bottomId,1);
% y0=points(Ids.bottomId,2);
% z0=points(Ids.bottomId,3);
% mesh(rad*x+x0,rad*y+y0,rad*z+z0);
% hold on;
% x0=points(Ids.topId,1);
% y0=points(Ids.topId,2);
% z0=points(Ids.topId,3);
% mesh(rad*x+x0,rad*y+y0,rad*z+z0);
% hold on;
%%%%%%%%%%%%


scatter3(points(:,1),points(:,2),points(:,3),5,[0 0 0], 'filled');
hold on;
scatter3(points(Ids.bottomId,1),points(Ids.bottomId,2),points(Ids.bottomId,3),30,[1 0 0], 'filled');
hold on;
scatter3(points(Ids.topId,1),points(Ids.topId,2),points(Ids.topId,3),30,[0 0 1], 'filled');
hold on;
scatter3(points(Ids.stemIds,1),points(Ids.stemIds,2),points(Ids.stemIds,3),10,[0 1 0], 'filled');
hold on;
scatter3(points(Ids.bottomNeighbors,1),points(Ids.bottomNeighbors,2),points(Ids.bottomNeighbors,3),10,[0.5 0 0], 'filled');
hold on;
scatter3(points(Ids.topNeighbors,1),points(Ids.topNeighbors,2),points(Ids.topNeighbors,3),10,[0 0 0.5], 'filled');
hold on;
scatter3(points(Ids.stemIds,1),points(Ids.stemIds,2),points(Ids.stemIds,3),10,[0 1 0], 'filled');
hold on;
%axis off; 
axis equal; 
camorbit(0,0,'camera'); axis vis3d; view(0,0);view3d ZOOM;


function UpdateResults(handles)
global points;
axes(handles.axes1);
 cameraPos=get(handles.axes1,'CameraPosition');
 cameraTarget=get(handles.axes1,'CameraTarget');
 cameraUpVector=get(handles.axes1,'CameraUpVector');
 cla;
Ids=handles.Ids;
rad=handles.StemRadius;
%%%%%%画球bottomId%%%%
% [x,y,z]=sphere;
% C=repmat([1 0 0],[length(x),1]);
% x0=points(Ids.bottomId,1);
% y0=points(Ids.bottomId,2);
% z0=points(Ids.bottomId,3);
% mesh(rad*x+x0,rad*y+y0,rad*z+z0);
% hold on;
% x0=points(Ids.topId,1);
% y0=points(Ids.topId,2);
% z0=points(Ids.topId,3);
% mesh(rad*x+x0,rad*y+y0,rad*z+z0);
hold on;
scatter3(points(:,1),points(:,2),points(:,3),5,[0 0 0], 'filled');
hold on;
scatter3(points(Ids.bottomId,1),points(Ids.bottomId,2),points(Ids.bottomId,3),30,[1 0 0], 'filled');
hold on;
scatter3(points(Ids.topId,1),points(Ids.topId,2),points(Ids.topId,3),30,[0 0 1], 'filled');
hold on;
scatter3(points(Ids.stemIds,1),points(Ids.stemIds,2),points(Ids.stemIds,3),10,[0 1 0], 'filled');
hold on;
scatter3(points(Ids.bottomNeighbors,1),points(Ids.bottomNeighbors,2),points(Ids.bottomNeighbors,3),10,[0.5 0 0], 'filled');
hold on;
scatter3(points(Ids.topNeighbors,1),points(Ids.topNeighbors,2),points(Ids.topNeighbors,3),10,[0 0 0.5], 'filled');
hold on;

scatter3(points(Ids.stemIds,1),points(Ids.stemIds,2),points(Ids.stemIds,3),10,[0 1 0], 'filled');
hold on;
axis off; axis equal; camorbit(0,0,'camera'); axis vis3d; view(0,0);view3d ZOOM;
set(handles.axes1,'CameraPosition',cameraPos);
set(handles.axes1,'CameraTarget',cameraTarget);
set(handles.axes1,'CameraUpVector',cameraUpVector);

% --- Executes on button press in BottomStemYes.
function BottomStemYes_Callback(hObject, eventdata, handles)
% hObject    handle to BottomStemYes (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if(handles.select==1)
   dcm_obj = datacursormode(handles.figure2);
   c_info = getCursorInfo(dcm_obj);
   global points;
   if(~isempty(c_info))
       pos=c_info.Position;
       handles.Ids.bottomId=[handles.Ids.bottomId;findPointId(points,pos)];
       guidata(hObject, handles);
       findStemNeighbors(hObject,handles);
   end  
end

%UpdateResults(handles);

% --- Executes on button press in TopStemYes.
function TopStemYes_Callback(hObject, eventdata, handles)
% hObject    handle to TopStemYes (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if(handles.select==2)
   dcm_obj = datacursormode(handles.figure2);
   c_info = getCursorInfo(dcm_obj);
   global points;
   if(~isempty(c_info))
       pos=c_info.Position;
       handles.Ids.topId=[findPointId(points,pos);handles.Ids.topId];
       guidata(hObject, handles);
       findStemNeighbors(hObject,handles);
   end  
end

%UpdateResults(handles);


% --- Executes on button press in Reset.



% function findStemNeighbors(hObject,handles)
%    r=get(handles.RadiusSlider,'value');
%    handles.StemRadius=r;
%    global points;
%    Ids=handles.Ids;
%    bpt=points(Ids.bottomId,:);
%    handles.Ids.bottomNeighbors=findNeighborId(points,bpt,r);
%    tpt=points(Ids.topId,:);
%    handles.Ids.topNeighbors=findNeighborId(points,tpt,r);
%    guidata(hObject, handles);
%    UpdateResults(handles);
   
 
function findStemNeighbors(hObject,handles)
   r=get(handles.RadiusSlider,'value');
   handles.StemRadius=r;
   global points;
   Ids=handles.Ids;
   bpt=points(Ids.bottomId,:);
   bpt=median(bpt,1);
   if(~isnan(bpt(1)))
   handles.Ids.bottomNeighbors=findNeighborId(points,bpt,r);
   else
   handles.Ids.bottomNeighbors=[];    
   end
   tpt=points(Ids.topId,:);
   tpt=median(tpt,1);
   if(~isnan(tpt(1)))
   handles.Ids.topNeighbors=findNeighborId(points,tpt,r);
   else
   handles.Ids.topNeighbors=[];   
   end
   guidata(hObject, handles);
   UpdateResults(handles);     
  

function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in MedianOperation.
function MedianOperation_Callback(hObject, eventdata, handles)
% hObject    handle to MedianOperation (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if(isempty(handles.Ids.stemIds))
  msgbox('please segment the stem points firstly', 'warning'); 
  return; 
end
Num = str2num(get(handles.edit1,'string'));
global points;
unSegId=(1:size(points,1))';
unSegId=setdiff(unSegId,handles.Ids.stemIds);
[handles.Ids.stemIds,~]=MedianStem(points,handles.Ids.stemIds,unSegId,Num);
guidata(hObject, handles);
UpdateResults(handles);
msgbox('finish'); 

% --- Executes on button press in ClearBottomPts.
function ClearBottomPts_Callback(hObject, eventdata, handles)
% hObject    handle to ClearBottomPts (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if(handles.select==1)
    handles.Ids.bottomId=[];
    guidata(hObject, handles);
    findStemNeighbors(hObject,handles);
end


% --- Executes on button press in ClearTopPoints.
function ClearTopPoints_Callback(hObject, eventdata, handles)
% hObject    handle to ClearTopPoints (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if(handles.select==2)
    handles.Ids.topId=[];
    guidata(hObject, handles);
    findStemNeighbors(hObject,handles);
end


% --- Executes on button press in SaveLabelFile.
function SaveLabelFile_Callback(hObject, eventdata, handles)
% hObject    handle to SaveLabelFile (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if(isempty(handles.Ids.stemIds))
    msgbox('please segment stem points firstyly', 'warning'); 
    return;
end
filter = {'*.txt'};
[fileName, Path] = uiputfile(filter);
fid=fopen([Path fileName],'w');
global points;
global original_points;
stemIds=handles.Ids.stemIds;
labels=zeros(size(points,1),1);
labels(stemIds)=1;
labels(labels==0)=2;
for i=1:size(original_points,1)
       pt=original_points(i,:);
       label=labels(i);
      fprintf(fid,'%f %f %f %d\r\n',pt(1),pt(2),pt(3),label);
 end
fclose(fid);


% --- Executes on button press in SaveStemPoints.
function SaveStemPoints_Callback(hObject, eventdata, handles)
% hObject    handle to SaveStemPoints (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if(isempty(handles.Ids.stemIds))
    msgbox('please segment stem points firstly', 'warning'); 
    return;
end
global points;
global original_points;
organPoints=original_points(handles.Ids.stemIds,:);
defaultName = 'Stem Points.ply';
filter = {'*.ply'};
[fileName, Path,fileindex] = uiputfile(filter,'save point cloud',defaultName);
if(fileindex==0)
    return;
end
fname=[Path fileName];
ptCloud=pointCloud(organPoints);
pcwrite(ptCloud,fname);
