function varargout = SegmentByLabel(varargin)
% SEGMENTBYLABEL MATLAB code for SegmentByLabel.fig
%      SEGMENTBYLABEL, by itself, creates a new SEGMENTBYLABEL or raises the existing
%      singleton*.
%
%      H = SEGMENTBYLABEL returns the handle to a new SEGMENTBYLABEL or the handle to
%      the existing singleton*.
%
%      SEGMENTBYLABEL('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SEGMENTBYLABEL.M with the given input arguments.
%
%      SEGMENTBYLABEL('Property','Value',...) creates a new SEGMENTBYLABEL or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before SegmentByLabel_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to SegmentByLabel_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help SegmentByLabel

% Last Modified by GUIDE v2.5 05-May-2020 11:02:44

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @SegmentByLabel_OpeningFcn, ...
                   'gui_OutputFcn',  @SegmentByLabel_OutputFcn, ...
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


% --- Executes just before SegmentByLabel is made visible.
function SegmentByLabel_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to SegmentByLabel (see VARARGIN)

% Choose default command line output for SegmentByLabel
handles.select=0;
handles.bSgment=false;
handles.Ids.bottomId=[];
set(handles.View3DState,'value',1);
%set(handles.SelectBottomStemState,'value',0);
datacursormode off;
handles.output = hObject;
handles.stemRadius=0;
% Update handles structure
guidata(hObject, handles);
set(handles.KNN_Number,'string',3); 
ShowResult1(handles,false);
% UIWAIT makes SegmentByLabel wait for user response (see UIRESUME)
uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = SegmentByLabel_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Get default command line output from handles structure
varargout{1} = handles.stemRadius;


% --- Executes on button press in LoadLabelingFile.
function LoadLabelingFile_Callback(hObject, eventdata, handles)
% hObject    handle to LoadLabelingFile (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
  filter = {'*.txt'};
  [fileName, Path,filterindex] = uigetfile(filter,'label file');
  if(filterindex==0)return;end
   txtName=[Path fileName];
   P=load(txtName);
   points=P(:,1:3);
   pointlabels=P(:,4);
   handles.points=points;
   handles.pointlabels=pointlabels;
   labelNum=max(pointlabels);
   regions=cell(1,labelNum);
   for i=1:labelNum
      indices=find(pointlabels==i);
      regions{i}=indices;
   end
   handles.regions=regions;
   guidata(hObject, handles);
   ShowResult2(handles);
  
function KNN_Number_Callback(hObject, eventdata, handles)
% hObject    handle to KNN_Number (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of KNN_Number as text
%        str2double(get(hObject,'String')) returns contents of KNN_Number as a double


% --- Executes during object creation, after setting all properties.
function KNN_Number_CreateFcn(hObject, eventdata, handles)
% hObject    handle to KNN_Number (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Segment.
function Segment_Callback(hObject, eventdata, handles)
% hObject    handle to Segment (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
knn = str2num(get(handles.KNN_Number,'string'));
srcPts=handles.points;
global points;
global g_regions;
dstPts=points;
srcLabels=handles.pointlabels;
global g_pointlabels;
g_pointlabels = SegmentByKNN(srcPts,dstPts,srcLabels,knn);
labelNum=max(g_pointlabels);
   g_regions=cell(labelNum,1);
   for i=1:labelNum
      indices=find(g_pointlabels==i);
      g_regions{i}=indices;
   end
   handles.bSgment=true;
    guidata(hObject, handles);
   ShowResult1(handles,true);
 msgbox('finish');  
%    stemIds=g_regions{1};
%    [globalDirs ,~,~]= pca(points(stemIds,:),'Algorithm','eig');  
%    gDir1=globalDirs(:,1)';
%    points=PlantCoordinate(points,stemIds,v);
%    guidata(hObject, handles);
%    ShowResult1(handles,true);
   
function ShowResult1(handles,update)
global points;
global g_regions;
axes(handles.axes1);
if(update)
 cameraPos=get(handles.axes1,'CameraPosition');
 cameraTarget=get(handles.axes1,'CameraTarget');
 cameraUpVector=get(handles.axes1,'CameraUpVector');
end
cla;
scatter3(points(:,1),points(:,2),points(:,3),5,[0 0 0], 'filled');
hold on;
color=MyGS.MYCOLOR;
if(~isempty(g_regions))
  for i=1:length(g_regions)
  indices=g_regions{i};
  scatter3(points(indices,1),points(indices,2),points(indices,3),5,color(i,:), 'filled');
  hold on;
  end
end
% scatter3(points(handles.Ids.bottomId,1),points(handles.Ids.bottomId,2),points(handles.Ids.bottomId,3),20,[0.7 0.7 0.7], 'filled');
% hold on;
axis off; axis equal; camorbit(0,0,'camera'); axis vis3d; view(0,0);view3d ZOOM;
if(update)
set(handles.axes1,'CameraPosition',cameraPos);
set(handles.axes1,'CameraTarget',cameraTarget);
set(handles.axes1,'CameraUpVector',cameraUpVector);
end

function ShowResult2(handles)
points=handles.points;
regions=handles.regions;
figure('Name','Label file','NumberTitle','off');set(gcf,'color','white');movegui('southwest');
cla;
scatter3(points(:,1),points(:,2),points(:,3),5,[0 0 0], 'filled');
hold on;
color=MyGS.MYCOLOR;
if(~isempty(regions))
  for i=1:size(regions,2)
  indices=regions{i};
  scatter3(points(indices,1),points(indices,2),points(indices,3),5,color(i,:), 'filled');
  hold on;
  end
end
axis off; axis equal; camorbit(0,0,'camera'); axis vis3d; view(0,0);view3d ZOOM;



% --- Executes on button press in SubmitResults.
function SubmitResults_Callback(hObject, eventdata, handles)
% hObject    handle to SubmitResults (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
uiresume(gcbf);


% --- Executes on button press in SelectBottomStemState.
function SelectBottomStemState_Callback(hObject, eventdata, handles)
% hObject    handle to SelectBottomStemState (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of SelectBottomStemState
set(handles.SelectBottomStemState,'value',1);
set(handles.View3DState,'value',0);
%global select;
handles.select=1;
guidata(hObject, handles);
datacursormode on;
dcm_obj = datacursormode(handles.figure1);
dcm_obj.DisplayStyle='window';

% --- Executes on button press in SelectBottomStem.
function SelectBottomStem_Callback(hObject, eventdata, handles)
% hObject    handle to SelectBottomStem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if(handles.select==1)
   dcm_obj = datacursormode(handles.figure1);
   c_info = getCursorInfo(dcm_obj);
   global points;
   if(~isempty(c_info))
       pos=c_info.Position;
       handles.Ids.bottomId=findPointId(points,pos);
       guidata(hObject, handles);
       ShowResult1(handles,true);
   end  
end

% --- Executes on button press in Transformation.
function Transformation_Callback(hObject, eventdata, handles)
% hObject    handle to Transformation (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
   if(handles.bSgment==false)
      msgbox('please segment first', 'warning');  
      return; 
   end
  if(isempty(handles.Ids.bottomId))
      msgbox('please select the bottom stem point first', 'warning');  
      return; 
   end
   global points;
   global g_regions;
   stemIds=g_regions{1};
   [globalDirs ,~,~]= pca(points(stemIds,:),'Algorithm','eig');  
   v=globalDirs(:,1)';
   points=PlantCoordinate(points,stemIds,v);
   bx=points(handles.Ids.bottomId,1);
   tx=max(points(:,1));
   mx=min(points(:,1));
   minY=min(points(stemIds,2));
   maxY=max(points(stemIds,2));
   handles.stemRadius=abs(maxY-minY)/2;
   if(abs(mx-bx)>abs(tx-bx))
     points(:,1)=-1*points(:,1);
   end
   guidata(hObject, handles);
   ShowResult1(handles,true);
   msgbox('finish'); 
% --- Executes on button press in View3DState.
function View3DState_Callback(hObject, eventdata, handles)
% hObject    handle to View3DState (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of View3DState
set(handles.SelectBottomStemState,'value',0);
set(handles.View3DState,'value',1);
%global select;
handles.select=0;
guidata(hObject, handles);
datacursormode off;
dcm_obj = datacursormode(handles.figure1);
dcm_obj.DisplayStyle='window';
