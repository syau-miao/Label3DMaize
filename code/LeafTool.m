function varargout = LeafTool(varargin)
% LEAFTOOL MATLAB code for LeafTool.fig
%      LEAFTOOL, by itself, creates a new LEAFTOOL or raises the existing
%      singleton*.
%
%      H = LEAFTOOL returns the handle to a new LEAFTOOL or the handle to
%      the existing singleton*.
%
%      LEAFTOOL('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in LEAFTOOL.M with the given input arguments.
%
%      LEAFTOOL('Property','Value',...) creates a new LEAFTOOL or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before LeafTool_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to LeafTool_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help LeafTool

% Last Modified by GUIDE v2.5 21-Sep-2020 16:09:49

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @LeafTool_OpeningFcn, ...
                   'gui_OutputFcn',  @LeafTool_OutputFcn, ...
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


% --- Executes just before LeafTool is made visible.
function LeafTool_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to LeafTool (see VARARGIN)

% Choose default command line output for LeafTool
handles.output = hObject;
handles.Ids.TipIds=[];
handles.Ids.StopIds=[];
handles.Ids.TempOverLapIds=[];
handles.Ids.OverLapPtIds=[];
handles.SelectOrganId=[];
handles.Ids.LeafIds=[];
handles.Ids.FinalLeafIds=[];
handles.Features=[];
handles.Ids.FinalStemIds=varargin{2};
handles.Ids.StemIds=varargin{2};
handles.Ids.SpotIds=varargin{3};
handles.Ids.UnSegIds=varargin{4};
handles.StemRadius=varargin{1};
handles.StopRadius=3.0*varargin{1};
handles.MaxProjd=varargin{1};
handles.MaxCos=0.95;
handles.Select=0;
handles.bSegment=0;
set(handles.View,'value',1);
set(handles.LeafTipNum,'string',10); 
set(handles.edit2,'string',1); 
%set(handles.edit4,'string',varargin{1}*5); 
%set(handles.edit3,'string',5*varargin{1}); 
set(handles.SelectLeafTip,'value',0);
set(handles.DeleteLeafTip,'value',0);
%set(handles.SelectOrganState,'value',0);
set(handles.FeatureRadius,'Min',0);
set(handles.FeatureRadius,'Max',varargin{1}*10);
set(handles.FeatureRadius,'value',varargin{1}*5);
global points;
global EMD;
EMD=computeEMD(points);
stemPts=points(handles.Ids.StemIds,:);
datacursormode  off;
% Update handles structure
guidata(hObject, handles);
showResults(handles);
I = imread('icon.png');
javaImage = im2java(I);%
newIcon = javax.swing.ImageIcon(javaImage);
figFrame = get(handles.figure1,'JavaFrame'); %取得Figure的JavaFrame。
figFrame.setFigureIcon(newIcon); %修改图标
% UIWAIT makes LeafTool wait for user response (see UIRESUME)
 uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = LeafTool_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.bSegment;


% --- Executes on button press in EstimateLeafTip.
function EstimateLeafTip_Callback(hObject, eventdata, handles)
% hObject    handle to EstimateLeafTip (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 global points;
 StemIds=handles.Ids.StemIds;
 SpotIds=handles.Ids.SpotIds;
 %searchR=handles.StemRadius*5;
 sample=ones(size(points,1),1);
 sample(StemIds)=0;
 sample(SpotIds)=0;
 searchR=get(handles.FeatureRadius,'value');
 tipNum= str2num(get(handles.LeafTipNum,'string'));
 handles.Ids.TipIds=findTip(points,searchR,tipNum,sample);
 guidata(hObject, handles);
 UpdateResults(handles);
% --- Executes on button press in SelectLeafTip.
function SelectLeafTip_Callback(hObject, eventdata, handles)
% hObject    handle to SelectLeafTip (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of SelectLeafTip
set(handles.View,'value',0);
set(handles.SelectLeafTip,'value',1);
set(handles.DeleteLeafTip,'value',0);
%set(handles.SelectOrganState,'value',0);
handles.Select=1;
guidata(hObject, handles);
datacursormode  on;
dcm_obj = datacursormode(handles.figure1);
dcm_obj.DisplayStyle='window';
% --- Executes on button press in SelectOK.
function SelectOK_Callback(hObject, eventdata, handles)
% hObject    handle to SelectOK (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%handles.Ids.TipIds=[];
if(handles.Select==1)
   dcm_obj = datacursormode(handles.figure1);
   c_info = getCursorInfo(dcm_obj);
   global points;
   if(~isempty(c_info))
       pos=c_info.Position;
       id=findPointId(points,pos);
       handles.Ids.TipIds=[handles.Ids.TipIds;id];
       set(handles.LeafTipNum,'string',length(handles.Ids.TipIds)); 
       guidata(hObject, handles);
       UpdateResults(handles);
   end  
end


% --- Executes on button press in DeleteLeafTip.
function DeleteLeafTip_Callback(hObject, eventdata, handles)
% hObject    handle to DeleteLeafTip (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of DeleteLeafTip
set(handles.View,'value',0);
set(handles.SelectLeafTip,'value',0);
set(handles.DeleteLeafTip,'value',1);
handles.Select=2;
guidata(hObject, handles);
datacursormode  on;
dcm_obj = datacursormode(handles.figure1);
dcm_obj.DisplayStyle='window';
% --- Executes on button press in DeleteOK.
function DeleteOK_Callback(hObject, eventdata, handles)
% hObject    handle to DeleteOK (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%handles.Ids.TipIds=[];
if(handles.Select==2)
   dcm_obj = datacursormode(handles.figure1);
   c_info = getCursorInfo(dcm_obj);
   global points;
   if(~isempty(c_info))
       pos=c_info.Position;
       TipPoints=points(handles.Ids.TipIds,:);
       [~,I]=pdist2(TipPoints,pos,'euclidean','Smallest',1);
       I=unique(I);
       handles.Ids.TipIds(I)=[];
       set(handles.LeafTipNum,'string',length(handles.Ids.TipIds)); 
       guidata(hObject, handles);
       UpdateResults(handles);
   end  
end


function LeafTipNum_Callback(hObject, eventdata, handles)
% hObject    handle to LeafTipNum (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of LeafTipNum as text
%        str2double(get(hObject,'String')) returns contents of LeafTipNum as a double


% --- Executes during object creation, after setting all properties.
function LeafTipNum_CreateFcn(hObject, eventdata, handles)
% hObject    handle to LeafTipNum (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function StopAreaSlider_Callback(hObject, eventdata, handles)
% hObject    handle to StopAreaSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
scale=get(handles.StopAreaSlider,'value');
global points;
stemPts=points(handles.Ids.StemIds,:);
handles.Ids.StopIds=findStopAreaId(points,handles.Ids.UnSegIds,stemPts,handles.StemRadius*scale);
guidata(hObject, handles);
UpdateResults(handles);
% --- Executes during object creation, after setting all properties.
function StopAreaSlider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to StopAreaSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in LeafCoaseSegmentation.
function LeafCoaseSegmentation_Callback(hObject, ~, handles)
% hObject    handle to LeafCoaseSegmentation (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global points;
global normals;
% handles.Ids.TipIds=[];
% handles.Ids.StopIds=[];
% handles.Ids.LeafIds=[];
% handles.Ids.StemIds=varargin{2};
% handles.Ids.UnSegIds=varargin{3};
global unSegId;
unSegIds=unSegId;
TipIds=handles.Ids.TipIds;
StopAreas=handles.Ids.StopIds;
StemIds=handles.Ids.StemIds;
Projd=get(handles.ProjdSlider,'value');
MaxCos=get(handles.AngleSlider,'value');
[handles.Ids.LeafIds, handles.Ids.unSegIds]=CoaseSegmentLeaf_GUI(points,normals,unSegIds,StemIds,TipIds,StopAreas,Projd,MaxCos);
handles.Ids.FinalLeafIds=handles.Ids.LeafIds;
handles.Ids.FinalStemIds=handles.Ids.StemIds;
guidata(hObject, handles);
UpdateResults(handles);

function showResults(handles)
global points;
global unSegId;
cla;
Ids=handles.Ids;
axes(handles.axes1);
scatter3(points(Ids.UnSegIds,1),points(Ids.UnSegIds,2),points(Ids.UnSegIds,3),5,[0 0 0], 'filled');
hold on;
scatter3(points(Ids.TipIds,1),points(Ids.TipIds,2),points(Ids.TipIds,3),30,[0.4 0 0], 'filled');
hold on;
scatter3(points(Ids.StopIds,1),points(Ids.StopIds,2),points(Ids.StopIds,3),10,[0 0 0.4], 'filled');
hold on;
scatter3(points(Ids.StemIds,1),points(Ids.StemIds,2),points(Ids.StemIds,3),5,[1 0 0], 'filled');
hold on;
scatter3(points(Ids.SpotIds,1),points(Ids.SpotIds,2),points(Ids.SpotIds,3),5,[0 1 0], 'filled');
hold on;


color=MyGS.MYCOLOR;
for i=1:length(Ids.LeafIds)
   leafid=Ids.LeafIds{i}; 
   scatter3(points(leafid,1),points(leafid,2),points(leafid,3),5,color(i+1,:), 'filled');
   hold on; 
end
hold on;

% axis off; 
axis equal; camorbit(0,0,'camera'); axis vis3d; view(0,0);view3d ZOOM;
x1=xlabel('Height AXIS——Growth Direction');       
x2=ylabel('Diameter AXIS');        
%x3=zlabel('Z');        

function UpdateResults(handles)
global points;
global g_regions
axes(handles.axes1);
 cameraPos=get(handles.axes1,'CameraPosition');
 cameraTarget=get(handles.axes1,'CameraTarget');
 cameraUpVector=get(handles.axes1,'CameraUpVector');
 cla;
 Ids=handles.Ids;
axes(handles.axes1);
% scatter3(points(:,1),points(:,2),points(:,3),5,[1 0 0], 'filled');
% hold on;
scatter3(points(Ids.UnSegIds,1),points(Ids.UnSegIds,2),points(Ids.UnSegIds,3),5,[0 0 0], 'filled');
hold on;
scatter3(points(Ids.TipIds,1),points(Ids.TipIds,2),points(Ids.TipIds,3),50,[0.6 0 0], 'filled');
hold on;
scatter3(points(Ids.TempOverLapIds,1),points(Ids.TempOverLapIds,2),points(Ids.TempOverLapIds,3),10,[0.6 0.6 0.6], 'filled');
hold on;
hold on;scatter3(points(Ids.SpotIds,1),points(Ids.SpotIds,2),points(Ids.SpotIds,3),5,[0 1 0], 'filled');
hold on;
scatter3(points(Ids.StopIds,1),points(Ids.StopIds,2),points(Ids.StopIds,3),10,[0 0 0.4], 'filled');
hold on;

scatter3(points(Ids.FinalStemIds,1),points(Ids.FinalStemIds,2),points(Ids.FinalStemIds,3),5,[1 0 0], 'filled');
hold on;
color=MyGS.MYCOLOR;
colorNum=length(color)-1;
for i=2:length(g_regions)
   leafid=g_regions{i}; 
   scatter3(points(leafid,1),points(leafid,2),points(leafid,3),5,color(i+1,:), 'filled');
   hold on; 
end
if(handles.bSegment==1)
 for i=1:length(handles.SelectOrganId)
   indices=g_regions{handles.SelectOrganId(i)};
   scatter3(points(indices,1),points(indices,2),points(indices,3),5,color(i,:)./20, 'filled');
   hold on;   
 end
end 
%axis off; 
axis equal; camorbit(0,0,'camera'); axis vis3d; view(0,0);view3d ZOOM;
set(handles.axes1,'CameraPosition',cameraPos);
 set(handles.axes1,'CameraTarget',cameraTarget);
 set(handles.axes1,'CameraUpVector',cameraUpVector);


% --- Executes on slider movement.
function ProjdSlider_Callback(hObject, eventdata, handles)
% hObject    handle to ProjdSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function ProjdSlider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ProjdSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function AngleSlider_Callback(hObject, eventdata, handles)
% hObject    handle to AngleSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function AngleSlider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to AngleSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in View.
function View_Callback(hObject, ~, handles)
% hObject    handle to View (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of View
set(handles.View,'value',1);
set(handles.SelectLeafTip,'value',0);
set(handles.DeleteLeafTip,'value',0);
%set(handles.SelectOrganState,'value',0);
handles.Select=0;
guidata(hObject, handles);
datacursormode  off;


% --- Executes on button press in StemLeafClassification.
% function StemLeafClassification_Callback(hObject, eventdata, handles)
% % hObject    handle to StemLeafClassification (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    structure with handles and user data (see GUIDATA)
% %  handles.Ids.LeafIds, handles.Ids.unSegIds
%  sampleNum1 = str2num(get(handles.edit2,'string'));
%  sampleNum2 = str2num(get(handles.edit3,'string'));
% Regions=cell(size(handles.Ids.LeafIds,2)+1,1);
%  Regions{1}= handles.Ids.StemIds;
%  for i=1:size(handles.Ids.LeafIds,2)
%    Regions{i+1}=handles.Ids.LeafIds{i};   
%  end
%  global points;
%  global normals;
%  NewRegions=[];
%  NewRegions=classifyLeafPoints_mt3(points,normals,Regions,handles.Ids.unSegIds,sampleNum1,sampleNum2,1);
%  handles.Ids.FinalStemIds=NewRegions{1};
%  for i=1:size(handles.Ids.LeafIds,2)
%    handles.Ids.FinalLeafIds{i}=NewRegions{i+1};   
%  end
%  guidata(hObject, handles);
%  UpdateResults(handles);
function StemLeafClassification_Callback(hObject, eventdata, handles)
% hObject    handle to StemLeafClassification (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%  handles.Ids.LeafIds, handles.Ids.unSegIds
if(isempty(handles.Ids.TipIds))
  msgbox('please select the peak points for every organ first', 'warning'); 
  return;  
end
sampleNum1 = str2num(get(handles.edit2,'string'));
sampleNum2 = str2num(get(handles.edit4,'string'));
disw=get(handles.DistanceWeight,'value');
Regions=cell(length(handles.Ids.TipIds)+1,1);
 Regions{1}= handles.Ids.StemIds;
 for i=1:size(handles.Ids.TipIds,1)
   Regions{i+1}=handles.Ids.TipIds(i);   
 end
 global points;
 %global EMD;
 
 NewRegions=[];
% handles.Ids.FinalLeafIds=
  NewRegions=classifyLeafPoints_mt3(points,[],Regions,handles.Ids.UnSegIds,sampleNum1,sampleNum2,disw,1);
 %NewRegions=classifyLeafPoints_EMD(points,EMD,Regions,handles.Ids.UnSegIds,sampleNum1,sampleNum2,disw,1); 
 handles.Ids.FinalStemIds=NewRegions{1};
%  handles.Ids.FinalLeafIds=[];
%  for i=1:length(handles.Ids.TipIds)
%    handles.Ids.FinalLeafIds{i}=NewRegions{i+1};   
%  end
global g_regions;
g_regions=NewRegions;
 handles.bSegment=1;
 guidata(hObject, handles);
 Labeling(handles);
 UpdateResults(handles);
 msgbox('finish'); 
% --- Executes on button press in Done.
function Done_Callback(hObject, eventdata, handles)
% hObject    handle to Done (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Labeling(handles);
uiresume(gcbf);


function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function FeatureRadius_Callback(hObject, eventdata, handles)
% hObject    handle to FeatureRadius (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function FeatureRadius_CreateFcn(hObject, eventdata, handles)
% hObject    handle to FeatureRadius (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function DistanceWeight_Callback(hObject, eventdata, handles)
% hObject    handle to DistanceWeight (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function DistanceWeight_CreateFcn(hObject, eventdata, handles)
% hObject    handle to DistanceWeight (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in SelectOrganState.
function SelectOrganState_Callback(hObject, eventdata, handles)
% hObject    handle to SelectOrganState (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of SelectOrganState
set(handles.View,'value',0);
set(handles.SelectLeafTip,'value',0);
set(handles.DeleteLeafTip,'value',0);
%set(handles.SelectOrganState,'value',1);
handles.Select=3;
guidata(hObject, handles);
datacursormode  on;
dcm_obj = datacursormode(handles.figure1);
dcm_obj.DisplayStyle='window';

% --- Executes on button press in SelectOrgan.
function SelectOrgan_Callback(hObject, eventdata, handles)
% hObject    handle to SelectOrgan (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 global points;
 global g_pointlabels;
 
if(handles.Select==3&&handles.bSegment==1)
   dcm_obj = datacursormode(handles.figure1);
   c_info = getCursorInfo(dcm_obj);
   if(~isempty(c_info))
       pos=c_info.Position;
       I=findPointId(points,pos);
       I=unique(I);
       organId=g_pointlabels(I);
       handles.SelectOrganId=[handles.SelectOrganId;organId];
       guidata(hObject, handles);
       UpdateResults(handles);
   end  
end

function findOverLap(hObject,handles)
   r=get(handles.OverLapRadius,'value');
   global points;
   Ids=handles.Ids.OverLapPtIds;
   if(isempty(Ids))
       return;
   end
   bpt=points(Ids,:);
   handles.Ids.TempOverLapIds=findNeighborId(points,bpt,r);
   guidata(hObject, handles);
   UpdateResults(handles);


% --- Executes when figure1 is resized.
function figure1_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in ResetSelectOrgan.
function ResetSelectOrgan_Callback(hObject, eventdata, handles)
% hObject    handle to ResetSelectOrgan (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.SelectOrganId=[];
guidata(hObject, handles);
UpdateResults(handles);

% --- Executes on button press in pushbutton14.
function pushbutton14_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 if(length(handles.SelectOrganId)<2)
    msgbox('please select at least two organs', 'warning'); 
    return;
 end
PostProcess(handles.SelectOrganId,handles.StemRadius);
 global g_regions;
 for i=1:length(handles.SelectOrganId)
    index=handles.SelectOrganId(i);
    if(index>1)
      handles.Ids.FinalLeafIds{index-1}=g_regions{index}; 
    else
      handles.Ids.FinalStemIds=g_regions{index};   
    end
 end
% if(handles.bSegment==1)
%  for i=1:length(handles.SelectOrganId)
handles.SelectOrganId=[];
guidata(hObject, handles);
UpdateResults(handles);


function Labeling(handles)
global g_regions;
global points;
global g_pointlabels;
g_pointlabels=zeros(length(points),1);
for i=1:length(g_regions)
   ids=g_regions{i}; 
   g_pointlabels(ids)=i; 
end


% --- Executes on button press in OrganClassificationEMD.
function OrganClassificationEMD_Callback(hObject, eventdata, handles)
% hObject    handle to OrganClassificationEMD (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if(isempty(handles.Ids.TipIds))
  msgbox('please select the peak points for every organ first', 'warning'); 
  return;  
end
sampleNum1 = str2num(get(handles.edit2,'string'));
% sampleNum2 = str2num(get(handles.edit4,'string'));
% Planefactor=get(handles.PlaneFactor,'value');
Regions=cell(length(handles.Ids.TipIds)+1,1);
Regions{1}= handles.Ids.StemIds;
for i=1:size(handles.Ids.TipIds,1)
  Regions{i+1}=handles.Ids.TipIds(i);   
end
if(~isempty(handles.Ids.SpotIds))
  Regions{end+1}=handles.Ids.SpotIds;
end
 global points;
 global EMD;
 myEMD=EMD;
%  for i=1:length(points)
%     emd=myEMD(i,:);
%     [~,indices]=sort(emd,'descend');
%     indices=indices(sampleNum2:end);
%     myEMD(i,indices)=0;
%  end
%  maxEMD=max(max(myEMD));
%  myEMD(myEMD<maxEMD*sampleNum2 )=0;
 NewRegions=[];
% handles.Ids.FinalLeafIds=
  %NewRegions=classifyLeafPoints_mt3(points,[],Regions,handles.Ids.UnSegIds,sampleNum1,sampleNum2,disw,1);
 NewRegions=classifyLeafPoints_EMD(points,myEMD,Regions,handles.Ids.UnSegIds,sampleNum1,1); 
 handles.Ids.FinalStemIds=NewRegions{1};
%  handles.Ids.FinalLeafIds=[];
%  for i=1:length(handles.Ids.TipIds)
%    handles.Ids.FinalLeafIds{i}=NewRegions{i+1};   
%  end
global g_regions;
g_regions=NewRegions;
 handles.bSegment=1;
 guidata(hObject, handles);
 Labeling(handles);
 UpdateResults(handles);
 msgbox('finish'); 



function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double


% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function PlaneFactor_Callback(hObject, eventdata, handles)
% hObject    handle to PlaneFactor (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function PlaneFactor_CreateFcn(hObject, eventdata, handles)
% hObject    handle to PlaneFactor (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in SegmentSpot.
function SegmentSpot_Callback(hObject, eventdata, handles)
% hObject    handle to SegmentSpot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton17.
function pushbutton17_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if(isempty(handles.Ids.TipIds))
  msgbox('please select the peak points for every organ first', 'warning'); 
  return;  
end
sampleNum1 = str2num(get(handles.edit2,'string'));
% sampleNum2 = str2num(get(handles.edit4,'string'));
% Planefactor=get(handles.PlaneFactor,'value');
Regions=cell(length(handles.Ids.TipIds)+1,1);
Regions{1}= handles.Ids.StemIds;
for i=1:size(handles.Ids.TipIds,1)
  Regions{i+1}=handles.Ids.TipIds(i);   
end
if(~isempty(handles.Ids.SpotIds))
  Regions{end+1}=handles.Ids.SpotIds;
end
 global points;
 %global EMD;
 global normals;
 NewRegions=[];
% handles.Ids.FinalLeafIds=
  NewRegions=classifyLeafPoints_mt3(points,normals,Regions,handles.Ids.UnSegIds,sampleNum1,5,0,1);
 %NewRegions=classifyLeafPoints_EMD(points,EMD,Regions,handles.Ids.UnSegIds,sampleNum1,sampleNum2,disw,1); 
 handles.Ids.FinalStemIds=NewRegions{1};
%  handles.Ids.FinalLeafIds=[];
%  for i=1:length(handles.Ids.TipIds)
%    handles.Ids.FinalLeafIds{i}=NewRegions{i+1};   
%  end
global g_regions;
g_regions=NewRegions;
 handles.bSegment=1;
 guidata(hObject, handles);
 Labeling(handles);
 UpdateResults(handles);
 msgbox('finish'); 
