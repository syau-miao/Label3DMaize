function varargout = RefineSegmentation(varargin)
% REFINESEGMENTATION MATLAB code for RefineSegmentation.fig
%      REFINESEGMENTATION, by itself, creates a new REFINESEGMENTATION or raises the existing
%      singleton*.
%
%      H = REFINESEGMENTATION returns the handle to a new REFINESEGMENTATION or the handle to
%      the existing singleton*.
%
%      REFINESEGMENTATION('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in REFINESEGMENTATION.M with the given input arguments.
%
%      REFINESEGMENTATION('Property','Value',...) creates a new REFINESEGMENTATION or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before RefineSegmentation_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to RefineSegmentation_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help RefineSegmentation

% Last Modified by GUIDE v2.5 04-May-2020 13:38:55

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @RefineSegmentation_OpeningFcn, ...
                   'gui_OutputFcn',  @RefineSegmentation_OutputFcn, ...
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


% --- Executes just before RefineSegmentation is made visible.
function RefineSegmentation_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to RefineSegmentation (see VARARGIN)

% Choose default command line output for RefineSegmentation
handles.output = hObject;
handles.SelectOrganId=[];
handles.Select=0;
handles.StemRadius=varargin{1};
set(handles.View3DState,'value',1);
set(handles.SelectOrganState,'value',0);
datacursormode  off;
guidata(hObject, handles);
% Update handles structure
guidata(hObject, handles);
UpdateResults(handles,false);
% UIWAIT makes RefineSegmentation wait for user response (see UIRESUME)
 uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = RefineSegmentation_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in SelectOrganState.
function SelectOrganState_Callback(hObject, eventdata, handles)
% hObject    handle to SelectOrganState (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of SelectOrganState
set(handles.View3DState,'value',0);
set(handles.SelectOrganState,'value',1);
handles.Select=1;
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
if(handles.Select==1)
   dcm_obj = datacursormode(handles.figure1);
   c_info = getCursorInfo(dcm_obj);
   if(~isempty(c_info))
       pos=c_info.Position;
       I=findPointId(points,pos);
       I=unique(I);
       organId=g_pointlabels(I);
       handles.SelectOrganId=[handles.SelectOrganId;organId];
       guidata(hObject, handles);
       UpdateResults(handles,true);
   end  
end

% --- Executes on button press in ResetSelectOrgan.
function ResetSelectOrgan_Callback(hObject, eventdata, handles)
% hObject    handle to ResetSelectOrgan (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.SelectOrganId=[];
guidata(hObject, handles);
UpdateResults(handles,true);

% --- Executes on button press in Segmentation.
function Segmentation_Callback(hObject, eventdata, handles)
% hObject    handle to Segmentation (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if(length(handles.SelectOrganId)<2)
    msgbox('please select at least two organs', 'warning'); 
    return;
 end
PostProcess(handles.SelectOrganId,handles.StemRadius);
%global g_regions;
handles.SelectOrganId=[];

guidata(hObject, handles);
UpdateResults(handles,true);

% --- Executes on button press in View3DState.
function View3DState_Callback(hObject, eventdata, handles)
% hObject    handle to View3DState (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of View3DState
set(handles.View3DState,'value',1);
set(handles.SelectOrganState,'value',0);
handles.Select=0;
guidata(hObject, handles);
datacursormode  off;

% --- Executes on button press in SubmitResult.
function SubmitResult_Callback(hObject, eventdata, handles)
% hObject    handle to SubmitResult (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Labeling(handles);
uiresume(gcbf);

function UpdateResults(handles,update)
global points;
global g_regions
axes(handles.axes1);
if(update)
cameraPos=get(handles.axes1,'CameraPosition');
cameraTarget=get(handles.axes1,'CameraTarget');
cameraUpVector=get(handles.axes1,'CameraUpVector');
end
cla;
axes(handles.axes1);
% scatter3(points(:,1),points(:,2),points(:,3),5,[1 0 0], 'filled');
% hold on;
color=MyGS.MYCOLOR;
for i=1:length(g_regions)
   leafid=g_regions{i}; 
   scatter3(points(leafid,1),points(leafid,2),points(leafid,3),5,color(i,:), 'filled');
   hold on; 
end
for i=1:length(handles.SelectOrganId)
  indices=g_regions{handles.SelectOrganId(i)};
  scatter3(points(indices,1),points(indices,2),points(indices,3),5,color(i,:)./20, 'filled');
  hold on;   
end
axis off; axis equal; camorbit(0,0,'camera'); axis vis3d; view(0,0);view3d ZOOM;
if(update)
set(handles.axes1,'CameraPosition',cameraPos);
 set(handles.axes1,'CameraTarget',cameraTarget);
 set(handles.axes1,'CameraUpVector',cameraUpVector);
end
 
 
function Labeling(handles)
global g_regions;
global points;
global g_pointlabels;
g_pointlabels=zeros(length(points),1);
for i=1:length(g_regions)
   ids=g_regions{i}; 
   g_pointlabels(ids)=i; 
end
