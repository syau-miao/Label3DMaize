function varargout = PostProcess(varargin)
% POSTPROCESS MATLAB code for PostProcess.fig
%      POSTPROCESS, by itself, creates a new POSTPROCESS or raises the existing
%      singleton*.
%
%      H = POSTPROCESS returns the handle to a new POSTPROCESS or the handle to
%      the existing singleton*.
%
%      POSTPROCESS('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in POSTPROCESS.M with the given input arguments.
%
%      POSTPROCESS('Property','Value',...) creates a new POSTPROCESS or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before PostProcess_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to PostProcess_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help PostProcess

% Last Modified by GUIDE v2.5 28-Dec-2020 20:13:57

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @PostProcess_OpeningFcn, ...
                   'gui_OutputFcn',  @PostProcess_OutputFcn, ...
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


% --- Executes just before PostProcess is made visible.
function PostProcess_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to PostProcess (see VARARGIN)

% Choose default command line output for PostProcess
handles.output = hObject;
handles.OrganIds=sort(varargin{1});
handles.select=0;
handles.OrganNum=length(handles.OrganIds);
temp=(1:handles.OrganNum)';
temp=handles.OrganIds;
set(handles.popupmenu1,'string',num2str(temp));
set(handles.popupmenu2,'string',num2str(temp));
set(handles.View3DState,'value',1);
set(handles.SelectPointState,'value',0);
set(handles.SelectPointsState,'value',0);
set(handles.DeletePointState,'value',0);
set(handles.SelectSegmentAreaState,'value',0);
set(handles.edit1,'string',0.1); 
set(handles.PlaneRadius,'string',16); 
set(handles.EdgeWeight,'string',1);
set(handles.NormalWeight,'string',1);
% set(handles.PlaneWeight,'Min',0);
% set(handles.PlaneWeight,'Max',1);
% set(handles.PlaneWeight,'value',0.0);
global points;
global g_regions;
handles.PointIndices=[];
handles.points_=[];
handles.OrganIndices=cell(length(handles.OrganIds),1);
handles.OrganIndices_=cell(length(handles.OrganIds),1);
handles.SeedIndices=cell(length(handles.OrganIds),1);
handles.UnSegmentIndices=[];
handles.SegmentIndices=cell(length(handles.OrganIds),1);
handles.SampleId=[];
handles.Neighbors=[];
for i=1:size(handles.OrganIds,1)
   indices=g_regions{handles.OrganIds(i)};
   istart=length(handles.PointIndices)+1;
   handles.PointIndices=[handles.PointIndices;indices];
   iend=length(handles.PointIndices);
   handles.OrganIndices{i}=(istart:iend)';
   handles.OrganIndices_{i}=(istart:iend)';
end
handles.points_=points(handles.PointIndices,:);
handles.normals=find_normal_data(handles.points_);
 XStep=max(handles.points_(:,1))-min(handles.points_(:,1));
 YStep=max(handles.points_(:,2))-min(handles.points_(:,2));
 ZStep=max(handles.points_(:,3))-min(handles.points_(:,3));
 r=sqrt(XStep*XStep+YStep*YStep+ZStep*ZStep);
 set(handles.slider2,'Min',0);
 set(handles.slider2,'Max',r);
 set(handles.slider2,'value',r*0.05);
 set(handles.slider4,'Min',0);
 set(handles.slider4,'Max',r);
 set(handles.slider4,'value',r*0.05);

handles.SeedIds=[];
%handles.EMD_=computeEMD(handles.points_);
% Update handles structure
datacursormode  off;
handles.bSegment=false;
guidata(hObject, handles);
UpdateResults(handles,false);
% UIWAIT makes PostProcess wait for user response (see UIRESUME)
 uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = PostProcess_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
%varargout{1} = handles.bSegment;


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1


% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in SelectPointState.
function SelectPointState_Callback(hObject, eventdata, handles)
% hObject    handle to SelectPointState (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of SelectPointState
set(handles.View3DState,'value',0);
set(handles.SelectPointState,'value',1);
set(handles.DeletePointState,'value',0);
set(handles.SelectPointsState,'value',0);
set(handles.SelectSegmentAreaState,'value',0);
%set(handles,'select',0);
%global select;
handles.select=1;
handles.Neighbors=[];
 handles.SampleId=[];
guidata(hObject, handles);
datacursormode  on;

% --- Executes on button press in SelectPoints.
function SelectPoints_Callback(hObject, eventdata, handles)
% hObject    handle to SelectPoints (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if(handles.select==1)
    if(isempty(handles.UnSegmentIndices))
        msgbox('please select the points to be segmented first', 'warning'); 
        return;
    end
   unSegPts=handles.points_(handles.UnSegmentIndices,:);
   dcm_obj = datacursormode(handles.figure1);
   c_info = getCursorInfo(dcm_obj);
   if(~isempty(c_info))
       pos=c_info.Position;
       id=findPointId(unSegPts,pos);
       id=handles.UnSegmentIndices(id,:);
       index = get(handles.popupmenu1,'value');
       for i=1:handles.OrganNum
         if(i==index)
           handles.SeedIndices{i}=[handles.SeedIndices{i};id];
         else
           indices=handles.SeedIndices{i};
           indices(indices==id)=[];
           handles.SeedIndices{i}=indices;
         end
       end
       for i=1:handles.OrganNum
         handles.SeedIndices{i}=unique(handles.SeedIndices{i});
       end
       guidata(hObject, handles);
       UpdateResults(handles,true);
   end  
end

% --- Executes on button press in DeletePointState.
function DeletePointState_Callback(hObject, eventdata, handles)
% hObject    handle to DeletePointState (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of DeletePointState
set(handles.View3DState,'value',0);
set(handles.SelectPointState,'value',0);
set(handles.DeletePointState,'value',1);
set(handles.SelectPointsState,'value',0);
set(handles.SelectSegmentAreaState,'value',0);
%set(handles,'select',0);
%global select;
handles.Neighbors=[];
 handles.SampleId=[];
handles.select=2;
guidata(hObject, handles);
datacursormode  on;

% --- Executes on button press in DeletePoints.
function DeletePoints_Callback(hObject, eventdata, handles)
% hObject    handle to DeletePoints (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if(handles.select==2)
     if(isempty(handles.UnSegmentIndices))
        msgbox('please select the points to be segmented first', 'warning'); 
        return;
    end
   unSegPts=handles.points_(handles.UnSegmentIndices,:);
   dcm_obj = datacursormode(handles.figure1);
   c_info = getCursorInfo(dcm_obj);
   if(~isempty(c_info))
       pos=c_info.Position;
       id=findPointId(unSegPts,pos);
       id=handles.UnSegmentIndices(id);
       index = get(handles.popupmenu1,'value');
       indices=handles.SeedIndices{index};
       %%%%%%%%%%查找距离最近的点%%%%%%%%%%%%
       points=handles.points_;
       pts=points(indices,:);
       pt=points(id,:);
       dis=sum((pts-pt).*(pts-pt),2);
       [md,idx]=min(dis);
       id=indices(idx);
       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       indices(indices==id)=[];
       handles.SeedIndices{index}=indices;
       
       guidata(hObject, handles);
       UpdateResults(handles,true);
   end  
end

function ShowResults(handles)
global points;
global g_regions;
axes(handles.axes1);
  %cameraPos=get(handles.axes1,'CameraPosition');
 % cameraTarget=get(handles.axes1,'CameraTarget');
%  cameraUpVector=get(handles.axes1,'CameraUpVector');
 cla;
 color=MyGS.MYCOLOR;
%  if(~isempty(g_regions))
%    for i=1:size(handles.OrganIds,1)
%    indices=g_regions{handles.OrganIds(i)};
%    scatter3(points(indices,1),points(indices,2),points(indices,3),5,color(i,:), 'filled');
%    hold on;
%    end
%  end
 scatter3(handles.points_(:,1),handles.points_(:,2),handles.points_(:,3),5,[0 0 0], 'filled');
 axis off; axis equal; camorbit(0,0,'camera'); axis vis3d; view(0,0);view3d ZOOM;
 % set(handles.axes1,'CameraPosition',cameraPos);
%  set(handles.axes1,'CameraTarget',cameraTarget);
%  set(handles.axes1,'CameraUpVector',cameraUpVector);


function UpdateResults(handles,update)
global points;
global g_regions;
axes(handles.axes1);
if(update)
  cameraPos=get(handles.axes1,'CameraPosition');
  cameraTarget=get(handles.axes1,'CameraTarget');
  cameraUpVector=get(handles.axes1,'CameraUpVector');
end
 cla;
 color=MyGS.MYCOLOR;
 scatter3(handles.points_(handles.UnSegmentIndices,1),handles.points_(handles.UnSegmentIndices,2),handles.points_(handles.UnSegmentIndices,3),5,[0 0 0], 'filled');
 for i=1:length(handles.OrganIndices)
   indices=handles.OrganIndices{i};
   indices2=handles.SegmentIndices{i};
   scatter3(handles.points_(indices,1),handles.points_(indices,2),handles.points_(indices,3),5,color(i,:), 'filled');
   hold on;
   scatter3(handles.points_(indices2,1),handles.points_(indices2,2),handles.points_(indices2,3),5,color(i,:), 'filled');
   hold on;    
 end
 
 scatter3(handles.points_(handles.Neighbors,1),handles.points_(handles.Neighbors,2),handles.points_(handles.Neighbors,3),5,[0.5,0.5,0.5], 'filled');
 scatter3(handles.points_(handles.SampleId,1),handles.points_(handles.SampleId,2),handles.points_(handles.SampleId,3),40,[0.5,0.5,0.5], 'filled');

 hold on;
 for i=1:length(handles.SeedIndices)
   indices=handles.SeedIndices{i};
   scatter3(handles.points_(indices,1),handles.points_(indices,2),handles.points_(indices,3),50,color(i,:)/2, 'filled');
   hold on;    
 end


%  axis off; 
 axis equal; camorbit(0,0,'camera'); axis vis3d; view(0,0);view3d ZOOM;
  if(update)
 set(handles.axes1,'CameraPosition',cameraPos);
  set(handles.axes1,'CameraTarget',cameraTarget);
  set(handles.axes1,'CameraUpVector',cameraUpVector);
  end


% --- Executes on button press in View3DState.
function View3DState_Callback(hObject, eventdata, handles)
% hObject    handle to View3DState (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of View3DState
set(handles.View3DState,'value',1);
set(handles.SelectPointState,'value',0);
set(handles.DeletePointState,'value',0);
set(handles.SelectPointsState,'value',0);
set(handles.SelectSegmentAreaState,'value',0);
%set(handles,'select',0);
%global select;
handles.select=0;
handles.Neighbors=[];
 handles.SampleId=[];
guidata(hObject, handles);

 UpdateResults(handles,true);
datacursormode  off;


% --- Executes on button press in OrganClassification.
function OrganClassification_Callback(hObject, eventdata, handles)
% hObject    handle to OrganClassification (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
DataWeight = str2num(get(handles.edit1,'string'));
NeighborNum = str2num(get(handles.PlaneRadius,'string'));
EdgeWeight = str2num(get(handles.EdgeWeight,'string'));
NormalWeight = str2num(get(handles.NormalWeight,'string'));
SeedIndices=[];
SeedNum=length(handles.SeedIndices);
OrganNum=length(handles.OrganIndices);
%if(SeedNum<OrganNum)
%    msgbox('please select the sample points for every organ instance', 'warning'); 
%    return; 
% end
% if(OrganNum~=2)
%      msgbox('this segmentation is only suitable for 2 classification', 'warning'); 
%     return;     
% end
Regions=cell(length(handles.SeedIndices),1);
Knum=0;
 for i=1:size(handles.SeedIndices,1)
   indices=handles.OrganIndices{i};
   Regions{i}=[handles.SeedIndices{i};indices];  
   if(~isempty(Regions{i}))
       Knum=Knum+1;
   end
 end
%  if(Knum~=2)
 if(Knum<OrganNum)
     msgbox('this segmentation is only suitable for 2 classification', 'warning'); 
    return;     
 end
 UnSegIds= handles.UnSegmentIndices;
 if(length(UnSegIds)<OrganNum)
     msgbox('select segment points firstly', 'warning'); 
    return;     
 end
%[~, NewRegions]=classifyLeafPoints_EMD2(handles.points_,handles.EMD_,Regions,UnSegIds,sampleNum1,sampleNum2,weight,1);
NewRegions=classifyLeafPoints_MRF(handles.points_,handles.normals,Regions,UnSegIds,DataWeight,NeighborNum,EdgeWeight,NormalWeight); 
 for i=1:length(handles.SeedIndices)
    handles.SegmentIndices{i}=NewRegions{i};   
 end
 handles.bSegment=true;
 guidata(hObject, handles);
 UpdateResults(handles,true);
%  Labeling(handles);
 msgbox('finish'); 

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


% --- Executes on button press in SubmitResult.
function SubmitResult_Callback(hObject, eventdata, handles)
% hObject    handle to SubmitResult (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%global points;
if(handles.bSegment)
  global g_regions;  
  for i=1:length(handles.OrganIndices)
    indices1=handles.PointIndices(handles.OrganIndices{i});
    indices2=handles.PointIndices(handles.SegmentIndices{i});
    index= handles.OrganIds(i);
    g_regions{index}=[indices1;indices2];
  end
  Labeling(handles);
end
uiresume(gcbf);


function Labeling(handles)
global g_regions;
global points;
global g_pointlabels;
g_pointlabels=zeros(length(points),1);
for i=1:length(g_regions)
   ids=g_regions{i}; 
   g_pointlabels(ids)=i; 
end


% --- Executes on button press in SelectSegmentAreaState.
function SelectSegmentAreaState_Callback(hObject, eventdata, handles)
% hObject    handle to SelectSegmentAreaState (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of SelectSegmentAreaState
set(handles.View3DState,'value',0);
set(handles.SelectPointState,'value',0);
set(handles.DeletePointState,'value',0);
set(handles.SelectPointsState,'value',0);
set(handles.SelectSegmentAreaState,'value',1);
handles.select=3;
handles.Neighbors=[];
 handles.SampleId=[];
guidata(hObject, handles);
datacursormode  on;
% --- Executes on slider movement.
function slider2_Callback(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
Neighbors=findSegmentNeighbors(hObject,handles);
handles.Neighbors=Neighbors;
guidata(hObject, handles);
UpdateResults(handles,true);  
% --- Executes during object creation, after setting all properties.
function slider2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in AddSegmentArea.
function AddSegmentArea_Callback(hObject, eventdata, handles)
% hObject    handle to AddSegmentArea (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if(handles.select==3)
   handles.Neighbors=findSegmentNeighbors(hObject,handles);
   if(~isempty(handles.Neighbors))
   handles.UnSegmentIndices=[handles.UnSegmentIndices;handles.Neighbors]; 
   handles.UnSegmentIndices=unique(handles.UnSegmentIndices);
   for i=1:length(handles.OrganIndices)
      A=handles.OrganIndices{i};
      handles.OrganIndices{i}=setdiff(A,handles.UnSegmentIndices);
   end
   handles.Neighbors=[];
   handles.SampleId=[];
   guidata(hObject, handles);
   UpdateResults(handles,true);
end
end

% --- Executes on button press in DeleteSegmentArea.
function DeleteSegmentArea_Callback(hObject, eventdata, handles)
% hObject    handle to DeleteSegmentArea (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if(handles.select==3)
   handles.Neighbors=findSegmentNeighbors(hObject,handles);
   if(~isempty(handles.Neighbors))
     A=intersect(handles.UnSegmentIndices,handles.Neighbors);
     handles.UnSegmentIndices=setdiff(handles.UnSegmentIndices,A); 
     for i=1:length(handles.OrganIndices)
      B=intersect(A,handles.OrganIndices_{i});
      handles.OrganIndices{i}=[handles.OrganIndices{i};B];
     end
     handles.Neighbors=[];
     handles.SampleId=[];
     guidata(hObject, handles);
     UpdateResults(handles,true);
   end
end

% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global g_regions;
 handles.PointIndices=[];
 handles.Neighbors=[];
 handles.SampleId=[];
 handles.OrganIndices=cell(length(handles.OrganIds),1);
 handles.OrganIndices_=cell(length(handles.OrganIds),1);
 handles.SeedIndices=cell(length(handles.OrganIds),1);
 handles.UnSegmentIndices=[];
 for i=1:size(handles.OrganIds,1)
   indices=g_regions{handles.OrganIds(i)};
   istart=length(handles.PointIndices)+1;
   handles.PointIndices=[handles.PointIndices;indices];
   iend=length(handles.PointIndices);
   handles.OrganIndices{i}=(istart:iend)';
   handles.OrganIndices_{i}=(istart:iend)';
 end
guidata(hObject, handles);
UpdateResults(handles,true);


function [Neighbors]=findSegmentNeighbors(hObject,handles)
   if(isempty(handles.SampleId))
       %msgbox('please select a sample point first', 'warning'); 
       return;
   end
   r=get(handles.slider2,'value');
   points_=handles.points_;
   bpt=points_(handles.SampleId,:);
   Neighbors=[];
   if(~isnan(bpt(1)))
   Neighbors=findNeighborId(points_,bpt,r);
%    else
%    handles.Neighbors=[];    
   end
%    guidata(hObject, handles);
%    UpdateResults(handles,true);  

function [Neighbors]=findSegmentNeighbors2(hObject,handles)
   if(isempty(handles.SampleId))
       %msgbox('please select a sample point first', 'warning'); 
       return;
   end
   r=get(handles.slider4,'value');
   points_=handles.points_;
   bpt=points_(handles.SampleId,:);
   Neighbors=[];
   if(~isnan(bpt(1)))
   Neighbors=findNeighborId(points_,bpt,r); 
   end







function ResetIndices(hObject,handles)
 global g_regions;
 handles.PointIndices=[];
 handles.Neighbors=[];
 handles.SampleId=[];
 handles.OrganIndices=cell(length(handles.OrganIds),1);
 handles.OrganIndices_=cell(length(handles.OrganIds),1);
 handles.SeedIndices=cell(length(handles.OrganIds),1);
 handles.UnSegmentIndices=[];
 for i=1:size(handles.OrganIds,1)
   indices=g_regions{handles.OrganIds(i)};
   istart=length(handles.PointIndices)+1;
   handles.PointIndices=[handles.PointIndices;indices];
   iend=length(handles.PointIndices);
   handles.OrganIndices{i}=(istart:iend)';
   handles.OrganIndices_{i}=(istart:iend)';
 end
 guidata(hObject, handles);


% --- Executes on button press in SelectYes.
function SelectYes_Callback(hObject, eventdata, handles)
% hObject    handle to SelectYes (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if(handles.select==3)
   dcm_obj = datacursormode(handles.figure1);
   c_info = getCursorInfo(dcm_obj);
   if(~isempty(c_info))
       pos=c_info.Position;
       id=findPointId(handles.points_,pos);
       handles.SampleId=id;
       handles.Neighbors=findSegmentNeighbors(hObject,handles);
       guidata(hObject, handles);
%    UpdateResults(handles,true);  
       UpdateResults(handles,true);
   end  
end


% --- Executes on button press in RemoveSegmentResults.
function RemoveSegmentResults_Callback(hObject, eventdata, handles)
% hObject    handle to RemoveSegmentResults (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
OrganNum=length(handles.OrganIndices_);
 for i=1:OrganNum
    handles.SegmentIndices{i}=[];   
 end
 handles.bSegment=false;
 guidata(hObject, handles);
 UpdateResults(handles,true);


% --- Executes on button press in pushbutton10.
function pushbutton10_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
sampleNum1 = str2num(get(handles.edit1,'string'));
sampleNum2 = str2num(get(handles.PlaneRadius,'string'));
weight=get(handles.PlaneWeight,'value');
SeedIndices=[];
SeedNum=length(handles.SeedIndices);
OrganNum=length(handles.OrganIndices);
%if(SeedNum<OrganNum)
%    msgbox('please select the sample points for every organ instance', 'warning'); 
%    return; 
% end

for i=1:length(handles.SeedIndices)
SeedIndices=[SeedIndices;handles.SeedIndices{i}];
end

Regions=cell(length(SeedIndices),1);
 for i=1:size(SeedIndices,1)
   Regions{i}=SeedIndices(i);   
 end
 UnSegIds= handles.UnSegmentIndices;
%[~, NewRegions]=classifyLeafPoints_EMD2(handles.points_,handles.EMD_,Regions,UnSegIds,sampleNum1,sampleNum2,weight,1);
[~, NewRegions]=classifyLeafPoints_EMD2(handles.points_,handles.EMD_,Regions,UnSegIds,sampleNum1,sampleNum2,weight,1); 
 start=1;
 last=1;
 for i=1:length(handles.SeedIndices)
   indices=length(handles.SeedIndices{i});
   last=start+indices-1;
   handles.SegmentIndices{i}=[];
   for j=start:last
     handles.SegmentIndices{i}=[handles.SegmentIndices{i};NewRegions{j}];   
   end
   start=last+1;
 end
 handles.bSegment=true;
 guidata(hObject, handles);
 UpdateResults(handles,true);
% Labeling(handles);
 msgbox('finish'); 



function PlaneRadius_Callback(hObject, eventdata, handles)
% hObject    handle to PlaneRadius (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of PlaneRadius as text
%        str2double(get(hObject,'String')) returns contents of PlaneRadius as a double


% --- Executes during object creation, after setting all properties.
function PlaneRadius_CreateFcn(hObject, eventdata, handles)
% hObject    handle to PlaneRadius (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function PlaneWeight_Callback(hObject, eventdata, handles)
% hObject    handle to PlaneWeight (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function PlaneWeight_CreateFcn(hObject, eventdata, handles)
% hObject    handle to PlaneWeight (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in pushbutton11.
function pushbutton11_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
  index = get(handles.popupmenu1,'value');
  for i=1:handles.OrganNum
    if(i==index)
       handles.SeedIndices{i}=[];
    end
  end
  guidata(hObject, handles);
  UpdateResults(handles,true);



function EdgeWeight_Callback(hObject, eventdata, handles)
% hObject    handle to EdgeWeight (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of EdgeWeight as text
%        str2double(get(hObject,'String')) returns contents of EdgeWeight as a double


% --- Executes during object creation, after setting all properties.
function EdgeWeight_CreateFcn(hObject, eventdata, handles)
% hObject    handle to EdgeWeight (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function NormalWeight_Callback(hObject, eventdata, handles)
% hObject    handle to NormalWeight (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of NormalWeight as text
%        str2double(get(hObject,'String')) returns contents of NormalWeight as a double


% --- Executes during object creation, after setting all properties.
function NormalWeight_CreateFcn(hObject, eventdata, handles)
% hObject    handle to NormalWeight (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in SelectPointsState.
function SelectPointsState_Callback(hObject, eventdata, handles)
% hObject    handle to SelectPointsState (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of SelectPointsState
set(handles.View3DState,'value',0);
set(handles.SelectPointState,'value',0);
set(handles.DeletePointState,'value',0);
set(handles.SelectPointsState,'value',1);
set(handles.SelectSegmentAreaState,'value',0);
handles.select=4;
handles.Neighbors=[];
handles.SampleId=[];
guidata(hObject, handles);
datacursormode  on;
% --- Executes on slider movement.
function slider4_Callback(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
Neighbors=findSegmentNeighbors2(hObject,handles);
handles.Neighbors=Neighbors;
guidata(hObject, handles);
UpdateResults(handles,true);  

% --- Executes during object creation, after setting all properties.
function slider4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in SelectSegmentedPoints.
function SelectSegmentedPoints_Callback(hObject, eventdata, handles)
% hObject    handle to SelectSegmentedPoints (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if(handles.select==4)
   dcm_obj = datacursormode(handles.figure1);
   c_info = getCursorInfo(dcm_obj);
   if(~isempty(c_info))
       pos=c_info.Position;
       id=findPointId(handles.points_,pos);
       handles.SampleId=id;
       handles.Neighbors=findSegmentNeighbors2(hObject,handles);
       guidata(hObject, handles);
%    UpdateResults(handles,true);  
       UpdateResults(handles,true);
   end  
end

% --- Executes on selection change in popupmenu2.
function popupmenu2_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu2 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu2


% --- Executes during object creation, after setting all properties.
function popupmenu2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in SegmentByHands.
function SegmentByHands_Callback(hObject, eventdata, handles)
% hObject    handle to SegmentByHands (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if(handles.select==4)
   handles.Neighbors=findSegmentNeighbors2(hObject,handles);
   index = get(handles.popupmenu2,'value');
   AddIndices=[];
   for i=1:length(handles.OrganIndices)
      A=handles.OrganIndices{i};
      handles.OrganIndices{i}=setdiff(A,handles.Neighbors);
      if(i==index)
        C=[A;handles.Neighbors];  
        C=unique(C);  
        handles.OrganIndices{i}=C;
      end
   end
   
   handles.Neighbors=[];
   handles.SampleId=[];
   handles.bSegment=true;
   guidata(hObject, handles);
   UpdateResults(handles,true);
   
end
    
    
    
   
