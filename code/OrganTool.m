function varargout = OrganTool(varargin)
% ORGANTOOL MATLAB code for OrganTool.fig
%      ORGANTOOL, by itself, creates a new ORGANTOOL or raises the existing
%      singleton*.
%
%      H = ORGANTOOL returns the handle to a new ORGANTOOL or the handle to
%      the existing singleton*.
%
%      ORGANTOOL('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ORGANTOOL.M with the given input arguments.
%
%      ORGANTOOL('Property','Value',...) creates a new ORGANTOOL or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before OrganTool_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to OrganTool_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help OrganTool

% Last Modified by GUIDE v2.5 04-May-2020 18:12:00

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @OrganTool_OpeningFcn, ...
                   'gui_OutputFcn',  @OrganTool_OutputFcn, ...
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


% --- Executes just before OrganTool is made visible.
function OrganTool_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to OrganTool (see VARARGIN)

% Choose default command line output for OrganTool
handles.output = hObject;
handles.SelectId=-1;
handles.Select=1;

set(handles.View3D,'value',1);
%set(handles.LeafTipNum,'string',10); 
set(handles.SmoothVar,'string',10); 
set(handles.EdgeLenVar,'string',5); 
set(handles.SelectOrgan,'value',0);
global g_regions;
handles.BoundingBox=cell(length(g_regions),1);
handles.TriArea=zeros(length(g_regions),1);
handles.PhenoTraits=zeros(length(g_regions)+1,4);
set(handles.OrganLabel,'value',0);
datacursormode  off;
% Update handles structure
guidata(hObject, handles);
ShowResults(handles);











% Update handles structure
guidata(hObject, handles);

% UIWAIT makes OrganTool wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = OrganTool_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on selection change in popupmenu_organ.
function popupmenu_organ_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu_organ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu_organ contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu_organ


% --- Executes during object creation, after setting all properties.
function popupmenu_organ_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu_organ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in SaveAllOrganPoints.
function SaveAllOrganPoints_Callback(hObject, eventdata, handles)
% hObject    handle to SaveAllOrganPoints (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global points;
global original_points;
global g_regions;
Path=uigetdir();
if(Path==0)return;end
for i=1:length(g_regions)
defaultName = ['\' num2str(i) '.ply'];
fname=[Path defaultName];
indices=g_regions{i};
organPoints=original_points(indices,:);
ptCloud=pointCloud(organPoints);
pcwrite(ptCloud,fname);
end
msgbox('finish'); 

% --- Executes on button press in View3D.
function View3D_Callback(hObject, eventdata, handles)
% hObject    handle to View3D (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of View3D
set(handles.View3D,'value',1);
set(handles.SelectOrgan,'value',0);
handles.Select=1;
handles.SelectId=-1;
guidata(hObject, handles);
datacursormode  off;
UpdateResults(handles);
% --- Executes on button press in SelectOrgan.
function SelectOrgan_Callback(hObject, eventdata, handles)
% hObject    handle to SelectOrgan (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of SelectOrgan
set(handles.View3D,'value',0);
set(handles.SelectOrgan,'value',1);
handles.Select=2;
guidata(hObject, handles);
datacursormode  on;
dcm_obj = datacursormode(handles.figure1);
dcm_obj.DisplayStyle='window';

% --- Executes on button press in SaveOrganPoint.
function SaveOrganPoint_Callback(hObject, eventdata, handles)
% hObject    handle to SaveOrganPoint (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global points;
global original_points;
global g_regions;
SelectId=handles.SelectId;
if(SelectId==-1)
  msgbox('please select an organ first', 'warning'); 
  return;
end
indices=g_regions{handles.SelectId};
organPoints=original_points(indices,:);
defaultName = [num2str(handles.SelectId) '.ply'];
filter = {'*.ply'};
[fileName, Path,fileindex] = uiputfile(filter,'save point cloud',defaultName);
if(fileindex==0)
    return;
end
fname=[Path fileName];
ptCloud=pointCloud(organPoints);
pcwrite(ptCloud,fname);
msgbox('finish'); 




% --- Executes on button press in ExtractStemTraits.
function ExtractStemTraits_Callback(hObject, eventdata, handles)
% hObject    handle to ExtractStemTraits (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 SelectId=handles.SelectId;
if(SelectId==-1)
  msgbox('please select an organ first', 'warning'); 
  return;
end
 global points;
 global g_regions;
 indices=g_regions{handles.SelectId};
 StemPoints=points(indices,:);
 [StemLen,StemRadius,~,~]=StemTrait(StemPoints,5);
 defaultName = [num2str(handles.SelectId) '.txt'];
 filter = {'*.txt'};
 [fileName, Path,filterindex] = uiputfile(filter,'organ phenotrait file',defaultName);
 if(filterindex==0)return;end
 txtName=[Path fileName];
 fid=fopen(txtName,'w');
 fprintf(fid,'Stemlength %f\r\n',StemLen);
 fprintf(fid,'StemRadius %f\r\n',StemRadius*2);
 fclose(fid);
 handles.PhenoTraits(handles.SelectId,:)=[StemLen StemRadius*2 0 0];
 msgbox('finish'); 
% --- Executes on button press in GenerateOrganMesh.
function GenerateOrganMesh_Callback(hObject, eventdata, handles)
% hObject    handle to GenerateOrganMesh (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
SelectId=handles.SelectId;
if(SelectId==-1)
  msgbox('please select an organ first', 'warning'); 
  return;
end
global points;
global original_points;
global g_regions;
indices=g_regions{handles.SelectId};
organPoints=original_points(indices,:);
Smooth= str2num(get(handles.SmoothVar,'string'));
EdgeLen= str2num(get(handles.EdgeLenVar,'string'));
Area=Point2Mesh(organPoints,Smooth,EdgeLen);
handles.TriArea(handles.SelectId)=Area;
guidata(hObject, handles);
%msgbox('finish'); 
% --- Executes on button press in SelectOrganButton.
function SelectOrganButton_Callback(hObject, eventdata, handles)
% hObject    handle to SelectOrganButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if(handles.Select==2)
   dcm_obj = datacursormode(handles.figure1);
   c_info = getCursorInfo(dcm_obj);
   global points;
   global g_pointlabels;
   global g_regions;
   if(~isempty(c_info))
       pos=c_info.Position;
       I=find(ismember(points,pos,'rows'));
       I=unique(I);
       organId=g_pointlabels(I);
       handles.SelectId=organId;
       if(isempty(handles.BoundingBox{organId}))
          indices= g_regions{organId};
          organPts=points(indices,:);
          [~,handles.BoundingBox{organId}]=computeOBB(organPts);
       end
       box=handles.BoundingBox{organId};
       width=box(4)-box(3);
       set(handles.EdgeLenVar,'string',width/10); 
       set(handles.OrganLabel,'string',organId); 
       guidata(hObject, handles);
       UpdateResults(handles);
   end  
end













% function ShowResults(handles)
% global points;
% global g_regions;
% axes(handles.axes1);
%   cameraPos=get(handles.axes1,'CameraPosition');
%   cameraTarget=get(handles.axes1,'CameraTarget');
%   cameraUpVector=get(handles.axes1,'CameraUpVector');
%  cla;
%  color=MyGS.MYCOLOR;
%  scatter3(points(:,1),points(:,2),points(:,3),5,[0 0 0], 'filled');
%  hold on;
%  if(~isempty(g_regions))
%    for i=1:size(g_regions,1)
%    indices=g_regions{i};
%    scatter3(points(indices,1),points(indices,2),points(indices,3),5,color(i,:), 'filled');
%    hold on;
%    end
%  end
%  axis off; axis equal; camorbit(0,0,'camera'); axis vis3d; view(0,0);view3d ZOOM;
%   set(handles.axes1,'CameraPosition',cameraPos);
%   set(handles.axes1,'CameraTarget',cameraTarget);
%   set(handles.axes1,'CameraUpVector',cameraUpVector);



function ShowResults(handles)
global points;
global g_regions;
axes(handles.axes1);
  %cameraPos=get(handles.axes1,'CameraPosition');
 % cameraTarget=get(handles.axes1,'CameraTarget');
%  cameraUpVector=get(handles.axes1,'CameraUpVector');
 cla;
 color=MyGS.MYCOLOR;
 scatter3(points(:,1),points(:,2),points(:,3),5,[0 0 0], 'filled');
 hold on;
 if(~isempty(g_regions))
   for i=1:length(g_regions)
       if(handles.SelectId==i)
           continue;
       end
   indices=g_regions{i};
   scatter3(points(indices,1),points(indices,2),points(indices,3),5,color(i,:), 'filled');
   hold on;
   end
 end
 axis off; axis equal; camorbit(0,0,'camera'); axis vis3d; view(0,0);view3d ZOOM;
 % set(handles.axes1,'CameraPosition',cameraPos);
%  set(handles.axes1,'CameraTarget',cameraTarget);
%  set(handles.axes1,'CameraUpVector',cameraUpVector);


function UpdateResults(handles)
global points;
global g_regions;
axes(handles.axes1);
  cameraPos=get(handles.axes1,'CameraPosition');
  cameraTarget=get(handles.axes1,'CameraTarget');
  cameraUpVector=get(handles.axes1,'CameraUpVector');
 cla;
 color=MyGS.MYCOLOR;
 scatter3(points(:,1),points(:,2),points(:,3),5,[0 0 0], 'filled');
 hold on;
 if(~isempty(g_regions))
   for i=1:length(g_regions)
       if(handles.SelectId==i)
           continue;
       end
   indices=g_regions{i};
   scatter3(points(indices,1),points(indices,2),points(indices,3),5,color(i,:), 'filled');
   hold on;
   end
 end
 axis off; axis equal; camorbit(0,0,'camera'); axis vis3d; view(0,0);view3d ZOOM;
  set(handles.axes1,'CameraPosition',cameraPos);
  set(handles.axes1,'CameraTarget',cameraTarget);
  set(handles.axes1,'CameraUpVector',cameraUpVector);



function SmoothVar_Callback(hObject, eventdata, handles)
% hObject    handle to SmoothVar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of SmoothVar as text
%        str2double(get(hObject,'String')) returns contents of SmoothVar as a double


% --- Executes during object creation, after setting all properties.
function SmoothVar_CreateFcn(hObject, eventdata, handles)
% hObject    handle to SmoothVar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function EdgeLenVar_Callback(hObject, eventdata, handles)
% hObject    handle to EdgeLenVar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of EdgeLenVar as text
%        str2double(get(hObject,'String')) returns contents of EdgeLenVar as a double


% --- Executes during object creation, after setting all properties.
function EdgeLenVar_CreateFcn(hObject, eventdata, handles)
% hObject    handle to EdgeLenVar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function OrganLen_Callback(hObject, eventdata, handles)
% hObject    handle to OrganLen (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of OrganLen as text
%        str2double(get(hObject,'String')) returns contents of OrganLen as a double


% --- Executes during object creation, after setting all properties.
function OrganLen_CreateFcn(hObject, eventdata, handles)
% hObject    handle to OrganLen (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function OrganWidth_Callback(hObject, eventdata, handles)
% hObject    handle to OrganWidth (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of OrganWidth as text
%        str2double(get(hObject,'String')) returns contents of OrganWidth as a double


% --- Executes during object creation, after setting all properties.
function OrganWidth_CreateFcn(hObject, eventdata, handles)
% hObject    handle to OrganWidth (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function OrganArea_Callback(hObject, eventdata, handles)
% hObject    handle to OrganArea (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of OrganArea as text
%        str2double(get(hObject,'String')) returns contents of OrganArea as a double


% --- Executes during object creation, after setting all properties.
function OrganArea_CreateFcn(hObject, eventdata, handles)
% hObject    handle to OrganArea (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function OrganLabel_Callback(hObject, eventdata, handles)
% hObject    handle to OrganLabel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of OrganLabel as text
%        str2double(get(hObject,'String')) returns contents of OrganLabel as a double


% --- Executes during object creation, after setting all properties.
function OrganLabel_CreateFcn(hObject, eventdata, handles)
% hObject    handle to OrganLabel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in ChangeOrganLabel.
function ChangeOrganLabel_Callback(hObject, eventdata, handles)
% hObject    handle to ChangeOrganLabel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
SelectId=handles.SelectId;
if(SelectId==-1)
  msgbox('please select an organ first', 'warning'); 
  return;
end
organlabel= str2num(get(handles.OrganLabel,'string'));
global points;
global g_pointlabels;
global g_regions;
if(organlabel>length(g_regions))
    organlabel=length(g_regions);
end
indicesSrc=g_regions{handles.SelectId};
indicesDst=g_regions{organlabel};
g_pointlabels(indicesSrc)=organlabel;
g_pointlabels(indicesDst)=handles.SelectId;
g_regions{organlabel}=indicesSrc;
g_regions{handles.SelectId}=indicesDst;
handles.SelectId=organlabel;
set(handles.OrganLabel,'string',organlabel); 
guidata(hObject, handles);
UpdateResults(handles);


% --- Executes on button press in ExtractLeafTraits.
function ExtractLeafTraits_Callback(hObject, eventdata, handles)
% hObject    handle to ExtractLeafTraits (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
  SelectId=handles.SelectId;
if(SelectId==-1)
  msgbox('please select an organ first', 'warning'); 
  return;
end
  global points;
  global g_regions;
  indices=g_regions{handles.SelectId};
  LeafPoints=points(indices,:);
  LeafArea=handles.TriArea(handles.SelectId);
  [LeafLen,LeafWidth,LeafAngle,~]=LeafTrait(LeafPoints);
  if(LeafArea==0) LeafArea=0.75*LeafLen*LeafWidth;end
  handles.PhenoTraits(handles.SelectId,:)=[LeafLen LeafWidth LeafArea LeafAngle];
  defaultName = [num2str(handles.SelectId) '.txt'];
  filter = {'*.txt'};
  [fileName, Path,filterindex] = uiputfile(filter,'organ phenotrait file',defaultName);
  if(filterindex==0)return;end
  txtName=[Path fileName];
  fid=fopen(txtName,'w');
  fprintf(fid,'leaflength %f\r\n',LeafLen);
  fprintf(fid,'leafwidth %f\r\n',LeafWidth);
  fprintf(fid,'leafArea %f\r\n',LeafArea);  
  fprintf(fid,'leafAngle %f\r\n',LeafAngle);
  fclose(fid);
  msgbox('finish'); 
    %%%%%%%%%%¾¥%%%%%%%%%%%%%
   
 
 %sprintf('Len:%g  Width:%g  Area:%g   Angle:%g',LeafLen,LeafWidth,LeafArea,LeafAngle);
 
 
 % --- Executes on button press in ExtractEarTraits.
function ExtractEarTraits_Callback(hObject, eventdata, handles)
% hObject    handle to ExtractEarTraits (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in AllTraits.
function AllTraits_Callback(hObject, eventdata, handles)
% hObject    handle to AllTraits (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
  global points;
  global g_regions;
  for i=1:length(g_regions)
    if(i==1)
      indices=g_regions{i};
      StemPoints=points(indices,:);
      [StemLen,StemRadius,~,~]=StemTrait(StemPoints,5);
      handles.PhenoTraits(i,:)=[StemLen StemRadius*2 0 0]; 
      continue; 
    end
    indices=g_regions{i};
    LeafPoints=points(indices,:);
    LeafArea=handles.TriArea(i);
    [LeafLen,LeafWidth,LeafAngle,~]=LeafTrait(LeafPoints);
    if(LeafArea==0) LeafArea=0.75*LeafLen*LeafWidth;end
    handles.PhenoTraits(i,:)=[LeafLen LeafWidth LeafArea LeafAngle];
  end
  pointX=points(:,1);pointY=points(:,2);
  minX=min(pointX); maxX=max(pointX);
  minY=min(pointY); maxY=max(pointY);
  PlantHeight=abs(maxX-minX);
  PlantDiameter=abs(maxY-minY);
  handles.PhenoTraits(end,:)=[PlantHeight PlantDiameter 0 0];
%%%%%%%%%%%%%%%%%´æÎÄ¼þ
  defaultName = ['Traits.txt'];
  filter = {'*.txt'};
  [fileName, Path,filterindex] = uiputfile(filter,'organ phenotrait file',defaultName);
  if(filterindex==0)return;end
  txtName=[Path fileName];
  fid=fopen(txtName,'w');
  stemHeight=handles.PhenoTraits(1,1);
  stemDiameter=handles.PhenoTraits(1,2);
  fprintf(fid,'Stem StemHeight %f\r\n',stemHeight);
  fprintf(fid,'Stem StemDiameter %f\r\n',stemDiameter); 
  for i=2:length(handles.PhenoTraits)-1
  LeafLen=handles.PhenoTraits(i,1);
  LeafWidth=handles.PhenoTraits(i,2);
  LeafArea=handles.PhenoTraits(i,3);
  LeafAngle=handles.PhenoTraits(i,4);
  fprintf(fid,'Leaf%d leaflength %f\r\n',i,LeafLen);
  fprintf(fid,'Leaf%d leafwidth %f\r\n',i,LeafWidth);
  fprintf(fid,'Leaf%d leafArea %f\r\n',i,LeafArea);  
  fprintf(fid,'Leaf%d leafAngle %f\r\n',i,LeafAngle);
  end
  PlantHeight=handles.PhenoTraits(end,1);
  PlantDiameter=handles.PhenoTraits(end,2);
  fprintf(fid,'Plant PlantHeight %f\r\n',PlantHeight);
  fprintf(fid,'Plant PlantDiameter %f\r\n',PlantDiameter);
  fclose(fid);
msgbox('finish'); 




% --- Executes on button press in PlantTraits.
function PlantTraits_Callback(hObject, eventdata, handles)
% hObject    handle to PlantTraits (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
  global points;
  pointX=points(:,1);
  pointY=points(:,2);
  minX=min(pointX); maxX=max(pointX);
  minY=min(pointY); maxY=max(pointY);
  PlantHeight=abs(maxX-minX);
  PlantDiameter=abs(maxY-minY);
 defaultName = ['plant.txt'];
 filter = {'*.txt'};
 [fileName,Path,filterindex] = uiputfile(filter,'plant phenotrait file',defaultName);
 if(filterindex==0)return;end
 txtName=[Path fileName];
 fid=fopen(txtName,'w');
 fprintf(fid,'PlantHeight %f\r\n',PlantHeight);
 fprintf(fid,'PlantDiameter %f\r\n',PlantDiameter);
 fclose(fid);
 handles.PhenoTraits(end,:)=[PlantHeight PlantDiameter 0 0];
msgbox('finish'); 

% --- Executes on button press in pushbutton11.
function pushbutton11_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global points;
global original_points;
global g_pointlabels;
filter = {'*.txt'};
[fileName, Path] = uiputfile(filter);
fid=fopen([Path fileName],'w');
for i=1:size(points,1)
      pt=original_points(i,:);
      label=g_pointlabels(i);
      fprintf(fid,'%f %f %f %d\r\n',pt(1),pt(2),pt(3),label);
end
fclose(fid);
msgbox('finish'); 

% --- Executes on button press in pushbutton11.
function pushbutton12_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
