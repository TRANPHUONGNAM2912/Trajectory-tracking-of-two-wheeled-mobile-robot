function phan3(block)

setup(block);


function setup(block)

% Register number of ports
block.NumInputPorts  = 11;
block.NumOutputPorts = 3;

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;



% Register parameters
block.NumDialogPrms     = 0;
block.SampleTimes = [0 0];
block.SimStateCompliance = 'DefaultSimState';


block.RegBlockMethod('Outputs', @Outputs);     % Required
block.RegBlockMethod('SetInputPortSamplingMode',@SetInpPortFrameData);
block.RegBlockMethod('Terminate', @Terminate); % Required


%%
function Outputs(block)
ex=block.InputPort(1).Data;
ey=block.InputPort(2).Data;
eth=block.InputPort(3).Data;
theta=block.InputPort(4).Data;
vr=block.InputPort(5).Data;
Jacv11=block.InputPort(6).Data;
Jacv12=block.InputPort(7).Data;
Jacv21=block.InputPort(8).Data;
Jacv22=block.InputPort(9).Data;
Jacv31=block.InputPort(10).Data;
Jacv32=block.InputPort(11).Data;

global Kx_old
global Ky_old
global Kth_old

global etaX;
global etaY;
global etaTh;

global DKx_old;
global DKy_old;
global DKth_old;

global Betax;
global Betay;
global Betath;

DKx_old=0;
DKy_old=0;
DKth_old=0;

Kx_old=1.5;
Ky_old=2.0;
Kth_old=3.0;
% 
% syms Kx
% syms Ky
% syms Kth

etaX=0.01;
etaY=0.01;
etaTh=0.01;

Betax=0.01;
Betay=0.01;
Betath=0.01;
ep=[ex;ey;eth];

Te=[cos(theta) sin(theta) 0;-sin(theta) cos(theta) 0;0 0 1];

Jacv=[sign(Jacv11) sign(Jacv12);sign(Jacv21) sign(Jacv22);sign(Jacv31) sign(Jacv32)];

Der1=[ex 0 0; 0 vr*ey vr*sin(eth)];
Der=-ep'*Te*Jacv*Der1;

DKx=-etaX*Der(1)+Betax*DKx_old;
DKy=-etaY*Der(2)+Betay*DKy_old;
DKth=-etaTh*Der(3)+Betath*DKth_old;

DKx_old=DKx;
DKy_old=DKy;
DKth_old=DKth;

Kx=Kx_old+DKx;
Ky=Ky_old+DKy;
Kth=Kth_old+DKth;

Kx_old=Kx;
Ky_old=Ky;
Kth_old=Kth;
% 
%  Kx=1;
%  Ky=2;
%  Kth=3;
block.OutputPort(1).Data=Kx;
block.OutputPort(2).Data=Ky;
block.OutputPort(3).Data=Kth;
%end Outputs

%%
%% Update:
%%   Functionality    : Called to update discrete states
%%                      during simulation step
%%   Required         : No
%%   C-MEX counterpart: mdlUpdate
%%
function SetInpPortFrameData(block, idx, fd)
  block.InputPort(idx).SamplingMode = fd;
  for i =1:block.NumOutputPorts
      block.OutputPort(i).SamplingMode= fd;
  end

%end Update

%%
%% Derivatives:
%%   Functionality    : Called to update derivatives of
%%                      continuous states during simulation step
%%   Required         : No
%%   C-MEX counterpart: mdlDerivatives
%%


%end Derivatives

%%
%% Terminate:
%%   Functionality    : Called at the end of simulation for cleanup
%%   Required         : Yes
%%   C-MEX counterpart: mdlTerminate
%%
function Terminate(block)

%end Terminate

