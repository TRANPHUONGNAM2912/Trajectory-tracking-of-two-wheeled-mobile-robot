function phan2(block)

setup(block);


function setup(block)

% Register number of ports
block.NumInputPorts  = 5;
block.NumOutputPorts = 6;

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


function Outputs(block)

x=block.InputPort(1).Data;
y=block.InputPort(2).Data;
theta=block.InputPort(3).Data;
vc=block.InputPort(4).Data;
wc=block.InputPort(5).Data;


global x_old;
global y_old;
global theta_old;
global vc_old;
global wc_old;

x_old=0.001;
y_old=0.001;
theta_old=0.001;
vc_old=0.001;
wc_old=0.001;

Jac11=(x-x_old)/(vc-vc_old);
Jac12=(x-x_old)/(wc-wc_old);
Jac21=(y-y_old)/(vc-vc_old);
Jac22=(y-y_old)/(wc-wc_old);
Jac31=(theta-theta_old)/(vc-vc_old);
Jac32=(theta-theta_old)/(wc-wc_old);

x_old=x;
y_old=y;
theta_old=theta;
vc_old=vc;
wc_old=wc;

block.OutputPort(1).Data=Jac11;
block.OutputPort(2).Data=Jac12;
block.OutputPort(3).Data=Jac21;
block.OutputPort(4).Data=Jac22;
block.OutputPort(5).Data=Jac31;
block.OutputPort(6).Data=Jac32;







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

