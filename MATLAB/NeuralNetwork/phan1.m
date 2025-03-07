function phan1(block)
%MSFUNTMPL_BASIC A Template for a Level-2 MATLAB S-Function
%   The MATLAB S-function is written as a MATLAB function with the
%   same name as the S-function. Replace 'msfuntmpl_basic' with the 
%   name of your S-function.
%
%   It should abe noted that the MATLAB S-function is very similar
%   to Level-2 C-Mex S-functions. You should be able to get more
%   information for each of the block methods by referring to the
%   documentation for C-Mex S-functions.
%
%   Copyright 2003-2010 The MathWorks, Inc.

%%
%% The setup method is used to set up the basic attributes of the
%% S-function such as ports, parameters, etc. Do not add any other
%% calls to the main body of the function.
%%
setup(block);

%endfunction

%% Function: setup ===================================================
%% Abstract:
%%   Set up the basic characteristics of the S-function block such as:
%%   - Input ports
%%   - Output ports
%%   - Dialog parameters
%%   - Options
%%
%%   Required         : Yes
%%   C-Mex counterpart: mdlInitializeSizes
%%
function setup(block)

% Register number of ports
block.NumInputPorts  = 5;
block.NumOutputPorts = 6;

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;



% Register parameters
block.NumDialogPrms     = 0;

% Register sample times
%  [0 offset]            : Continuous sample time
%  [positive_num offset] : Discrete sample time
%
%  [-1, 0]               : Inherited sample time
%  [-2, 0]               : Variable sample time
block.SampleTimes = [0 0];

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'CustomSimState',  < Has GetSimState and SetSimState methods
%    'DisallowSimState' < Error out when saving or restoring the model sim state
block.SimStateCompliance = 'DefaultSimState';

%% -----------------------------------------------------------------
%% The MATLAB S-function uses an internal registry for all
%% block methods. You should register all relevant methods
%% (optional and required) as illustrated below. You may choose
%% any suitable name for the methods and implement these methods
%% as local functions within the same file. See comments
%% provided for each function for more information.
%% -----------------------------------------------------------------


block.RegBlockMethod('Outputs', @Outputs);     % Required
block.RegBlockMethod('SetInputPortSamplingMode',@SetInpPortFrameData);
block.RegBlockMethod('Terminate', @Terminate); % Required

%end setup

%%
%% PostPropagationSetup:
%%   Functionality    : Setup work areas and state variables. Can
%%                      also register run-time methods here
%%   Required         : No
%%   C-Mex counterpart: mdlSetWorkWidths
%%


%%
%% InitializeConditions:
%%   Functionality    : Called at the start of simulation and if it is 
%%                      present in an enabled subsystem configured to reset 
%%                      states, it will be called when the enabled subsystem
%%                      restarts execution to reset the states.
%%   Required         : No
%%   C-MEX counterpart: mdlInitializeConditions
%%


%end InitializeConditions


%%
%% Start:
%%   Functionality    : Called once at start of model execution. If you
%%                      have states that should be initialized once, this 
%%                      is the place to do it.
%%   Required         : No
%%   C-MEX counterpart: mdlStart
%%


%end Start

%%
%% Outputs:
%%   Functionality    : Called to generate block outputs in
%%                      simulation step
%%   Required         : Yes
%%   C-MEX counterpart: mdlOutputs
%%
function Outputs(block)

x=block.InputPort(1).Data;
y=block.InputPort(2).Data;
theta=block.InputPort(3).Data;
v=block.InputPort(4).Data;
w=block.InputPort(5).Data;
global Whi; %
global Woh; %
global Ni; %the number of inputs to the neural network
global No; %the number of outputs of the neural network
global Nh; %the number of neurons in the hidden layer
global eta; % 
global beta;
Whi =[-6.4855 -4.0100 -4.5498;-6.5890 -4.7984 -2.1504;-6.5339 -3.9266 -4.3257;-7.4260 -3.7515 -2.3720;-6.3217 -4.2807 -3.3511;-6.1753 -4.2071 -3.1082;-8.9702 -5.8552 -4.7596;-7.3099 -6.6322 -4.5009;-6.2155 -5.9031 -5.4120;-5.9862 -3.5501 -3.1355;-5.9129 -4.1946 -3.8804;-6.3110 -4.0644 -2.8849;-6.2365 -2.9500 -2.3163;-8.2328 -6.5682 -3.4148;-7.1986 -3.6066 -2.8832;-5.9831 -3.4436 -3.6779;-6.3094 -3.8544 -3.2696;-6.2915 -5.4654 -4.1758;-6.5584 -4.4558 -3.2127;-5.8346 -3.3768 -2.5838];
Woh=[0.0188 -1.9734 0.6255 0.4932 -0.5925 -0.4435 1.0799 0.3705 -0.1926 -1.1629 -0.0350 -0.9390 0.3647 -0.6056 0.5726 -0.1639 -1.3403 -1.5193 -1.6494 -1.8132 -0.2483;-0.0693 1.2123 1.3129 1.6006 0.7402 1.1716 1.5867 3.1500 2.3577 3.3629 2.4780 2.6302 2.9674 0.7661 1.5265 2.0676 1.7297 4.0159 2.3748 2.4173 2.9054;6.3216 3.3388 1.4826 -0.2754 -2.1694 -0.7430 -2.2963 -4.7001 -4.6418 -4.0895 -4.7556 -4.6468 -4.9112 -5.4908 -3.4376 -4.7539 -4.9947 -4.5363 -5.4131 -3.7736 -4.9652];
Ni=2;
No=3;
Nh=10;
eta=0.7;
beta=0.3;
syms x1
fx=1/(1+exp(-x1)); %The sigmoid function
fx1=x1; 
fx_p=fx*(1-fx); 
 


%********************** Forward Propagation *************************%
 I=[1;v;w];
 A=Whi*I;
 syms Outh
 Outh=subs(fx,x1,A);
 Outh=eval(Outh);
 Outh=[1;Outh];
 
 B=Woh*Outh;
 syms Outo
 Outo=subs(fx1,x1,B);
 Outo=eval(Outo);
 
 syms Lo
 Lo=subs(fx_p,x1,Woh*Outh);
 Lo=eval(Lo);
 
 syms Lh
 Lh=subs(fx_p,x1,A);
 Lh=eval(Lh);
 
 Ex=x-Outo(1,1);
 Ex_sq=0.5*Ex*Ex;
 
 Ey=y-Outo(2,1); %l?i cua y
 Ey_sq=0.5*Ey*Ey;

 Eth=theta-Outo(3,1);%l?i cua théta
 Eth_sq=0.5*Eth*Eth;
 
 E_sq=Ex_sq+Ey_sq+Eth_sq;
 Em=[Ex;Ey;Eth];
 
 Dold=0;
 Doldi=0;
 
for k=1:No
for m=1:Nh+1
 DWoh(k,m)=Em(k)*1*Outh(m,1)*eta+beta*Dold;
% DWoh(k,m)=Em(k,Np*(j-1)+i)*Lo(k)*Outh(m,1)*eta+beta*Dold;
 Dold=DWoh(k,m);
 Woh_old(k,m)=Woh(k,m);
 Woh(k,m)=Woh(k,m)+DWoh(k,m);
end
end
for p=1:Nh
for k=1:Ni+1
DWhi(p,k)=eta*Lh(p,1)*I(k,1)*(1*Em(1)*Woh_old(1,p+1)+1*Em(2)*Woh_old(2,p+1)+1*Em(3)*Woh_old(3,p+1))+beta*Doldi;
% DWhi(p,k)=eta*Lh(p,1)*I(k,1)*[Lo(1)*Em(1,Np*(j1)+i)*Woh_old(1,p+1)+Lo(2)*Em(2,Np*(j1)+i)*Woh_old(2,p+1)+Lo(3)*Em(3,Np*(j-1)+i)*Woh_old(3,p+1)]+beta*Doldi;Doldi=DWhi(p,k);
Whi(p,k)=Whi(p,k)+DWhi(p,k);
end
end

block.OutputPort(1).Data=Outo(1,1);
block.OutputPort(2).Data=Outo(2,1);
block.OutputPort(3).Data=Outo(3,1);
block.OutputPort(4).Data=E_sq;
block.OutputPort(5).Data=Whi(1,1);
block.OutputPort(6).Data=Woh(1,1);








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

