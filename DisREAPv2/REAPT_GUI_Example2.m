function REAPT_GUI_Example2()
clc
clear all
close all
% Create the figure for the UI
fig = uifigure('Name', 'REAP-T Graphical User Interface', 'Position', [100 100 800 600]);

% Define default matrices
defaultMatrixA = [1 1;0 1];
defaultMatrixB = [0 0.5;1 0.5];
defaultMatrixC = [1 0];
defaultMatrixD =[0 0];
defaultMatrixQx = [1 0 ; 0 1];
defaultMatrixQu = [1 0;0 1];
defaultMatrixQv =    [100];

defaultMatrixUconstraintU=[10;10];
defaultMatrixUconstraintL=[-10;-10];
defaultMatrixX0Str=[  0;0];
defaultMatrixrStr=[4.85];
defaultMatrixXconstraintU=[5;5];
defaultMatrixXconstraintL=[-5;-5];

% Convert matrices to string representations
defaultMatrixAStr = mat2str(defaultMatrixA);
defaultMatrixBStr = mat2str(defaultMatrixB);
defaultMatrixCStr = mat2str(defaultMatrixC);
defaultMatrixDStr = mat2str(defaultMatrixD);
defaultMatrixQxStr = mat2str(defaultMatrixQx);
defaultMatrixQuStr = mat2str(defaultMatrixQu);
defaultMatrixQvStr = mat2str(defaultMatrixQv);
defaultMatrixUconstraintUStr=mat2str(defaultMatrixUconstraintU);
defaultMatrixUconstraintLStr=mat2str(defaultMatrixUconstraintL);
defaultMatrixXconstraintUStr=mat2str(defaultMatrixXconstraintU);
defaultMatrixXconstraintLStr=mat2str(defaultMatrixXconstraintL);
defaultMatrixX0Str=mat2str(defaultMatrixX0Str);
defaultMatrixrStr=mat2str(defaultMatrixrStr);

% First column
uilabel(fig, 'Text', 'Matrix A:', 'Position', [20 550 100 22]);
% uilabel(fig, 'Text', '[0 1;0 1]', 'Position',  [130 545 250 70]);
AInput = uitextarea(fig, 'Position', [130 500 250 70], 'Value', defaultMatrixAStr);

uilabel(fig, 'Text', 'Matrix B:', 'Position', [20 470 100 22]);
BInput = uitextarea(fig, 'Position', [130 420 250 70], 'Value', defaultMatrixBStr);

uilabel(fig, 'Text', 'Matrix C:', 'Position', [20 390 100 22]);
CInput = uitextarea(fig, 'Position', [130 340 250 70], 'Value', defaultMatrixCStr);

uilabel(fig, 'Text', 'Matrix D:', 'Position', [20 310 100 22]);
DInput = uitextarea(fig, 'Position', [130 260 250 70], 'Value', defaultMatrixDStr);

% Dropdown to select system's dynaimcs type (discrete/continuous)
uilabel(fig, 'Text', 'Modeling Framework:', 'Position', [20 230 200 22]);
DynamicsTypeDropdown = uidropdown(fig, ...
    'Items', {'Continuous-Time', 'Discrete-Time'}, ...
    'Position', [140 230 240 22]);

uilabel(fig, 'Text', 'Sampling Period:', 'Position',  [20 200 100 22]);
DeltaTInput = uieditfield(fig, 'numeric', 'Position',  [130 200 250 22], 'Value', 0.2);

uilabel(fig, 'Text', 'X constraint U.B.:', 'Position', [20 170 100 22]);
XConstraintsU = uitextarea(fig, 'Position', [130 170 250 22],'Value', defaultMatrixXconstraintUStr);

uilabel(fig, 'Text', 'X constraint L.B.:', 'Position',  [20 140 100 22]);
XConstraintsL = uitextarea(fig, 'Position',[130 140 250 22],'Value', defaultMatrixXconstraintLStr);

uilabel(fig, 'Text', 'U constraint U.B:', 'Position',  [20 110 100 22]);
UConstraintsU = uitextarea(fig, 'Position', [130 110 250 22],'Value', defaultMatrixUconstraintUStr);

uilabel(fig, 'Text', 'U constraint L.B.:', 'Position',   [20 80 100 22]);
UConstraintsL = uitextarea(fig, 'Position', [130 80 250 22],'Value', defaultMatrixUconstraintLStr);

% Second column
uilabel(fig, 'Text', 'Matrix Qx:', 'Position', [420 550 100 22]);
QxInput = uitextarea(fig, 'Position', [530 500 250 70], 'Value', defaultMatrixQxStr);

uilabel(fig, 'Text', 'Matrix Qu:', 'Position', [420 470 100 22]);
QuInput = uitextarea(fig, 'Position', [530 420 250 70], 'Value', defaultMatrixQuStr);

uilabel(fig, 'Text', 'Prediction Horizon:', 'Position', [420 390 150 22]);
PredictionHorizonInput = uieditfield(fig, 'numeric', 'Position', [530 390 250 22], 'Value', 10);

uilabel(fig, 'Text', '# Time Instants:', 'Position', [420 360 100 22]);
nSimInput = uieditfield(fig, 'numeric', 'Position', [530 360 250 22], 'Value', 100);

uilabel(fig, 'Text', 'Initial Condistion:', 'Position', [420 330 100 22]);
x0 = uitextarea(fig, 'Position',[530 330 250 22], 'Value', defaultMatrixX0Str);

% Dropdown to select desired input type
uilabel(fig, 'Text', 'Desired Target:', 'Position', [420 300 150 22]);
InputTypeDropdown = uidropdown(fig, ...
    'Items', {'r (Reference)', 'xbar (Equilibrium Point)'}, ...
    'Position', [530 300 250 22]);

% Field for xbar (hidden initially)
uilabel(fig, 'Text', 'Equilibrium Point (xbar):', 'Position', [420 270 150 22], 'Visible', 'off', 'Tag', 'xbarLabel');
xbarInput = uitextarea(fig, 'Position', [530 270 250 22], 'Visible', 'off', 'Value', '[0; 0]', 'Tag', 'xbarInput');

uilabel(fig, 'Text', 'Desired:', 'Position',[420 220 100 22]);
Target = uitextarea(fig, 'Position', [530 220 250 22], 'Value', defaultMatrixrStr);

% Dropdown to select mode (Linear TC/Lyapanov TC)
uilabel(fig, 'Text', 'Terminal Constaint', 'Position', [420 190 100 22]);
ModeDropdown = uidropdown(fig, ...
    'Items', {'Prediction-Based', 'Lyapunov-Based'}, ...
    'Position', [530 190 250 22]);

% Add checkboxes for plot activation
uilabel(fig, 'Text', 'Select Plots:', 'Position', [420 240-130 100 22]);
checkbox1 = uicheckbox(fig, 'Text', 'States', 'Position', [640 240-130 100 22]);
checkbox2 = uicheckbox(fig, 'Text', 'Control Inputs', 'Position', [530 240-130 100 22]);
checkbox3 = uicheckbox(fig, 'Text', 'Output', 'Position', [640 210-120 100 22]);
checkbox4 = uicheckbox(fig, 'Text', 'Sigma', 'Position', [530 210-130 100 22]);

% Add a button to trigger the MPC calculation
btn = uibutton(fig, 'Text', 'Run REAP', 'Position', [350 30 100 20], ...
    'ButtonPushedFcn', @(btn, event) runMPCButtonPushed(AInput, BInput, CInput, DInput,XConstraintsU,XConstraintsL,UConstraintsU,UConstraintsL, x0,Target,QxInput, QuInput, DeltaTInput, PredictionHorizonInput,InputTypeDropdown,DynamicsTypeDropdown,ModeDropdown,nSimInput,checkbox1, checkbox2, checkbox3,checkbox4,xbarInput));
end

function runMPCButtonPushed(AInput, BInput, CInput, DInput,XConstraintsU,XConstraintsL,UConstraintsU,UConstraintsL,x0,Target, QxInput, QuInput, DeltaTInput, PredictionHorizonInput,InputTypeDropdown,DynamicsTypeDropdown,ModeDropdown,nSimInput,checkbox1, checkbox2, checkbox3,checkbox4,xbarInput)
% Parse the user inputs

A = str2num(char(AInput.Value)); %#ok<ST2NM>
B = str2num(char(BInput.Value)); %#ok<ST2NM>
C = str2num(char(CInput.Value)); %#ok<ST2NM>
D = str2num(char(DInput.Value)); %#ok<ST2NM>
XConstraints = [str2num(char(XConstraintsL.Value)), str2num(char(XConstraintsU.Value))]; %#ok<ST2NM>
UConstraints = [str2num(char(UConstraintsL.Value)),str2num(char(UConstraintsU.Value))]; %#ok<ST2NM>
AllConstraints.XUB=str2num(char(XConstraintsU.Value));
AllConstraints.XLB=str2num(char(XConstraintsL.Value));
AllConstraints.UUB=str2num(char(UConstraintsU.Value));
AllConstraints.ULB=str2num(char(UConstraintsL.Value));
XConstraints(XConstraints == Inf) = 1000;
XConstraints(XConstraints == -Inf) = -1000;
UConstraints(UConstraints == Inf) = 1000;
UConstraints(UConstraints == -Inf) = -1000;
x0 = str2num(char(x0.Value)); %#ok<ST2NM>
Target = str2num(char(Target.Value)); %#ok<ST2NM>
Qx = str2num(char(QxInput.Value)); %#ok<ST2NM>
Qu = str2num(char(QuInput.Value)); %#ok<ST2NM>
% Qv = str2num(char(QvInput.Value)); %#ok<ST2NM>
Qv=100;
Qv=eye(size(C, 1));
DeltaT = DeltaTInput.Value;
Prediction_Horizon = PredictionHorizonInput.Value;
nSim = nSimInput.Value;
if strcmp(DynamicsTypeDropdown.Value, 'Discrete')
    G = ss(A, B, C, D,DeltaT);
    Gc = d2c(G);
    A = Gc.A;
    B = Gc.B;
    C = Gc.C;
    D = Gc.D;
end

G = ss(A, B, C, D);
Gd = c2d(G, DeltaT);
Ad = Gd.A;
Bd = Gd.B;
Cd = Gd.C;
Dd = Gd.D;
NoS = size(Ad, 1);
NoI = size(Bd, 2);
AllConstraints.NoI=NoI;
AllConstraints.NoS=NoS;
if strcmp(InputTypeDropdown.Value, 'r (Reference)')
    Target = Functions.desiredCalculation(Ad, Bd, Cd, Dd,NoS,NoI,Target); % Call the function to compute r  
end
Functions.validateInputs(A, B, C, D, Qx, Qu, Qv, XConstraints(:,end), XConstraints(:,1), UConstraints(:,end), UConstraints(:,1), x0,Target, nSim, DeltaT,ModeDropdown);

% Compute MPC
if strcmp(ModeDropdown.Value, 'Lyapunov-Based') 
Omegastar=0;
else
Omegastar =  Functions.computeOmegastar(Ad, Bd,Cd,Dd,XConstraints,UConstraints,Target, NoS,NoI, Qx, Qu); % Replace with actual algorithm
end

% Run the MPC
[x, u_app] = Functions.runMPC(A, B, C, D,XConstraints,UConstraints,x0,Target, Qx, Qu, Qv, DeltaT, Prediction_Horizon, Omegastar, nSim,checkbox1, checkbox2, checkbox3,checkbox4,ModeDropdown,AllConstraints);

% Display results in a new figure
resultFig = uifigure('Name', 'MPC Results', 'Position', [600 100 400 600]);
uilabel(resultFig, 'Text', 'State Trajectories (x):', 'Position', [20 550 150 20]);
uitextarea(resultFig, 'Value', mat2str(x), 'Position', [20 300 360 250]);
uilabel(resultFig, 'Text', 'Applied Control Inputs (u_app):', 'Position', [20 270 200 20]);
uitextarea(resultFig, 'Value', mat2str(u_app), 'Position', [20 20 360 250]);
end


