%% build_Exp3_Simulink.m
% Experiment 3 : Mamdani FLC speed control of a DC motor
% Builds Exp3_Mamdani_FLC_DCmotor.slx, runs it, exports all model images and
% report-style graphs to ./Exp3_Simulink/figures, and prints measured vs
% expected (Table 3.3) readings.
%
% Requires: MATLAB R2018b+, Simulink, Fuzzy Logic Toolbox, Control System Toolbox
% Run from an empty working folder:   >> build_Exp3_Simulink
%
% Model contents (three loops on the SAME plant, all Ts = 0.01 s):
%   A) Mamdani FLC : e, de -> Ge, Gde -> Fuzzy Logic Controller -> Gu -> accumulator -> +/-200 V
%   B) PID         : Kp=2, Ki=5, Kd=0.05, +/-200 V (no anti-windup, as in the report run)
%   C) Open loop   : Va = wref / 1.4286
% Reference 100 -> 150 rad/s at 5 s, load 1 N.m at 3 s.

clc; close all;
mdl    = 'Exp3_Mamdani_FLC_DCmotor';
outDir = fullfile(pwd,'Exp3_Simulink');
figDir = fullfile(outDir,'figures');
if ~exist(figDir,'dir'), mkdir(figDir); end
addpath(outDir);

%% 1. Parameters (from the report)
R = 1.0;  L = 0.5;  K = 0.5;  J = 0.01;  b = 0.1;
Vmax = 200;  Ts = 0.01;  Tend = 8;
Ge = 1/60;  Gde = 1/300;  Gu = 8;
Kp = 2;  Ki = 5;  Kd = 0.05;

A = [-b/J  K/J; -K/L  -R/L];         % x = [w; ia]
B = [ 0   -1/J;  1/L   0 ];          % u = [Va; TL]
sysd = c2d(ss(A,B,eye(2),zeros(2,2)),Ts,'zoh');
Ad = sysd.A;  Bd = sysd.B;  Cd = sysd.C;  Dd = sysd.D;   % outputs [w; ia]
Gw = tf(K,[J*L, J*R+b*L, b*R+K^2]);
openLoopGain = 1/dcgain(Gw);         % 0.7

%% 2. Mamdani FIS (7 triangular MFs, 49 rules, min-max, centroid)
lab = {'NB','NM','NS','ZE','PS','PM','PB'};
cen = [-1 -2/3 -1/3 0 1/3 2/3 1];  wd = 1/3;
fis = mamfis('Name','DCmotorFLC','AndMethod','min','OrMethod','max', ...
    'ImplicationMethod','min','AggregationMethod','max', ...
    'DefuzzificationMethod','centroid');
fis = addInput (fis,[-1 1],'Name','e');
fis = addInput (fis,[-1 1],'Name','de');
fis = addOutput(fis,[-1 1],'Name','du');
for i = 1:7
    p = [cen(i)-wd cen(i) cen(i)+wd];
    fis = addMF(fis,'e' ,'trimf',p,'Name',lab{i});
    fis = addMF(fis,'de','trimf',p,'Name',lab{i});
    fis = addMF(fis,'du','trimf',p,'Name',lab{i});
end
rules = zeros(49,5);  k = 1;
for i = 1:7
    for j = 1:7
        rules(k,:) = [i j min(max(i+j-4,1),7) 1 1];   % Table 3.2
        k = k+1;
    end
end
fis = addRule(fis,rules);
try, writeFIS(fis,fullfile(outDir,'DCmotorFLC')); catch, end

% Simulink resolves block parameters in the base workspace
expo = {'R','L','K','J','b','Vmax','Ts','Tend','Ge','Gde','Gu','Kp','Ki','Kd', ...
        'Ad','Bd','Cd','Dd','openLoopGain','fis'};
for i = 1:numel(expo), assignin('base',expo{i},eval(expo{i})); end

%% 3. Create the model
if bdIsLoaded(mdl), close_system(mdl,0); end
slxFile = fullfile(outDir,[mdl '.slx']);
if exist(slxFile,'file'), delete(slxFile); end
new_system(mdl);  open_system(mdl);
set_param(mdl,'Solver','FixedStepDiscrete','FixedStep',num2str(Ts), ...
    'StartTime','0','StopTime',num2str(Tend));

lib.step  = 'simulink/Sources/Step';
lib.gain  = 'simulink/Math Operations/Gain';
lib.sum   = 'simulink/Math Operations/Sum';
lib.sat   = 'simulink/Discontinuities/Saturation';
lib.delay = 'simulink/Discrete/Unit Delay';
lib.dti   = 'simulink/Discrete/Discrete-Time Integrator';
lib.dss   = 'simulink/Discrete/Discrete State-Space';
lib.mux   = 'simulink/Signal Routing/Mux';
lib.demux = 'simulink/Signal Routing/Demux';
lib.goto  = 'simulink/Signal Routing/Goto';
lib.from  = 'simulink/Signal Routing/From';

% ---------- sources ----------
addb(mdl,lib.step,'Speed_Ref_100to150',[30 85 80 115],'Time','5','Before','100','After','150');
addb(mdl,lib.step,'Load_Torque_1Nm',[30 250 80 280],'Time','3','Before','0','After','1');
addb(mdl,lib.goto,'Goto_TL',[110 255 170 275],'GotoTag','TL','ShowName','off');
ln(mdl,'Load_Torque_1Nm/1','Goto_TL/1');

% ---------- A) Mamdani FLC loop ----------
addb(mdl,lib.goto,'Goto_wref',[110 20 170 40],'GotoTag','wref','ShowName','off');
addb(mdl,lib.sum ,'Sum_e',[130 85 155 115],'Inputs','+-');
addb(mdl,lib.from,'From_w_fz',[60 135 120 155],'GotoTag','w_fz','ShowName','off');
addb(mdl,lib.goto,'Goto_e_fz',[200 95 260 115],'GotoTag','e_fz','ShowName','off');
addb(mdl,lib.gain,'Ge',[200 45 250 75],'Gain','Ge');
addb(mdl,lib.sat ,'Sat_e',[275 45 315 75],'UpperLimit','1','LowerLimit','-1');
addb(mdl,lib.delay,'Delay_e',[200 150 245 180],'InitialCondition','0','SampleTime','Ts');
addb(mdl,lib.sum ,'Sum_de',[270 145 295 175],'Inputs','+-');
addb(mdl,lib.gain,'Divide_by_Ts',[320 145 370 175],'Gain','1/Ts');
addb(mdl,lib.gain,'Gde',[390 145 440 175],'Gain','Gde');
addb(mdl,lib.sat ,'Sat_de',[460 145 500 175],'UpperLimit','1','LowerLimit','-1');
addb(mdl,lib.mux ,'Mux_FLC',[530 80 536 140],'Inputs','2');

useFLC = true;
try
    load_system('fuzblock');   % library block name contains line breaks
    addb(mdl,sprintf('fuzblock/Fuzzy Logic \nController'),'Mamdani_FLC',[575 80 665 140]);
    set_param([mdl '/Mamdani_FLC'],'FIS','fis');
catch ME
    warning('Fuzzy Logic Controller block unavailable (%s). Using a 2-D lookup table of the same FIS surface.',ME.message);
    useFLC = false;
    try, delete_block([mdl '/Mamdani_FLC']); catch, end
    g = linspace(-1,1,101);  [EE,DD] = ndgrid(g,g);
    Z = reshape(evalfis(fis,[EE(:) DD(:)]),size(EE));
    addb(mdl,'simulink/Signal Routing/Demux','FLC_Demux',[560 80 566 140],'Outputs','2');
    addb(mdl,'simulink/Lookup Tables/2-D Lookup Table','Mamdani_FLC',[600 80 690 140]);
    setp([mdl '/Mamdani_FLC'],'BreakpointsForDimension1',mat2str(g));
    setp([mdl '/Mamdani_FLC'],'BreakpointsForDimension2',mat2str(g));
    setp([mdl '/Mamdani_FLC'],'Table',mat2str(Z));
end

addb(mdl,lib.gain ,'Gu',[700 95 750 125],'Gain','Gu');
addb(mdl,lib.sum  ,'Sum_u',[780 95 805 125],'Inputs','++');
addb(mdl,lib.sat  ,'Sat_u',[835 95 880 125],'UpperLimit','Vmax','LowerLimit','-Vmax');
addb(mdl,lib.delay,'Delay_u',[790 150 835 180],'InitialCondition','0','SampleTime','Ts');
set_param([mdl '/Delay_u'],'Orientation','left');
addb(mdl,lib.goto ,'Goto_u_fz',[910 30 970 50],'GotoTag','u_fz','ShowName','off');
addb(mdl,lib.mux  ,'Mux_in_fz',[940 85 946 145],'Inputs','2');
addb(mdl,lib.from ,'From_TL_fz',[860 155 920 175],'GotoTag','TL','ShowName','off');
addb(mdl,lib.dss  ,'Motor_fz',[980 80 1080 150],'A','Ad','B','Bd','C','Cd','D','Dd');
setp([mdl '/Motor_fz'],{'SampleTime','Ts'},'Ts');
addb(mdl,lib.demux,'Demux_fz',[1110 80 1116 150],'Outputs','2');
addb(mdl,lib.goto ,'Goto_w_fz',[1150 80 1210 100],'GotoTag','w_fz','ShowName','off');
addb(mdl,lib.goto ,'Goto_ia_fz',[1150 130 1210 150],'GotoTag','ia_fz','ShowName','off');

ln(mdl,'Speed_Ref_100to150/1','Sum_e/1');
ln(mdl,'Speed_Ref_100to150/1','Goto_wref/1');
ln(mdl,'From_w_fz/1','Sum_e/2');
ln(mdl,'Sum_e/1','Ge/1');
ln(mdl,'Sum_e/1','Delay_e/1');
ln(mdl,'Sum_e/1','Sum_de/1');
ln(mdl,'Sum_e/1','Goto_e_fz/1');
ln(mdl,'Delay_e/1','Sum_de/2');
ln(mdl,'Ge/1','Sat_e/1');
ln(mdl,'Sat_e/1','Mux_FLC/1');
ln(mdl,'Sum_de/1','Divide_by_Ts/1');
ln(mdl,'Divide_by_Ts/1','Gde/1');
ln(mdl,'Gde/1','Sat_de/1');
ln(mdl,'Sat_de/1','Mux_FLC/2');
if useFLC
    ln(mdl,'Mux_FLC/1','Mamdani_FLC/1');
else
    ln(mdl,'Mux_FLC/1','FLC_Demux/1');
    ln(mdl,'FLC_Demux/1','Mamdani_FLC/1');
    ln(mdl,'FLC_Demux/2','Mamdani_FLC/2');
end
ln(mdl,'Mamdani_FLC/1','Gu/1');
ln(mdl,'Gu/1','Sum_u/1');
ln(mdl,'Delay_u/1','Sum_u/2');
ln(mdl,'Sum_u/1','Sat_u/1');
ln(mdl,'Sat_u/1','Delay_u/1');          % feedback taken AFTER saturation (anti-windup)
ln(mdl,'Sat_u/1','Goto_u_fz/1');
ln(mdl,'Sat_u/1','Mux_in_fz/1');
ln(mdl,'From_TL_fz/1','Mux_in_fz/2');
ln(mdl,'Mux_in_fz/1','Motor_fz/1');
ln(mdl,'Motor_fz/1','Demux_fz/1');
ln(mdl,'Demux_fz/1','Goto_w_fz/1');
ln(mdl,'Demux_fz/2','Goto_ia_fz/1');

% ---------- B) PID loop ----------
addb(mdl,lib.from ,'From_wref_pid',[30 405 90 425],'GotoTag','wref','ShowName','off');
addb(mdl,lib.sum  ,'Sum_e_pid',[130 405 155 435],'Inputs','+-');
addb(mdl,lib.from ,'From_w_pid',[60 455 120 475],'GotoTag','w_pid','ShowName','off');
addb(mdl,lib.goto ,'Goto_e_pid',[170 520 230 540],'GotoTag','e_pid','ShowName','off');
addb(mdl,lib.gain ,'Kp',[220 335 270 365],'Gain','Kp');
addb(mdl,lib.gain ,'Ki',[220 375 270 405],'Gain','Ki');
addb(mdl,lib.dti  ,'Integrator_pid',[300 375 350 405]);
setp([mdl '/Integrator_pid'],'IntegratorMethod','Integration: Backward Euler');
setp([mdl '/Integrator_pid'],'SampleTime','Ts');
addb(mdl,lib.delay,'Delay_e_pid',[220 460 265 490],'InitialCondition','0','SampleTime','Ts');
addb(mdl,lib.sum  ,'Sum_d_pid',[290 455 315 485],'Inputs','+-');
addb(mdl,lib.gain ,'Kd_over_Ts',[340 455 400 485],'Gain','Kd/Ts');
addb(mdl,lib.sum  ,'Sum_PID',[450 385 475 445],'Inputs','+++');
addb(mdl,lib.sat  ,'Sat_PID',[510 405 555 435],'UpperLimit','Vmax','LowerLimit','-Vmax');
addb(mdl,lib.goto ,'Goto_u_pid',[590 335 650 355],'GotoTag','u_pid','ShowName','off');
addb(mdl,lib.mux  ,'Mux_in_pid',[620 395 626 455],'Inputs','2');
addb(mdl,lib.from ,'From_TL_pid',[540 470 600 490],'GotoTag','TL','ShowName','off');
addb(mdl,lib.dss  ,'Motor_pid',[660 385 760 455],'A','Ad','B','Bd','C','Cd','D','Dd');
setp([mdl '/Motor_pid'],{'SampleTime','Ts'},'Ts');
addb(mdl,lib.demux,'Demux_pid',[790 385 796 455],'Outputs','2');
addb(mdl,lib.goto ,'Goto_w_pid',[830 385 890 405],'GotoTag','w_pid','ShowName','off');
addb(mdl,lib.goto ,'Goto_ia_pid',[830 435 890 455],'GotoTag','ia_pid','ShowName','off');

ln(mdl,'From_wref_pid/1','Sum_e_pid/1');
ln(mdl,'From_w_pid/1','Sum_e_pid/2');
ln(mdl,'Sum_e_pid/1','Kp/1');
ln(mdl,'Sum_e_pid/1','Ki/1');
ln(mdl,'Sum_e_pid/1','Delay_e_pid/1');
ln(mdl,'Sum_e_pid/1','Sum_d_pid/1');
ln(mdl,'Sum_e_pid/1','Goto_e_pid/1');
ln(mdl,'Delay_e_pid/1','Sum_d_pid/2');
ln(mdl,'Sum_d_pid/1','Kd_over_Ts/1');
ln(mdl,'Ki/1','Integrator_pid/1');
ln(mdl,'Kp/1','Sum_PID/1');
ln(mdl,'Integrator_pid/1','Sum_PID/2');
ln(mdl,'Kd_over_Ts/1','Sum_PID/3');
ln(mdl,'Sum_PID/1','Sat_PID/1');
ln(mdl,'Sat_PID/1','Goto_u_pid/1');
ln(mdl,'Sat_PID/1','Mux_in_pid/1');
ln(mdl,'From_TL_pid/1','Mux_in_pid/2');
ln(mdl,'Mux_in_pid/1','Motor_pid/1');
ln(mdl,'Motor_pid/1','Demux_pid/1');
ln(mdl,'Demux_pid/1','Goto_w_pid/1');
ln(mdl,'Demux_pid/2','Goto_ia_pid/1');

% ---------- C) Open loop ----------
addb(mdl,lib.from ,'From_wref_open',[30 690 90 710],'GotoTag','wref','ShowName','off');
addb(mdl,lib.gain ,'Open_Loop_Gain',[120 685 200 715],'Gain','openLoopGain');
addb(mdl,lib.goto ,'Goto_u_open',[230 630 290 650],'GotoTag','u_open','ShowName','off');
addb(mdl,lib.mux  ,'Mux_in_open',[260 690 266 750],'Inputs','2');
addb(mdl,lib.from ,'From_TL_open',[170 735 230 755],'GotoTag','TL','ShowName','off');
addb(mdl,lib.dss  ,'Motor_open',[300 675 400 745],'A','Ad','B','Bd','C','Cd','D','Dd');
setp([mdl '/Motor_open'],{'SampleTime','Ts'},'Ts');
addb(mdl,lib.demux,'Demux_open',[430 675 436 745],'Outputs','2');
addb(mdl,lib.goto ,'Goto_w_open',[470 675 530 695],'GotoTag','w_open','ShowName','off');
addb(mdl,lib.goto ,'Goto_ia_open',[470 725 530 745],'GotoTag','ia_open','ShowName','off');

ln(mdl,'From_wref_open/1','Open_Loop_Gain/1');
ln(mdl,'Open_Loop_Gain/1','Goto_u_open/1');
ln(mdl,'Open_Loop_Gain/1','Mux_in_open/1');
ln(mdl,'From_TL_open/1','Mux_in_open/2');
ln(mdl,'Mux_in_open/1','Motor_open/1');
ln(mdl,'Motor_open/1','Demux_open/1');
ln(mdl,'Demux_open/1','Goto_w_open/1');
ln(mdl,'Demux_open/2','Goto_ia_open/1');

% ---------- logging (To Workspace, timeseries) ----------
addLogger(mdl,lib,{'w_fz','ia_fz','u_fz','e_fz'},      'Mux_log_fz',  'log_fz',  1260,  20,Ts);
addLogger(mdl,lib,{'w_pid','ia_pid','u_pid','e_pid'},  'Mux_log_pid', 'log_pid', 1260, 340,Ts);
addLogger(mdl,lib,{'w_open','ia_open','u_open','wref'},'Mux_log_open','log_open',1260, 640,Ts);

% ---------- scopes ----------
addScope(mdl,lib,'Scope_Speed',          {'wref','w_open','w_pid','w_fz'},1560,  20);
addScope(mdl,lib,'Scope_Error',          {'e_pid','e_fz'},                1560, 200);
addScope(mdl,lib,'Scope_ArmatureVoltage',{'u_pid','u_fz'},                1560, 340);
addScope(mdl,lib,'Scope_ArmatureCurrent',{'ia_pid','ia_fz'},              1560, 480);

note(mdl,'A) Mamdani FLC loop (incremental output, +/-200 V)',[30 -10]);
note(mdl,'B) PID comparison loop (Kp=2, Ki=5, Kd=0.05, +/-200 V)',[30 315]);
note(mdl,'C) Open loop (Va = wref/1.4286)',[30 610]);
note(mdl,'Logged signals [w ia Va e] -> log_fz / log_pid / log_open',[1260 -10]);

save_system(mdl,slxFile);
fprintf('Created %s\n',slxFile);

%% 4. Export model diagram
try
    set_param(mdl,'ZoomFactor','FitSystem');
    set_param(mdl,'PaperPositionMode','auto');
    print(['-s' mdl],'-dpng','-r300',fullfile(figDir,'model_top_level.png'));
    % close-up of the Mamdani FLC loop (top-left region of the fixed layout)
    img = imread(fullfile(figDir,'model_top_level.png'));
    [hI,wI,~] = size(img);
    imwrite(img(1:round(0.39*hI),1:round(0.70*wI),:),fullfile(figDir,'model_fuzzy_loop_closeup.png'));
catch ME
    warning('Model image export failed: %s',ME.message);
end

%% 5. Simulate
simOut = sim(mdl,'ReturnWorkspaceOutputs','on');
Sf = simOut.get('log_fz');   Sp = simOut.get('log_pid');   So = simOut.get('log_open');
t  = Sf.Time;
Wf = Sf.Data(:,1); Iaf = Sf.Data(:,2); Uf = Sf.Data(:,3); Ef = Sf.Data(:,4);
Wp = Sp.Data(:,1); Iap = Sp.Data(:,2); Up = Sp.Data(:,3); Ep = Sp.Data(:,4);
Wo = So.Data(:,1); Iao = So.Data(:,2); Uo = So.Data(:,3); wref = So.Data(:,4);
Eo = wref - Wo;

%% 6. Report graphs (styled as in the report)
cF = [0 0.447 0.741];  cP = [0.85 0.15 0.15];  cO = [0.17 0.63 0.17];
xl = @(a) set(a,'XLim',[0 Tend]);

f = figure('Color','w','Position',[100 100 700 720]);
subplot(3,1,1); plotmf(fis,'input',1);  title('Input 1: speed error e (normalised)');   grid on;
subplot(3,1,2); plotmf(fis,'input',2);  title('Input 2: change in error de (normalised)'); grid on;
subplot(3,1,3); plotmf(fis,'output',1); title('Output: change in control voltage du (normalised)'); grid on;
saveFig(f,fullfile(figDir,'fig3_2_membership_functions.png'));

f = figure('Color','w','Position',[100 100 700 520]);
gensurf(fis); title('Fuzzy control surface (Mamdani, centroid)'); view(-135,28);
saveFig(f,fullfile(figDir,'fig3_3_control_surface.png'));

f = figure('Color','w','Position',[100 100 900 500]); hold on; grid on;
plot(t,wref,'k--','LineWidth',1.2);
plot(t,Wo,'Color',cO,'LineWidth',1.2);
plot(t,Wp,'Color',cP,'LineWidth',1.4);
plot(t,Wf,'Color',cF,'LineWidth',1.8);
xline(3,':','Load torque 1 N\cdotm','LabelVerticalAlignment','bottom');
xline(5,':','Set-point 100\rightarrow150 rad/s','LabelVerticalAlignment','bottom');
xlabel('Time (s)'); ylabel('Speed \omega (rad/s)'); ylim([0 170]); xl(gca);
title('Speed response of DC motor');
legend('Reference','Open loop','Conventional PID','Mamdani FLC','Location','southeast');
saveFig(f,fullfile(figDir,'fig3_4_speed_response.png'));

f = figure('Color','w','Position',[100 100 900 420]); hold on; grid on;
plot(t,Ep,'Color',cP,'LineWidth',1.3); plot(t,Ef,'Color',cF,'LineWidth',1.6);
xline(3,':'); xline(5,':'); ylim([-15 110]); xl(gca);
xlabel('Time (s)'); ylabel('Error e (rad/s)'); title('Speed error'); legend('PID','Mamdani FLC');
saveFig(f,fullfile(figDir,'fig3_5_error.png'));

f = figure('Color','w','Position',[100 100 900 620]);
subplot(2,1,1); hold on; grid on;
plot(t,Up,'Color',cP,'LineWidth',1.3); plot(t,Uf,'Color',cF,'LineWidth',1.6); xline(3,':'); xline(5,':');
ylabel('Armature voltage V_a (V)'); title('Controller output (armature voltage)'); legend('PID','Mamdani FLC'); ylim([0 210]); xl(gca);
subplot(2,1,2); hold on; grid on;
plot(t,Iap,'Color',cP,'LineWidth',1.3); plot(t,Iaf,'Color',cF,'LineWidth',1.6); xline(3,':'); xline(5,':');
ylabel('Armature current i_a (A)'); xlabel('Time (s)'); title('Armature current'); legend('PID','Mamdani FLC'); xl(gca);
saveFig(f,fullfile(figDir,'fig3_6_voltage_current.png'));

f = figure('Color','w','Position',[100 100 1000 420]);
subplot(1,2,1); hold on; grid on;
plot(t,wref,'k--'); plot(t,Wp,'Color',cP); plot(t,Wf,'Color',cF,'LineWidth',1.6);
xlim([2.8 4.2]); ylim([95 104]); title('Load disturbance rejection (t = 3 s)'); xlabel('Time (s)'); ylabel('Speed \omega (rad/s)'); legend('Ref','PID','FLC');
subplot(1,2,2); hold on; grid on;
plot(t,wref,'k--'); plot(t,Wp,'Color',cP); plot(t,Wf,'Color',cF,'LineWidth',1.6);
xlim([4.8 6.5]); ylim([95 160]); title('Set-point tracking (t = 5 s)'); xlabel('Time (s)'); legend('Ref','PID','FLC');
saveFig(f,fullfile(figDir,'fig3_7_zoomed_views.png'));

%% 7. Readings: measured (this model) vs expected (report Table 3.3)
sm = @(w) stepMetrics(t,w,0,3,0,100);           % 0 -> 100 rad/s, no load
sp = @(w) stepMetrics(t,w,5,Tend+Ts/2,100,150);   % 100 -> 150 rad/s
dip = @(w) 100 - min(w(t>=3 & t<5));
sse = @(w) abs(100 - w(find(t<5,1,'last')));
Wm = {Wf,Wp,Wo};  Um = {Uf,Up,Uo};  Im = {Iaf,Iap,Iao};  Em = {Ef,Ep,Eo};
names = {'Mamdani FLC','PID','Open loop'};
%            tr    ts    os    dip   sse   ts_sp os_sp  Va    Ia    IAE
expct = [0.50 0.97 0.00 3.94 0.00 0.87 0.31 118  32.3 44.5;    % FLC
         0.12 1.05 5.02 3.77 0.00 1.07 4.53 200  35.6 22.1;    % PID
         0.26 0.71 3.95 6.58 2.86 NaN  NaN  105  32.2 43.4];   % open loop
tol   = [0.03 0.05 0.50 0.10 0.10 0.05 0.15 1.5  0.3  0.5];
hdr   = {'Rise 0-100 (s)','Settle 0-100 (s)','Overshoot 0-100 (%)','Load dip (rad/s)', ...
         'SS error under load','Settle 100-150 (s)','Overshoot 100-150 (%)', ...
         'Peak Va (V)','Peak ia (A)','IAE 0-8 s'};
fprintf('\n===== Measured (this Simulink model) vs expected (Table 3.3) =====\n');
for c = 1:3
    a = sm(Wm{c});  s2 = sp(Wm{c});
    meas = [a.tr a.ts a.os dip(Wm{c}) sse(Wm{c}) s2.ts s2.os max(Um{c}) max(Im{c}) trapz(t,abs(Em{c}))];
    fprintf('\n%s\n',names{c});
    for m = 1:numel(meas)
        if isnan(expct(c,m)), continue; end
        ok = abs(meas(m)-expct(c,m)) <= tol(m);
        fprintf('  %-24s measured %8.3f | expected %8.3f | %s\n',hdr{m},meas(m),expct(c,m),ternary(ok,'OK','CHECK'));
    end
end
fprintf('\nFiles written to %s\n',outDir);

%% ---------------- local functions ----------------
function addb(mdl,lib,name,pos,varargin)
    add_block(lib,[mdl '/' name],'Position',pos,varargin{:});
end
function ln(mdl,src,dst)
    add_line(mdl,src,dst,'autorouting','smart');
end
function setp(blk,names,val)      % set a dialog parameter only if it exists
    if ischar(names), names = {names}; end
    dp = get_param(blk,'DialogParameters');
    for i = 1:numel(names)
        if isstruct(dp) && isfield(dp,names{i}), set_param(blk,names{i},val); return; end
    end
    warning('%s: none of {%s} is a dialog parameter.',blk,strjoin(names,','));
end
function addLogger(mdl,lib,tags,muxName,wsName,x,y,Ts) %#ok<INUSD>
    n = numel(tags);
    addb(mdl,lib.mux,muxName,[x+90 y x+96 y+35*n],'Inputs',num2str(n));
    for i = 1:n
        fn = sprintf('%s_from%d',wsName,i);
        addb(mdl,lib.from,fn,[x y+35*(i-1) x+60 y+35*(i-1)+20],'GotoTag',tags{i},'ShowName','off');
        ln(mdl,[fn '/1'],[muxName '/' num2str(i)]);
    end
    addb(mdl,'simulink/Sinks/To Workspace',wsName, ...
        [x+140 y+round(35*n/2)-15 x+230 y+round(35*n/2)+15], ...
        'VariableName',wsName,'SaveFormat','Timeseries');
    setp([mdl '/' wsName],'SampleTime','Ts');
    ln(mdl,[muxName '/1'],[wsName '/1']);
end
function addScope(mdl,lib,name,tags,x,y)
    n = numel(tags);
    addb(mdl,'simulink/Sinks/Scope',name,[x+110 y x+150 y+35*n]);
    try, set_param([mdl '/' name],'NumInputPorts',num2str(n)); catch, end
    for i = 1:n
        fn = sprintf('%s_from%d',name,i);
        addb(mdl,lib.from,fn,[x y+35*(i-1) x+60 y+35*(i-1)+20],'GotoTag',tags{i},'ShowName','off');
        ln(mdl,[fn '/1'],[name '/' num2str(i)]);
    end
end
function note(mdl,txt,pos)
    try
        a = Simulink.Annotation(mdl,txt);  a.Position = pos;  a.FontSize = 14;  a.FontWeight = 'bold';
    catch
    end
end
function saveFig(fig,file)
    try, exportgraphics(fig,file,'Resolution',300);
    catch, print(fig,file,'-dpng','-r300'); end
end
function m = stepMetrics(t,w,t0,t1,r0,r1)
    idx = t>=t0 & t<t1;  tt = t(idx)-t0;  ww = w(idx);  d = r1-r0;
    i10 = find(ww >= r0+0.1*d,1);  i90 = find(ww >= r0+0.9*d,1);
    m.tr = tt(i90)-tt(i10);
    m.os = max(0,(max(ww)-r1)/d*100);
    out  = find(abs(ww-r1) > 0.02*abs(d));
    if isempty(out), m.ts = 0; else, m.ts = tt(min(out(end)+1,numel(tt))); end
end
function s = ternary(c,a,b)
    if c, s = a; else, s = b; end
end
