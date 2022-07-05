%% Post process and plot evolution

%% Init 
clear; % Clear workspace variables
% Add source of tudat feedback matlab server class
addpath('../src-tudat-matlab-thrust-feedback');
% Add source of osculating2mean package
% Source code and documentation at https://github.com/decenter2021/osculating2mean
addpath('/mnt/nfs/home/lpedroso/lib/osculating2mean');

%% Define execution options 
% Turn off tudat simulation for debug
tudatSimulation = true;
% Parameter upload from C header
headerParametersFilepath = 'tudat-matlab-parameters.h';

%% Load data
if tudatSimulation
    uploadHeaderParameters = true;
    load('./output/output.mat','x','u');
else
    load('./output/output_matlab.mat','x','u','uploadHeaderParameters');
end

%% Get some simulation parameters
if tudatSimulation && ~uploadHeaderParameters
    error('Parameters must be uploaded from C header file for tudat simulation.');
end
if uploadHeaderParameters
% Define nominal constellation
argv = getFloatMacrosFromCHeader(headerParametersFilepath,...
    {'CONSTELLATION_N_PLANES'; 'CONSTELLATION_N_PER_PLANE';...
    'CONSTELLATION_INC_DEG'; 'CONSTELLATION_F';...
    'CONSTELLATION_SMA';'SAT_Ct1';'EPOCH_CONTROL_UPDATE'});
numberOfPlanes = argv(1);
numberOfSatellitesPerPlane = argv(2);
inclination = argv(3)*pi/180; %(rad)
phasingParameter = argv(4);
walkerParameters = ... % i:T/P/F
    [inclination;... %(i) 
     numberOfPlanes*numberOfSatellitesPerPlane;... % (T) Number of satellites
     numberOfPlanes;... % (P)
     phasingParameter]; % (F)
semiMajorAxis = argv(5); % (m)
Ct1 = argv(6); % (N)
Tctrl = argv(7); % Control cycle time (s)
else
numberOfPlanes = 6;%72;
numberOfSatellitesPerPlane = 5;%22;
inclination = 53.0*pi/180; %(rad)
phasingParameter = 17;
walkerParameters = ... % i:T/P/F
    [inclination;... %(i) 
     numberOfPlanes*numberOfSatellitesPerPlane;... % (T) Number of satellites
     numberOfPlanes;... % (P)
     phasingParameter]; % (F)
semiMajorAxis = 6921000; % (m)
Ct1 = 0.068; % (N)
Tctrl = 10; % Control cycle time (s)
end
% Number of satellites
N = numberOfPlanes*numberOfSatellitesPerPlane;

%% Compute OE and OE error
% Compute evolution of mean OE
OE = cell(N,1);
%parfor i = 1:N
for i = 1:N
    i
    OE{i,1} = zeros(6,size(x{i,1},2));
    for it = 1:size(x{i,1},2)
        %OE{i,1}(:,it) = rv2OEMean(x{i,1}(1:6,it));
        OE{i,1}(:,it) = OEOsc2OEMeanEUK((it-1)*Tctrl,rv2OEOsc(x{i,1}(1:6,it)),12);
    end
end

%% Plot OE evolution of a single satellite
sat = 13;

ItSim = length(OE{sat,1}(1,:));
figure;
hold on;
grid on;
xlabel("$t$ (s)", 'Interpreter','latex');
ylabel("$a$ (m)", 'Interpreter','latex');
plot((0:ItSim-1)*Tctrl,OE{sat,1}(1,:),'LineWidth',3);

figure;
polarplot(OE{sat,1}(2,:),OE{sat,1}(1,:),'LineWidth',3);
title("$a(u)$ (m)", 'Interpreter','latex');
hold on;
p = gca;
p.RLim = [2*min(OE{sat,1}(1,:))-max(OE{sat,1}(1,:)) max(OE{sat,1}(1,:))];

figure;
hold on;
grid on;
xlabel("$t$ (s)", 'Interpreter','latex');
ylabel("$u$ (rad)", 'Interpreter','latex');
plot((0:ItSim-1)*Tctrl,OE{sat,1}(2,:),'LineWidth',3);

figure;
hold on;
grid on;
xlabel("$t$ (s)", 'Interpreter','latex');
ylabel("$e$", 'Interpreter','latex');
plot((0:ItSim-1)*Tctrl,OE{sat,1}(3,:),'LineWidth',3);
plot((0:ItSim-1)*Tctrl,OE{sat,1}(4,:),'LineWidth',3);
legend({'$e_x$','$e_y$'}, 'Interpreter','latex');

figure;
hold on;
grid on;
xlabel("$t$ (s)", 'Interpreter','latex');
ylabel("$i$ (rad)", 'Interpreter','latex');
plot((0:ItSim-1)*Tctrl,OE{sat,1}(5,:),'LineWidth',3);

figure;
hold on;
grid on;
xlabel("$t$ (s)", 'Interpreter','latex');
ylabel("$\Omega$ (rad)", 'Interpreter','latex');
plot((0:ItSim-1)*Tctrl,OE{sat,1}(6,:),'LineWidth',3);

figure;
hold on;
grid on;
xlabel("$t$ (s)", 'Interpreter','latex');
ylabel("$m$ (Kg)", 'Interpreter','latex');
ylim([min(x{sat,1}(7,:))-0.001 max(x{sat,1}(7,:))+0.001])
plot((0:ItSim-1)*Tctrl,x{sat,1}(7,:),'LineWidth',3);

