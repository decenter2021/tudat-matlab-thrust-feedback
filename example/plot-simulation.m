%% Post process and plot evolution

%% Init 
clear; % Clear workspace variables
% Add source of tudat feedback matlab server class
addpath('../src-tudat-matlab-thrust-feedback','-frozen');

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
    load('./output/output_matlab_propagation.mat','x','u','uploadHeaderParameters');
end

%% Get some simulation parameters
if tudatSimulation && ~uploadHeaderParameters
    error('Parameters must be uploaded from C header file for tudat simulation.');
end
if uploadHeaderParameters
% Define nominal constellation
argv = getFloatMacrosFromCHeader('constellationParameters.h',...
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
parfor i = 1:N
    i
    OE{i,1} = zeros(6,size(x{i,1},2));
    for it = 1:size(x{i,1},2)
        %OE{i,1}(:,it) = rv2OEMean(x{i,1}(1:6,it));
        OE{i,1}(:,it) = OEOsc2OEMeanEUK((it-1)*Tctrl,rv2OEOsc(x{i,1}(1:6,it)),12);
    end
end
save('OE.mat','OE');
% Compute indicidual errors for each satellite
OEError = cell(N,1); 
for i = 1:N
    OEError{i,1} = zeros(6,size(x{i,1},2));
    OEError{i,1}(1,:) = OE{i,1}(1,:)-semiMajorAxis;
    OEError{i,1}(3,:) = OE{i,1}(3,:);
    OEError{i,1}(4,:) = OE{i,1}(4,:);
    OEError{i,1}(5,:) = OE{i,1}(5,:)-inclination;
end



%% Global performance results 
constellationRigidityError = zeros(N,size(x{i,1},2));
intantaneousAnchor = zeros(2,size(x{i,1},2));
globalError = zeros(5,size(x{i,1},2));
for t = 1:size(x{i,1},2)
    aux = zeros(2,N);
    for i = 1:N
        aux(:,i) = OE{i,1}([2 6],t);
    end
    [constellationRigidityError(:,t,1), constellationRigidityError(:,t,2),intantaneousAnchor(:,t)]=...
        constellationRigidity(aux,walkerParameters);
    for i = 1:N
        % --- Inertial global error ---
        % SMA (m)
        globalError(1,t) = globalError(1,t) + abs(OE{i}(1,t)-semiMajorAxis)/N;
        % Excentricity
        globalError(2,t) = globalError(2,t) + sqrt(OE{i}(3,t)^2+OE{i}(4,t)^2)/N;
        % Inclination (rad)
        globalError(3,t) = globalError(3,t) + abs(OE{i}(5,t)-inclination)/N;        
    end  
    % --- Relative global error ---
    %[globalError(4,t),globalError(5,t)] = constellationRigidity(aux,walkerParameters);
    globalError(4,t) = mean(abs(constellationRigidityError(:,t,1)));
    globalError(5,t) = mean(abs(constellationRigidityError(:,t,2)));
end

% Plot global performance results
ItSim = size(x{1,1},2);

p = figure;
hold on
title('global avg a')
plot((0:ItSim-1)*Tctrl,globalError(1,:));
savefig(p,'a.fig')
hold off

p = figure;
hold on
title('global avg e')
plot((0:ItSim-1)*Tctrl,globalError(2,:));
savefig(p, 'e.fig')
hold off

p = figure;
hold on
title('global avg i')
plot((0:ItSim-1)*Tctrl,globalError(3,:));
savefig(p, 'i.fig')
hold off

p = figure;
hold on
title('global u')
plot((0:ItSim-1)*Tctrl,globalError(4,:));
savefig(p,'u.fig')
hold off

p = figure;
hold on
title('global Omega')
plot((0:ItSim-1)*Tctrl,globalError(5,:));
savefig(p,'Omega.fig')
hold off

writematrix(sum(globalError,2)','performance.txt','Delimiter','tab')

%% Plot input 
sat = 1;

figure;
hold on
title('input')
plot((0:ItSim-2)*Tctrl,u{sat,1}(1,:));
plot((0:ItSim-2)*Tctrl,u{sat,1}(2,:));
plot((0:ItSim-2)*Tctrl,u{sat,1}(3,:));
plot([0 ItSim-2]*Tctrl, [Ct1 Ct1],'--');
plot([0 ItSim-2]*Tctrl, -[Ct1 Ct1],'--');
savefig('input_sat_1')

return;
%% Plot u, Omega error for single sat
sat = 1;

% Plot error in plane RN
figure;
hold on;
plot(constellationRigidityError(sat,:,2),constellationRigidityError(sat,:,1));
xlabel('Omega')
ylabel('u')
axis equal
hold off

figure;
polarplot(intantaneousAnchor(1,:),constellationRigidityError(sat,:,1));
%polarplot(intantaneousAnchor(1,7e3:end),constellationRigidityError(sat,7e3:end,1));
hold on;
polarplot(intantaneousAnchor(1,:),constellationRigidityError(sat,:,2));
%polarplot(intantaneousAnchor(1,7e3:end),constellationRigidityError(sat,7e3:end,2));
legend('u','Omega')


%% Compute initial anchor and relative OE
aux = zeros(2,N);
for i = 1:N
    aux(:,i) = OE{i,1}([2 6],1);
end
[u0,Omega0] = nominalConstellationAnchor(aux,walkerParameters);
t0 = 0;
walkerAnchor = [t0; u0; Omega0];

dalpha = cell(N,1);
for i = 1:N 
    dalpha{i,1} = zeros(6,size(x{i,1},2));
    for it = 1:size(x{i,1},2)
        t_it = (it-1)*Tctrl;
        dalpha{i,1}(:,it) = OEMean2OERel(OE{i,1}(:,it),...
             nominalConstellationOEMean(t_it,i,walkerParameters,semiMajorAxis,walkerAnchor,0));
    end
end

% Plot  relative OE evolution of a single satellite
sat = 1;

figure;
hold on
title('da')
plot((0:ItSim-1)*Tctrl,dalpha{sat,1}(1,:));

figure;
hold on
title('du')
plot((0:ItSim-1)*Tctrl,dalpha{sat,1}(2,:));

figure;
hold on
title('de')
plot((0:ItSim-1)*Tctrl,dalpha{sat,1}(3,:));
plot((0:ItSim-1)*Tctrl,dalpha{sat,1}(4,:));

figure;
hold on
title('di')
plot((0:ItSim-1)*Tctrl,dalpha{sat,1}(5,:));

figure;
hold on
title('dOmega')
plot((0:ItSim-1)*Tctrl,dalpha{sat,1}(6,:));


%% Plot OE evolution of a single satellite
sat = 1;

figure;
hold on
title('a')
plot((0:ItSim-1)*Tctrl,OE{sat,1}(1,:));

figure;
hold on
title('us')
plot((0:ItSim-1)*Tctrl,OE{sat,1}(2,:));

figure;
hold on
title('e')
plot((0:ItSim-1)*Tctrl,OE{sat,1}(3,:));
plot((0:ItSim-1)*Tctrl,OE{sat,1}(4,:));

figure;
hold on
title('i')
plot((0:ItSim-1)*Tctrl,OE{sat,1}(5,:));

figure;
hold on
title('Omega')
plot((0:ItSim-1)*Tctrl,OE{sat,1}(6,:));

figure;
hold on
title('m')
plot((0:ItSim-1)*Tctrl,x{sat,1}(7,:));

%% Plot displacement planes
% Choose sat 
% sat = 1;
% drtn = zeros(3,ItSim);
% for t = 1:size(x{i,1},2)
%     t
%     % Compute instantaneous anchor
%     aux = zeros(2,N);
%     for i = 1:N
%         aux(:,i) = OE{i,1}([2 6],t);
%     end
%     [u_aux, Omega_aux] = nominalConstellationAnchor(aux,walkerParameters);
%     anchor_aux = [(t-1)*Tctrl;u_aux;Omega_aux];
%     % Compute mean orital elements of ideal position
%     aux = nominalConstellationOEMean((t-1)*Tctrl,sat,walkerParameters,semiMajorAxis,anchor_aux,0);
%     % Compute osculating orbital elements of ideal position
%     e_artificial = [0;0;eps;eps;0;0];
%     doe = kaulaGeoPert((t-1)*Tctrl,aux+e_artificial,12);
%     aux([1 2 5 6]) = aux([1 2 5 6]) + doe([1 2 5 6]);
%     % Compute ECI osculating position
%     aux = OEOsculating2rv(aux);
%     % Compute position error
%     dr = x{i,1}(1:3,t)-aux(1:3);
%     % Compute error components in RTN
%     u_r = aux(1:3)/norm(aux(1:3));
%     u_n = cross(aux(1:3),aux(4:6));
%     u_n = u_n/norm(u_n);
%     u_t = cross(u_n,u_r);
%     drtn(1,t) = dr'*u_r;
%     drtn(2,t) = dr'*u_t;
%     drtn(3,t) = dr'*u_n;
% end
% 
% % Plot error in plane RN
% figure;
% hold on;
% plot(drtn(1,:),drtn(3,:));
% xlabel('R')
% ylabel('N')
% 
% 
% return;
% 
% 
