%% Covert Tudat .dat ouput files to .mat files
%% Init workspace 
% Variable 'path_tudat_app' is defined by the makefile before running thos script
clearvars -except path_tudat_app

%% Get port and number of satellites
%args = {'CONSTELLATION_N_PLANES', 'CONSTELLATION_N_PER_PLANE','CONSTELLATION_F',...
%    'CONSTELLATION_SMA',...
%    'EPOCH_START','CONSTELLATION_ECC','CONSTELLATION_AOP',...
%    'EPOCH_END', 'EPOCH_SAMPLE', 'EPOCH_CONTROL_UPDATE',...
%    'SAT_Cd','SAT_Ad', 'SAT_MASS', 'SAT_Ct1', 'SAT_ISP', 'SAT_g0',...
%    'SAT_Cr', 'SAT_SRPA'};
%argv = cell(1,length(args));
%fid = fopen('simulationParameters.h');
%while 1
%    tline = fgetl(fid);
%    for i = 1:length(args)
%        len = length(args{i})+8;
%        if length(tline) > len && strcmp(tline(1:len), sprintf('#define %s',args{i}))
%            argv{i} = sscanf(tline,sprintf('#define %s %%d',args{i}));
%        end
%    end
%    if ~ischar(tline), break, end % EOF 
%end
%fclose(fid);
%params = cell2struct(argv,args,2);

%% Set parameters
NSats = length(dir([path_tudat_app,'/output/*.dat']))/2;
if NSats <= 0
	warning("No output files to process.");
	quit;
end


%% Init variables
x = cell(NSats,1);
%% Start converting output
if NSats > 100
    % Start a parallel pool
    %parpool(24);
    %parfor i = 1:NSats
    for i = 1:NSats
        fpath = sprintf(strcat(path_tudat_app,'/output/stateSat%d.dat'),i-1);
        data = dlmread(fpath);
        x{i,1} = data(:,2:end)';  
        fpath = sprintf(strcat(path_tudat_app,'/output/inputSat%d.dat'),i-1);
        data = dlmread(fpath);
        u{i,1} = data(:,2:end)';
        % Tudat's TNW frame is broken! Sencond and third axis are *(-1),
        % i.e., in tudat (X: velocity; Y:  along radius; Z: - angular momentum)
        u{i,1}([2 3],:) = - u{i,1}([2 3],:);
    end
    % Shut down parallel pool
    delete(gcp('nocreate'));
else
        for i = 1:NSats
        fpath = sprintf(strcat(path_tudat_app,'/output/stateSat%d.dat'),i-1);
        data = dlmread(fpath);
        x{i,1} = data(:,2:end)';  
        fpath = sprintf(strcat(path_tudat_app,'/output/inputSat%d.dat'),i-1);
        data = dlmread(fpath);
        u{i,1} = data(:,2:end)';
        % Tudat's TNW frame is broken! Sencond and third axis are *(-1),
        % i.e., in tudat (X: velocity; Y:  along radius; Z: - angular momentum)
        u{i,1}([2 3],:) = - u{i,1}([2 3],:);
    end
end

%% Save cell
save(strcat(path_tudat_app,"/output/output.mat"),'x','u');
clear;
