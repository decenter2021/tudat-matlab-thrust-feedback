%% Package: tudat-matlab-thrust-feedback
% Author: Leonardo Pedroso
%% Function getFloatMacrosFromCHeader
% Import float values of macros in a .h C header
% Inputs: 'path': path of C header file
% 		  'arg: cell array of macro strings'
% Outputs: array of values cooresponding to input macros
%% Example:
% In C header file 'path_to_header/header.h':
% '#DEFINE INCLINATION_DEG 53.0' 
% The MATLAB command 
% args = {'INCLINATION_DEG'};
% argv = getFloatMacrosFromCHeader('path_to_header/header.h',args);
% argv 
%	53.00
%% Implementation
function argv = getFloatMacrosFromCHeader(path,args)
    argv = zeros(length(args),1);
    fid = fopen(path);
    while 1
        tline = fgetl(fid);
        for i = 1:length(args)
            len = length(args{i})+8;
            if length(tline) > len && strcmp(tline(1:len), sprintf('#define %s',args{i}))
                argv(i) = sscanf(tline,sprintf('#define %s %%f',args{i}));
            end
        end
        if ~ischar(tline), break, end % EOF 
    end
    fclose(fid);
end
