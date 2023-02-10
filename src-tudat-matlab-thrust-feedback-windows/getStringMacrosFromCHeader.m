%% Package: tudat-matlab-thrust-feedback
% Author: Leonardo Pedroso
%% Function getStringMacrosFromCHeader
% Import int values of macros in a .h C header
% Inputs: 'path': path of C header file
% 		  'arg: cell array of macro strings'
% Outputs: cell array of strings cooresponding to input macros
%% Example:
% In C header file 'path_to_header/header.h':
% "#DEFINE THRUST_FRAME 'TNW_thrust_frame'" 
% The MATLAB command 
% args = {'THRUST_FRAME'};
% argv = getStringMacrosFromCHeader('path_to_header/header.h',args);
% argv{1}
%	'TNW_thrust_frame'
%% Implementation
function argv = getStringMacrosFromCHeader(path,args)
    argv = cell(length(args),1);
    fid = fopen(path);
    while 1
        tline = fgetl(fid);
        for i = 1:length(args)
            len = length(args{i})+8;
            if length(tline) > len && strcmp(tline(1:len), sprintf('#define %s',args{i}))
                argv{i} = sscanf(tline,sprintf('#define %s %%s',args{i}));
            end
            argv{i} = erase(string(argv{i,1}),['"',"'"]);
        end
        if ~ischar(tline), break, end % EOF 
    end
    fclose(fid);
end
