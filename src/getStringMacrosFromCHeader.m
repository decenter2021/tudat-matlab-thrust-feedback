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
