function info = GetStructInfo(s)
    % Get information about all elements of a structure.
    % 
    % PARAMETER
    % s ... The structure for which to obtain information
    % 
    % RETURN
    % info ... An N-by-1 cell array containing information like signal name, datatype, dimensions, etc.
    assert(isstruct(s), 'Input "s" must be a struct!');
    info = GetStructInfoRecursive(s, '');
end

function info = GetStructInfoRecursive(s, namePrefix)
    strFields = fieldnames(s);
    info = cell.empty();
    for i = 1:numel(strFields)
        % generate default info values
        Name = [namePrefix, strFields{i}];
        Value = s.(strFields{i});
        DataType = class(Value);
        Dimensions = size(Value);
        Min = [];
        Max = [];
        DimensionsMode = 'Fixed';
        Unit = '';
        Description = '';
        Complexity = 'real';
    
        % nested struct: handle recursively
        if(strcmp('struct',char(DataType)))
            nestedInfo = GetStructInfoRecursive(Value, [Name '.']);
            info = [info; nestedInfo];
        else
            % handle special data types and dimensions
            if(strcmp('logical',char(DataType)))
                DataType = 'boolean';
            end
            if(isenum(Value))
                DataType = ['Enum: ', DataType];
            end
            if(~isreal(Value))
                Complexity = 'complex';
            end
            if(isscalar(Value))
                Dimensions = 1;
            end

            % add new info entry
            entry = struct('Name',Name,'Value',Value,'DataType',DataType,'Dimensions',Dimensions,'Min',Min,'Max',Max,'DimensionsMode',DimensionsMode,'Unit',Unit,'Description',Description,'Complexity',Complexity);
            info = [info; entry];
        end
    end
end

