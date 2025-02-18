function StructToSubsystems(s, signalName, modelNamePrefix, outputDirectory)
    % Generate subsystem reference models for Simulink to use a MATLAB structure as a bus signal and implement serializers for
    % that bus. This function generates the subsystem reference models with the following meaning:
    % 
    % 1) Default Bus
    %    Has one output port that contains the bus definition. All values can be assigned afterwards using a Bus Assignment
    %    block. Use this subsystem to create the bus for later serialization.
    % 
    % 2) Pack Bus
    %    Serialize the bus into a set of bytes using the machine byte-order.
    % 
    % 3) Unpack Bus
    %    Deserialize binary data (bytes) to reconstruct the bus. The subsystem outputs a boolean success signal, that is TRUE if
    %    the binary data has been unpacked successfully (e.g. length matches).
    % 
    % PARAMETERS
    % s               ... [struct] The structure to be used for bus creation and serialization. The signal specification must
    %                     br supported by Simulink (e.g. built-in datatypes, signal dimension < 3).
    % signalName      ... [string] The signal name to be appear as input and output ports. Make sure that this parameter is
    %                     different from common values like "bytes", "length", "success", "Constant", etc.
    % modelNamePrefix ... [string] A prefix to be used for the model and file name generation. All the subsystems are named
    %                     according to the rule "[PREFIX]DefaultBus", "[PREFIX]PackBus", "[PREFIX]UnpackBus" with [PREFIX] being
    %                     the specified prefix string. If this parameter is not given, an empty string is used as prefix.
    % outputDirectory ... [string] Output directory, where to store all generated subsystem reference model files. If this
    %                     parameter is not given, then the current folder is returned (pwd).
    if(nargin < 4)
        outputDirectory = pwd();
    end
    if(nargin < 3)
        modelNamePrefix = '';
    end
    assert(isstruct(s) && ~isempty(s), 'Input "s" must be a non-empty struct!');
    assert(ischar(signalName), 'Input "signalName" must be a character vector!');
    assert(ischar(modelNamePrefix), 'Input "modelNamePrefix" must be a character vector!');
    assert(ischar(outputDirectory), 'Input "outputDirectory" must be a character vector!');

    % generate modelnames and corresponding filenames
    modelNameDefaultBus = [modelNamePrefix,'DefaultBus'];
    modelNamePackBus    = [modelNamePrefix,'PackBus'];
    modelNameUnpackBus  = [modelNamePrefix,'UnpackBus'];
    fileNameDefaultBus  = fullfile(outputDirectory,[modelNameDefaultBus, '.slx']);
    fileNamePackBus     = fullfile(outputDirectory,[modelNamePackBus, '.slx']);
    fileNameUnpackBus   = fullfile(outputDirectory,[modelNameUnpackBus, '.slx']);

    % get structural information like name, value, datatype, dimensions, etc.
    structInfo = BusSerializer.GetStructInfo(s);

    % make sure, that output directory exists
    [~,~] = mkdir(outputDirectory);

    % generate subsystem reference model: default bus
    mDefaultBus = NewModel(modelNameDefaultBus, fileNameDefaultBus);
    GenerateDefaultBusModel(modelNameDefaultBus, signalName, structInfo);
    SaveModel(mDefaultBus, fileNameDefaultBus);

    % generate subsystem reference model: pack bus
    mPackBus = NewModel(modelNamePackBus, fileNamePackBus);
    GeneratePackBusModel(modelNamePackBus, signalName, structInfo);
    SaveModel(mPackBus, fileNamePackBus);

    % generate subsystem reference model: unpack bus
    mUnpackBus = NewModel(modelNameUnpackBus, fileNameUnpackBus);
    GenerateUnpackBusModel(modelNameUnpackBus, signalName, structInfo, modelNameDefaultBus);
    SaveModel(mUnpackBus, fileNameUnpackBus);
end

function model = NewModel(modelName, fileName)
    close_system(modelName,0);
    if(exist(fileName,'file'))
        delete(fileName);
    end
    model = new_system(modelName,"Subsystem");
    model = load_system(model);
end

function SaveModel(model, fileName)
    save_system(model,fileName);
    close_system(model);
end

function numBytes = GetNumberOfBytes(structInfo)
    numBytes = uint32(0);
    for i = 1:numel(structInfo)
        switch(structInfo{i}.DataType)
            case {'double','int64','uint64'}
                L = uint32(8);
            case {'single','int32','uint32'}
                L = uint32(4);
            case {'int16','uint16'}
                L = uint32(2);
            case {'int8','uint8','logical','bool','boolean'}
                L = uint32(1);
            otherwise
                error(['Data type "' structInfo{i}.DataType '" is not supported for element selection!']);
        end
        numBytes = numBytes + uint32(prod(structInfo{i}.Dimensions))*L;
    end
end

function GenerateDefaultBusModel(modelName, signalName, structInfo)
    for i = 1:numel(structInfo)
        % convert signal specification to strings
        strDataType = structInfo{i}.DataType;
        strName = structInfo{i}.Name;
        strMin = mat2str(structInfo{i}.Min);
        strMax = mat2str(structInfo{i}.Max);
        strUnit = structInfo{i}.Unit;
        if(isempty(strUnit))
            strUnit = 'inherit';
        end
        strDimensions = mat2str(structInfo{i}.Dimensions);
        strSignalType = structInfo{i}.Complexity;
        if(numel(size(structInfo{i}.Value)) > 2)
            strValue = ['zeros(' mat2str((size(structInfo{i}.Value))) ')'];
        else
            strValue = mat2str(structInfo{i}.Value);
        end

        % y-position for block offset
        y = (i - 1) * 30;
        
        % constant
        h_constant = add_block('simulink/Sources/Constant', [modelName '/Contant'], 'MakeNameUnique', 'on', 'Position', [0 y 80 y+22], 'ShowName', 'off');
        set_param(h_constant, 'Value', strValue);
        set_param(h_constant, 'OutDataTypeStr', strDataType);
        
        % signal spec
        h_signalspec = add_block('simulink/Signal Attributes/Signal Specification', [modelName '/SignalSpecification'], 'MakeNameUnique', 'on', 'Position', [180 y 480 y+22], 'ShowName', 'off');
        set_param(h_signalspec, 'OutDataTypeStr', strDataType);
        set_param(h_signalspec, 'OutMin', strMin);
        set_param(h_signalspec, 'OutMax', strMax);
        set_param(h_signalspec, 'Unit', strUnit);
        set_param(h_signalspec, 'Dimensions', strDimensions);
        set_param(h_signalspec, 'VarSizeSig', 'No');
        set_param(h_signalspec, 'SignalType', strSignalType);
        
        % bus out
        if(1 == i)
            h_busout = add_block('simulink/Signal Routing/Bus Element Out', [modelName '/BusElementOut'], 'Position', [580 y+5 590 y+15], 'ShowName', 'off');
            set_param(h_busout, 'PortName', signalName);
        else
            h_busout = add_block([modelName '/BusElementOut'], [modelName '/BusElementOut'], 'CopyOption', 'duplicate', 'MakeNameUnique', 'on', 'Position', [580 y+5 590 y+15], 'ShowName', 'off');
        end
        set_param(h_busout, 'Element', strName);
        
        % lines
        add_line(modelName, get_param(h_constant,'PortHandles').Outport(1), get_param(h_signalspec,'PortHandles').Inport(1));
        add_line(modelName, get_param(h_signalspec,'PortHandles').Outport(1), get_param(h_busout,'PortHandles').Inport(1));
    end
end

function GeneratePackBusModel(modelName, signalName, structInfo)
    % byte pack block
    strDataTypes = ['{''' strjoin(cellfun(@(x)(x.DataType),structInfo,'UniformOutput',false),''',''') '''}'];
    h_bytepack = add_block('embeddedtargetslib/Host Communication/Byte Pack', [modelName '/BytePack'], 'MakeNameUnique', 'on', 'ShowName', 'off');
    set_param(h_bytepack, 'Position', [100 100 160 100+30*numel(structInfo)]);
    set_param([modelName '/BytePack'], 'DataTypes', strDataTypes);

    % output blocks, relative to byte packing
    portPositionBytePack = get_param(get_param(h_bytepack,'PortHandles').Outport(1), 'Position');
    h_outbytes = add_block('simulink/Sinks/Out1', [modelName '/bytes']);
    set_param(h_outbytes, 'OutDataTypeStr', 'uint8');
    h_outlength = add_block('simulink/Sinks/Out1', [modelName '/length']);
    set_param(h_outlength, 'OutDataTypeStr', 'uint32');
    set_param(h_outlength, 'PortDimensions', '1');
    h_width = add_block('simulink/Signal Attributes/Width', [modelName '/Width'], 'ShowName', 'off');
    set_param(h_width, 'DataType', 'uint32');
    set_param(h_outbytes, 'Position', [portPositionBytePack(1)+200, portPositionBytePack(2)-7, portPositionBytePack(1)+230, portPositionBytePack(2)+7]);
    set_param(h_outlength, 'Position', [portPositionBytePack(1)+200, portPositionBytePack(2)+50, portPositionBytePack(1)+230, portPositionBytePack(2)+64]);
    set_param(h_width, 'Position', [portPositionBytePack(1)+85, portPositionBytePack(2)+40, portPositionBytePack(1)+115, portPositionBytePack(2)+70]);
    add_line(modelName, get_param(h_bytepack,'PortHandles').Outport(1), get_param(h_width,'PortHandles').Inport(1), 'autorouting', 'smart');
    add_line(modelName, get_param(h_bytepack,'PortHandles').Outport(1), get_param(h_outbytes,'PortHandles').Inport(1), 'autorouting', 'smart');
    add_line(modelName, get_param(h_width,'PortHandles').Outport(1), get_param(h_outlength,'PortHandles').Inport(1), 'autorouting', 'smart');

    % bus element in
    for i = 1:numel(structInfo)
        portPosition = get_param(get_param(h_bytepack,'PortHandles').Inport(i),'Position');
        positionBusIn = [portPosition(1)-100 portPosition(2)-5 portPosition(1)-90 portPosition(2)+5];
        if(1 == i)
            h_busin = add_block('simulink/Signal Routing/Bus Element In', [modelName '/BusElementIn'], 'Position', positionBusIn, 'ShowName', 'off');
            set_param(h_busin, 'PortName', signalName);
        else
            h_busin = add_block([modelName '/BusElementIn'], [modelName '/BusElementIn'], 'CopyOption', 'duplicate', 'MakeNameUnique', 'on', 'Position', positionBusIn, 'ShowName', 'off');
        end
        set_param(h_busin, 'Element', structInfo{i}.Name);
        add_line(modelName, get_param(h_busin,'PortHandles').Outport(1), get_param(h_bytepack,'PortHandles').Inport(i));
    end
end

function GenerateUnpackBusModel(modelName, signalName, structInfo, referenceModelName)
    numBytes = GetNumberOfBytes(structInfo);
    strDimensions = ['{' strjoin(cellfun(@(x)(mat2str(x.Dimensions)),structInfo,'UniformOutput',false),',') '}'];
    strDataTypes = ['{''' strjoin(cellfun(@(x)(x.DataType),structInfo,'UniformOutput',false),''',''') '''}'];
    strAssigned = strjoin(cellfun(@(x)(x.Name),structInfo,'UniformOutput',false),',');

    % blocks at root level
    subSysName = [modelName '/unpack'];
    h_inbytes = add_block('simulink/Sources/In1', [modelName '/bytes'], 'OutDataTypeStr', 'uint8', 'Position', [0 113 30 127]);
    h_inlength = add_block('simulink/Sources/In1', [modelName '/length'], 'OutDataTypeStr', 'uint32', 'PortDimensions', '1', 'Position', [0 13 30 27]);
    h_compare = add_block('simulink/Logic and Bit Operations/Compare To Constant', [modelName '/CompareToConstant'], 'ShowName', 'off', 'relop', '==', 'const', num2str(numBytes), 'Position', [85 10 175 30]);
    h_width = add_block('simulink/Signal Attributes/Width', [modelName '/Width'], 'ShowName', 'off', 'DataType', 'uint32', 'Position', [85 55 115 85]);
    h_relop = add_block('simulink/Logic and Bit Operations/Relational Operator', [modelName '/RelationalOperator'], 'ShowName', 'off', 'relop', '<=', 'Position', [150 32 175 83]);
    h_and = add_block('simulink/Logic and Bit Operations/Logical Operator', [modelName '/AND'], 'ShowName', 'off', 'Inputs', '2', 'Position', [245 1 270 79]);
    h_outsuccess = add_block('simulink/Sinks/Out1', [modelName '/success'], 'OutDataTypeStr', 'boolean', 'PortDimensions', '1', 'Position', [475 33 505 47]);
    h_outbus = add_block('simulink/Sinks/Out1', [modelName '/' signalName], 'Position', [475 113 505 127]);
    h_enabled = add_block('simulink/Ports & Subsystems/Enabled Subsystem', subSysName, 'Position', [300 87 415 153]);
    
    add_line(modelName, get_param(h_inlength,'PortHandles').Outport(1), get_param(h_relop,'PortHandles').Inport(1), 'autorouting', 'smart');
    add_line(modelName, get_param(h_inlength,'PortHandles').Outport(1), get_param(h_compare,'PortHandles').Inport(1), 'autorouting', 'smart');
    add_line(modelName, get_param(h_inbytes,'PortHandles').Outport(1), get_param(h_width,'PortHandles').Inport(1), 'autorouting', 'smart');
    add_line(modelName, get_param(h_inbytes,'PortHandles').Outport(1), get_param(h_enabled,'PortHandles').Inport(1), 'autorouting', 'smart');
    add_line(modelName, get_param(h_width,'PortHandles').Outport(1), get_param(h_relop,'PortHandles').Inport(2), 'autorouting', 'smart');
    add_line(modelName, get_param(h_compare,'PortHandles').Outport(1), get_param(h_and,'PortHandles').Inport(1), 'autorouting', 'smart');
    add_line(modelName, get_param(h_relop,'PortHandles').Outport(1), get_param(h_and,'PortHandles').Inport(2), 'autorouting', 'smart');
    add_line(modelName, get_param(h_and,'PortHandles').Outport(1), get_param(h_enabled,'PortHandles').Enable(1), 'autorouting', 'smart');
    add_line(modelName, get_param(h_and,'PortHandles').Outport(1), get_param(h_outsuccess,'PortHandles').Inport(1), 'autorouting', 'smart');
    add_line(modelName, get_param(h_enabled,'PortHandles').Outport(1), get_param(h_outbus,'PortHandles').Inport(1), 'autorouting', 'smart');
    
    % blocks in enabled subsystem
    delete_line(get_param([subSysName '/In1'],'LineHandles').Outport(1));
    h_bytepack = add_block('embeddedtargetslib/Host Communication/Byte Unpack', [subSysName '/ByteUnack'], 'MakeNameUnique', 'on', 'ShowName', 'off', 'Dimensions', strDimensions, 'DataTypes', strDataTypes, 'Position', [290 100 350 100+30*numel(structInfo)]);
    portPositionByteUnpack = get_param(get_param(h_bytepack,'PortHandles').Inport(1),'Position');
    h_selector = add_block('simulink/Signal Routing/Selector', [subSysName '/Selector'], 'ShowName', 'off', 'InputPortWidth', '-1', 'Indices', ['1:' num2str(numBytes)], 'Position', [portPositionByteUnpack(1)-120 portPositionByteUnpack(2)-20 portPositionByteUnpack(1)-80 portPositionByteUnpack(2)+20]);
    set_param([subSysName '/In1'], 'Position', [portPositionByteUnpack(1)-235 portPositionByteUnpack(2)-7 portPositionByteUnpack(1)-205 portPositionByteUnpack(2)+7]);
    set_param([subSysName '/Enable'], 'Position', [portPositionByteUnpack(1)-230 portPositionByteUnpack(2)-70 portPositionByteUnpack(1)-210 portPositionByteUnpack(2)-50]);
    posByteUnpack = get_param(h_bytepack, 'Position');
    h_subref = add_block('simulink/Ports & Subsystems/Subsystem Reference', [subSysName '/SubsystemReference'], 'ShowName', 'off', 'ReferencedSubsystem', referenceModelName, 'Position', [posByteUnpack(1) posByteUnpack(2)-25 posByteUnpack(3) posByteUnpack(2)-5]);
    h_busassign = add_block('simulink/Signal Routing/Bus Assignment', [subSysName '/BusAssignment'], 'ShowName', 'off', 'AssignedSignals', strAssigned, 'Position', [posByteUnpack(1)+160 posByteUnpack(2)-30 posByteUnpack(3)+280 posByteUnpack(4)-5]);
    portPositionOut = get_param(get_param(h_busassign,'PortHandles').Outport(1),'Position');
    set_param([subSysName '/Out1'], 'Position', [portPositionOut(1)+60, portPositionOut(2)-7, portPositionOut(1)+90, portPositionOut(2)+7]);
    
    add_line(subSysName, get_param([subSysName '/In1'],'PortHandles').Outport(1), get_param(h_selector,'PortHandles').Inport(1), 'autorouting', 'smart');
    add_line(subSysName, get_param(h_selector,'PortHandles').Outport(1), get_param(h_bytepack,'PortHandles').Inport(1), 'autorouting', 'smart');
    add_line(subSysName, get_param(h_busassign,'PortHandles').Outport(1), get_param([subSysName '/Out1'],'PortHandles').Inport(1), 'autorouting', 'smart');
    add_line(subSysName, get_param(h_subref,'PortHandles').Outport(1), get_param(h_busassign,'PortHandles').Inport(1), 'autorouting', 'smart');
    for i = 1:numel(structInfo)
        add_line(subSysName, get_param(h_bytepack,'PortHandles').Outport(i), get_param(h_busassign,'PortHandles').Inport(i + 1), 'autorouting', 'smart');
    end
end

