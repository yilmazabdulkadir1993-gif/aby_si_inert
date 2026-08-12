function croppedFlights = extractTimeframes(flights,timeframes)

croppedFlights = cell(0,1); %creating empty cell array to store values

f = flights{1};%first log of flights object () 

for i = 1:size(timeframes,1)

    startTime = timeframes(i,1);
    endTime   = timeframes(i,2);

    fcrop = f; %copy f

    names = fieldnames(fcrop.data); %copying datas into names variable to compare if they are in the range of selected timeframe

    for k = 1:numel(names)

        name = names{k};

        ts = fcrop.data.(name).timestamps; %taking timestamps of every topic sequencially

        mask = ts > startTime & ts < endTime; %creating mask to chose just desired timeframe data
        %appllying mask
        fcrop.data.(name).timestamps = ts(mask);
        fcrop.data.(name).values = fcrop.data.(name).values(mask,:);

    end

    croppedFlights{end+1,1} = fcrop;

end

end