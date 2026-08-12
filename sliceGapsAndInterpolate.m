function flightsOut = sliceGapsAndInterpolate(flights)

flightsOut = {};

for f = 1:length(flights) %select the logs sequencially from flights object

    flight = flights{f};

    names = fieldnames(flight.data);

    %% Highest / lowest frequency
    
    %find the lowest and highest freq inside of the log
    lowestFreq = inf;
    highestFreq = 0;

    lowestName = "";
    highestName = "";

    for i = 1:numel(names)

        name = names{i};

        ts = flight.data.(name).timestamps;

        freq = 1/median(diff(ts)); %taking average of time intervals like median(0.004 0.0039 0.0041 ...) 1/0.00403= 248Hz (more robust than using df directly)

    %find the lowest and highest freq inside of the log
        if freq < lowestFreq
            lowestFreq = freq;
            lowestName = name;
        end

        if freq > highestFreq
            highestFreq = freq;
            highestName = name;  %probably gyro
        end

    end
    
    %if there is a gap between datas it must be max the varieable below
    intervalThreshold = 3/lowestFreq;

    fprintf("Lowest frequency : %.2f Hz (%s)\n",lowestFreq,lowestName);
    fprintf("Highest frequency: %.2f Hz (%s)\n",highestFreq,highestName);

    %% Synchronize start/end

    earliest = -inf;
    latest = inf;

    for i = 1:numel(names)

        ts = flight.data.(names{i}).timestamps;
        
        %Tüm sinyallerin ortak olduğu zaman aralığı bulunuyor (hepsinin
        %veri sağladığı kesişim bölgesi gyro acc motor icin). 
        % gyro:     10 ------------------------- 50
        % rpm:         12 ------------------- 48
        % acc:            15 ----------------------- 52
        earliest = max(earliest,ts(1)); % en geç başlayan sinyalin başlangıcı

        latest = min(latest,ts(end));   % en erken biten sinyalin bitişi
    end

    masterFull = flight.data.(highestName).timestamps; %gyro referans secilir frekans icin

    master = masterFull(masterFull>earliest & masterFull<latest); %the data with highest timestamp selected as 'master timeline' 

    earliest = master(1);

    latest = master(end);

    totalTime = latest-earliest;

%% Detect gaps

%gaps = [];
gaps = zeros(0,2);

for i = 1:numel(names)

    name = names{i};

    ts = flight.data.(name).timestamps;

    ts = ts(ts > earliest & ts < latest);   %filter the time interval acc. to master

    tsAug = [earliest; ts; latest];         %if there is gap at starting or ending of series, put there values of earliest and latest points of master, then assign it tsAug

    d = diff(tsAug);                        %then look at new series and detect the anormal gaps if there is

    idx = find(d > intervalThreshold);      %compare it against criteria that we chose

    for k = 1:length(idx)

        gapStart = tsAug(idx(k));           %tell the start and end of gaps
        gapEnd   = tsAug(idx(k)+1);

        gaps = [gaps;
                gapStart gapEnd];           %assign it to gaps 

                                            % sonuç: exmpl
                                            % gaps = [1.0 1.5;
                                            %         3.0 3.2;
                                            %         5.0 5.4];

    end

end
    %% Sort
%disp("---- BEFORE SORTROWS debug----")
%disp(size(gaps))
%disp(gaps)
%keyboard
    %gaps = sortrows(gaps,1);
    if ~isempty(gaps)
    gaps = sortrows(gaps,1);                %sort the data acc. to first column, acc. to time
end
    %% Merge gaps
    %if multiple gaps points same time intervals merge them in a
    %chronological
    merged = [];

    if ~isempty(gaps)

        currentStart = gaps(1,1);

        currentEnd = gaps(1,2);

        for k = 2:size(gaps,1)

            if gaps(k,1)-currentEnd <= intervalThreshold        %if time gap is more tiny merge it to next one via...----

                currentEnd = max(currentEnd,gaps(k,2));         %compare the end of the second column of next row if it has greater end take it as new end

            else

                merged = [merged;                               %if its greater tahn criteria saved it as it is
                          currentStart currentEnd];

                currentStart = gaps(k,1);

                currentEnd = gaps(k,2);

            end

        end

        merged = [merged;
                  currentStart currentEnd];

    end

    %% Split into subflights

    currentStart = earliest;            %start with earliest

    for k = 1:size(merged,1)+1          % earliest-----DATA-----GAP-----DATA------ GAP-----DATA------latest         2 gap may mean 3 data thatswhy +1 is added
                                        %            segment1         segment2           segment2         

        if k<=size(merged,1)

            segmentEnd = merged(k,1);   % start of k'th gap is end of the this segment

            nextStart = merged(k,2);    % end of k'th gap is start of the next segment

        else

            segmentEnd = latest;        %if its already latest, go lto latest

            nextStart = latest;

        end

        segmentTime = segmentEnd-currentStart;

        if segmentTime>0.01*totalTime       %if the gap is less than this, this if  condition never run and the tiny gap is skipped. if its greater tahn this it creates a flight segment

            masterSegment = master(master>currentStart & master<segmentEnd);    %creating new master line for segments

            sub = struct();

            for i = 1:numel(names)

                name = names{i}; % for gyro, acc, motor in the log


                sub.(name).timestamps = masterSegment;

                sub.(name).values = interpSignal(flight.data.(name).timestamps, flight.data.(name).values, masterSegment); %interpolate this data acc. to new master time line of segment to make each data as syncronous in the segment like gyro, acc, motor 

                %sub.(name).timestamps = masterSegment;

                %sub.(name).values = interp1( ...
                %   flight.data.(name).timestamps,...
                %   flight.data.(name).values,...
                %   masterSegment,...
                %   'linear');

            end

            newFlight.name = sprintf("%s_%d",flight.name,k);    %Bu segment, orijinal uçuş ismine _1, _2, _3 gibi bir ek (k numarası) eklenerek yeni bir struct (newFlight) haline getiriliyor ve flightsOut cell array'ine ekleniyor.

            newFlight.convention = flight.convention;

            newFlight.timestamps = masterSegment;

            newFlight.data = sub;

            flightsOut{end+1} = newFlight;                      %script does this for whole logs and put the outputs as a one struct whic is flightsOut

        end

        currentStart = nextStart;   %go to next one and repeat the loop for next row

    end

end

end