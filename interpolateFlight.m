function out = interpolateFlight(flight,masterTime)

out = flight;

names = fieldnames(flight.data);

for k = 1:numel(names)

    name = names{k};

    out.data.(name).timestamps = masterTime;

    out.data.(name).values = interp1(...
        flight.data.(name).timestamps,...
        flight.data.(name).values,...
        masterTime,...
        'linear');

end

end