function rpm = filterEMA(timestamps, setpoints, Tm)

N = size(setpoints,1);
M = size(setpoints,2);

rpm = zeros(N,M);

rpm(1,:) = setpoints(1,:);

for k = 2:N

    dt = timestamps(k) - timestamps(k-1);

    alpha = exp(-dt/Tm);

    rpm(k,:) = alpha .* rpm(k-1,:) + (1-alpha).*setpoints(k,:);

end

end