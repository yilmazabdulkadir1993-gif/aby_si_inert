function [combined, flights] = combineFlights(flights,Tm,thrustCurves,model)

if nargin < 3
    thrustCurves = [];
end

combined.rpm_setpoints = [];
combined.rpms          = [];
combined.acceleration  = [];
combined.omega         = [];
combined.domega        = [];

combined.thrusts       = [];
combined.preTorque     = [];
combined.preTorqueGeom = [];


for i = 1:length(flights)

    f = flights{i};

    %% -----------------------------
    % Raw signals
    %% -----------------------------

    timestamps   = f.timestamps;
    rpmSetpoints = f.data.motor.values(:,1:4);

    %% -----------------------------
    % First-order motor dynamics
    %% -----------------------------

    rpms = filterEMA( ...
        timestamps,...
        rpmSetpoints,...
        Tm);

    f.rpm_setpoints = rpmSetpoints;
    f.rpms          = rpms;

    % f.acceleration = f.data.acc.values;
    % f.omega        = f.data.gyro.values;
    % f.domega       = f.data.domega.values;

    f.acceleration = [ ...
    f.data.acc.values(:,1), ...
   -f.data.acc.values(:,2), ...
   -f.data.acc.values(:,3)];

f.omega = [ ...
    f.data.gyro.values(:,1), ...
   -f.data.gyro.values(:,2), ...
   -f.data.gyro.values(:,3)];

f.domega = [ ...
    f.data.domega.values(:,1), ...
   -f.data.domega.values(:,2), ...
   -f.data.domega.values(:,3)];

    %% -----------------------------
    % Compute thrust / torques
    %% -----------------------------

    if nargin >= 4 && ~isempty(thrustCurves)

        N = size(rpms,1);

        thrusts = zeros(N,4);

        preTorque = zeros(N,4,3);

        preTorqueGeom = zeros(N,4,3);

        for k = 1:N

            for m = 1:4

                rpm = rpms(k,m);

            T = 0;
        
            for e = 0:2
                        T = T + thrustCurves(m,e+1) * rpm^e;
            end

                thrusts(k,m) = T;

                %% Rotor reaction torque

                preTorque(k,m,:) = ...
                    T .* model.rotorTorqueDirections(m,:);

                %% Geometric torque

                r = model.rotorPositions(m,:);

                d = model.rotorThrustDirections(m,:);

                preTorqueGeom(k,m,:) = T .* cross(r,d);

            end

        end

        f.thrusts       = thrusts;
        f.preTorque     = preTorque;
        f.preTorqueGeom = preTorqueGeom;

    end

    flights{i} = f;

    %% -----------------------------
    % Concatenate signals
    %% -----------------------------

    combined.rpm_setpoints = [
        combined.rpm_setpoints;
        f.rpm_setpoints];

    combined.rpms = [
        combined.rpms;
        f.rpms];

    combined.acceleration = [
        combined.acceleration;
        f.acceleration];

    combined.omega = [
        combined.omega;
        f.omega];

    combined.domega = [
        combined.domega;
        f.domega];

    %% -----------------------------
    % Concatenate thrust information
    %% -----------------------------

    if isfield(f,'thrusts')

        combined.thrusts = [
            combined.thrusts;
            f.thrusts];

        combined.preTorque = cat(1,...
            combined.preTorque,...
            f.preTorque);

        combined.preTorqueGeom = cat(1,...
            combined.preTorqueGeom,...
            f.preTorqueGeom);

    end

end


%% ============================================================
%% First-order exponential motor model
%% ============================================================

function y = filterEMA(t,u,Tm)



N = size(u,1);

M = size(u,2);

y = zeros(N,M);

if N==0
    return
end

y(1,:) = u(1,:);

for k = 2:N

   % dt = seconds(t(k)-t(k-1));
dt = t(k) - t(k-1);


    if Tm <= 0
        alpha = 1;
    else
        alpha = 1-exp(-dt/Tm);
    end

    y(k,:) = ...
        y(k-1,:) + alpha*(u(k,:)-y(k-1,:));

end

end
save("combined_debug.mat", "combined");
end






























% function [combined, flights] = combineFlights(flights, Tm, thrustCurves, model)
% 
% combined.rpm_setpoints = [];
% combined.rpms          = [];
% combined.acceleration  = [];
% combined.omega         = [];
% combined.domega        = [];
% %thrust torque added
% combined.thrusts       = [];
% combined.preTorque     = [];
% combined.preTorqueGeom     = [];
% 
% if nargin < 3
%     thrustCurves =  [];  %modifyin to calculate torq moment inisde of combine but not in estinertia script
% end
% 
% for k = 1:numel(flights)
% 
%     f = flights{k};
% 
%     timestamps = f.timestamps;
% 
%     %% actuator commands
% 
%     rpmSetpoints = f.data.motor.values(:,1:4);
% 
%     %% first-order motor model
% 
%     rpms = filterEMA(timestamps,rpmSetpoints,Tm);
% 
%     %% save back into flight
% 
%     f.rpm_setpoints = rpmSetpoints;
%     f.rpms = rpms;
% 
%     f.acceleration = f.data.acc.values;
%     f.omega = f.data.gyro.values;
%     f.domega = f.data.domega.values;
% 
%     flights{k} = f;
% 
%     %% concatenate
% 
%     combined.rpm_setpoints = ...
%         [combined.rpm_setpoints;
%          rpmSetpoints];
% 
%     combined.rpms = ...
%         [combined.rpms;
%          rpms];
% 
%     combined.acceleration = ...
%         [combined.acceleration;
%          f.acceleration];
% 
%     combined.omega = ...
%         [combined.omega;
%          f.omega];
% 
%     combined.domega = ...
%         [combined.domega;
%          f.domega];
% 
% 
% 
% end
% %% thrust and torque calc
% if ~isempty(thrustCurves)
% 
%     N = size(f.data.motor.values,1);
% 
%     thrusts = zeros(N,4);
%     preTorque = zeros(N,4,3);
%     preTorqueGeom = zeros(N,4,3);
% 
%     for k = 1:N
% 
%         for m = 1:4
% 
%             rpm = rpms(k,m);
% 
%             T = 0;
% 
%             for e = 0:2
%                 T = T + thrustCurves(m,e+1) * rpm^e;
%             end
% 
%             thrusts(k,m) = T;
% 
%             preTorque(k,m,:) = ...
%                 T .* model.rotorTorqueDirections(m,:);
% 
%             preTorqueGeom(k,m,:) = ...
%                 T .* cross( ...
%                     model.rotorPositions(m,:), ...
%                     model.rotorThrustDirections(m,:) ...
%                 );
% 
%         end
% 
%     end
% 
%     f.thrusts = thrusts;
%     f.preTorque = preTorque;
%     f.preTorqueGeom = preTorqueGeom;
% 
% end
% 
% 
%     if isfield(flights{k},"thrusts")
% 
%     combined.thrusts = ...
%         [combined.thrusts;
%          flights{k}.thrusts];
% 
%     combined.preTorque = ...
%         cat(1,...
%             combined.preTorque,...
%             flights{k}.preTorque);
% 
%     combined.preTorqueGeom = ...
%         cat(1,...
%             combined.preTorqueGeom,...
%             flights{k}.preTorqueGeom);
% 
% end