clear
clc

%% Load PX4 logs

% ulogFiles = {
%     "C:\Users\AbdulkadirYilmaz\Downloads\data-driven-system-identification-main\data-driven-system-identification-main\logs_large\log_63_2024-1-8-16-37-54.ulg"
%     "C:\Users\AbdulkadirYilmaz\Downloads\data-driven-system-identification-main\data-driven-system-identification-main\logs_large\log_64_2024-1-8-16-39-44.ulg"
%     "C:\Users\AbdulkadirYilmaz\Downloads\data-driven-system-identification-main\data-driven-system-identification-main\logs_large\log_65_2024-1-8-16-40-52.ulg"
%     "C:\Users\AbdulkadirYilmaz\Downloads\data-driven-system-identification-main\data-driven-system-identification-main\logs_large\log_66_2024-1-8-16-42-48.ulg"
% };
logs(1).file = "C:\Users\AbdulkadirYilmaz\Downloads\data-driven-system-identification-main\data-driven-system-identification-main\logs_large\log_63_2024-1-8-16-37-54.ulg";
logs(1).type = "thrust";
logs(1).timeframes = [
    0 55
];

logs(2).file = "C:\Users\AbdulkadirYilmaz\Downloads\data-driven-system-identification-main\data-driven-system-identification-main\logs_large\log_64_2024-1-8-16-39-44.ulg";
logs(2).type = "roll_pitch";
logs(2).timeframes = [
    15 80
    20 80
];

logs(3).file = "C:\Users\AbdulkadirYilmaz\Downloads\data-driven-system-identification-main\data-driven-system-identification-main\logs_large\log_65_2024-1-8-16-40-52.ulg";
logs(3).type = "roll_pitch";
logs(3).timeframes = [
    18 80
    15 80
];

logs(4).file = "C:\Users\AbdulkadirYilmaz\Downloads\data-driven-system-identification-main\data-driven-system-identification-main\logs_large\log_66_2024-1-8-16-42-48.ulg";
logs(4).type = "yaw";
logs(4).timeframes = [
    12 28
];

% 
% flights = cell(length(ulogFiles),1);
% 
% for i = 1:length(ulogFiles)
% 
%     fprintf("Loading log %d/%d...\n",i,length(ulogFiles));
% 
%     flights{i} = loadULog(ulogFiles{i});
% 
% end

%% Define manoeuvre time windows
%not susing now
% timeframes = defineTimeframes();

%% Crop flights
% t = seconds(flight.gyro.Properties.RowTimes - ...
%             flight.gyro.Properties.RowTimes(1)); toplot for single log

% figure
% plot(t, flight.motor.control(:,1))
% xlabel('Time [s]')
% ylabel('Motor 1 command')
% grid on

% flights_thrust = extractTimeframes(flights,timeframes.thrust);
% 
% flights_roll_pitch = extractTimeframes(flights,timeframes.roll_pitch);
% 
% flights_yaw = extractTimeframes(flights,timeframes.yaw);
% 
% 
% 
%%

%% Plot all original logs for timeframe selection

for i = 1:length(logs)

    % Load original full log
    flightPlot = loadULog(logs(i).file);

    % Time in seconds from beginning of THIS log
    tg = seconds(flightPlot.data.gyro.timestamps - flightPlot.data.gyro.timestamps(1));

    tm = seconds(flightPlot.data.motor.timestamps - flightPlot.data.motor.timestamps(1));

    figure('Name', sprintf('Log %d - %s', i, logs(i).type));

    % =========================
    % Angular velocity
    % =========================
    subplot(2,1,1)

    plot(tg, flightPlot.data.gyro.values(:,1))
    hold on
    plot(tg, flightPlot.data.gyro.values(:,2))
    plot(tg, flightPlot.data.gyro.values(:,3))

    grid on
    ylabel('Angular rate [rad/s]')
    legend('p','q','r')
    title(sprintf('Log %d - %s - Gyro', i, logs(i).type))

    % =========================
    % Motor commands
    % =========================
    subplot(2,1,2)

    plot(tm, flightPlot.data.motor.values)

    grid on
    xlabel('Time from log start [s]')
    ylabel('Motor command')
    legend('M1','M2','M3','M4')
    title('Motor commands')

end
%%
flights_thrust = {};
flights_roll_pitch = {};
flights_yaw = {};

for i = 1:length(logs)

    fprintf("Loading %s\n",logs(i).file);

    flight = loadULog(logs(i).file);

    cropped = extractTimeframes({flight},logs(i).timeframes);

    switch logs(i).type

        case "thrust"

            flights_thrust = [flights_thrust ; cropped];

        case "roll_pitch"

            flights_roll_pitch = [flights_roll_pitch ; cropped];
fprintf("Before append:\n");
whos flights_roll_pitch
whos cropped

disp(size(flights_roll_pitch))
disp(size(cropped))


        case "yaw"

            flights_yaw = [flights_yaw ; cropped];

    end

end


flights_thrust = sliceGapsAndInterpolate(flights_thrust);

flights_roll_pitch = sliceGapsAndInterpolate(flights_roll_pitch);

flights_yaw = sliceGapsAndInterpolate(flights_yaw);

%%
%model params
model.gravity = 9.81;
model.mass = 3.3499;

model.rotorPositions = [
    0.20895   -0.24066   0;
    -0.20895   0.24066   0;
    0.20895   0.24066   0;
    -0.20895   -0.24066   0
];

model.rotorThrustDirections = [
    0 0 1;
    0 0 1;
    0 0 1;
    0 0 1
];

model.rotorTorqueDirections = [
     0 0 -1;
     0 0 1;
     0 0 -1;
     0 0  1
];


% Tm = 0.05;
exponents = [0 1 2];
%% Optimize motor time constant (same as repository)

TmCandidates = linspace(0.001,0.2,100);

rmses = zeros(size(TmCandidates));
Kfs   = cell(size(TmCandidates));

fprintf('Searching for optimal motor time constant...\n');

for i = 1:length(TmCandidates)

    TmTest = TmCandidates(i);

    [combinedTmp, ~] = combineFlights(flights_thrust, TmTest);

    [Kf, rmse] = estimateMotorParameters(combinedTmp, model, exponents, false);

    Kfs{i} = Kf;
    rmses(i) = rmse;

end

[bestRMSE, idx] = min(rmses);

Tm = TmCandidates(idx);
Kf = Kfs{idx};

fprintf('\n');
fprintf('===============================\n');
fprintf('Optimal motor time constant : %.4f s\n',Tm);
fprintf('Minimum thrust RMSE         : %.6f\n',bestRMSE);
fprintf('===============================\n');

%
[combinedThrust, flights_thrust] = ...
    combineFlights(flights_thrust,Tm, Kf, model);

[combinedRollPitch, flights_roll_pitch] = ...
    combineFlights(flights_roll_pitch,Tm, Kf, model);

[combinedYaw, flights_yaw] = ...
    combineFlights(flights_yaw,Tm, Kf, model);



%%
% [Kf, rmse] = estimateMotorParameters( ...         gereksiz yukarida tm Kf
% var
%     combinedThrust,...
%     model,...
%     exponents,...
%     false);
% 
% disp(Kf)
% disp(rmse)

%
disp(size(combinedYaw.thrusts))
disp(size(combinedYaw.preTorque))
disp(size(combinedYaw.preTorqueGeom))
disp(size(combinedYaw.domega))
%%
inertiaRatio = 1.832;

inertia = estimateInertia(combinedRollPitch, inertiaRatio);

disp(inertia)



disp("Finished cropping.");





%%
% %command line 
% % plot(flight.data.gyro.timestamps,...
% %      flight.data.gyro.values(:,1))
% % grid on
% 
% % plot(flight.data.motor.timestamps,...
% %      flight.data.motor.values(:,1))
% % grid on
% % 
% % plot(flight.data.acc.timestamps,...
% %      flight.data.acc.values(:,3))
% 
% figure('Name','Log 1')
% 
% subplot(2,1,1)
% plot(tg1, flight1.data.gyro.values)
% grid on
% ylabel('Angular rate [rad/s]')
% legend('p','q','r')
% title('Log 1 - Gyro')
% 
% subplot(2,1,2)
% plot(tm1, flight1.data.motor.values)
% grid on
% xlabel('Time from log start [s]')
% ylabel('Motor command')
% legend('M1','M2','M3','M4')
% title('Log 1 - Motors')