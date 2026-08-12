function inertia = estimateInertia(combined,inertiaRatio)

torque = squeeze(sum(combined.preTorqueGeom,2));

I = zeros(3,1);

figure

for axis = 1:2

    dw_full = combined.domega(:,axis);

    torque_full = torque(:,axis);

    perc = 50;

    dwLower = prctile(dw_full,perc);
    dwUpper = prctile(dw_full,100-perc);

    mask = ...
        (dw_full < dwLower) | ...
        (dw_full > dwUpper);

    dw = dw_full(mask);

    tau = torque_full(mask);

    I(axis) = ...
        (dw' * tau) / ...
        (dw' * dw);

    subplot(1,2,axis)

    scatter(torque_full,...
            dw_full,...
            5,...
            '.');

    hold on

    x = linspace(min(torque_full),...
                 max(torque_full),...
                 100);

    plot(x,...
         x./I(axis),...
         'r',...
         'LineWidth',2);

    grid on

    xlabel('Torque [Nm]')

    ylabel('\omega dot [rad/s^2]')

    title(sprintf('I_%c%c = %.4f',...
        'x'+axis-1,...
        'x'+axis-1,...
        I(axis)));

end





%% ----------------------------------------------------------
%% Build inertia vector
%% ----------------------------------------------------------

Ixx = I(1);
Iyy = I(2);

Izz = ((Ixx + Iyy)/2) * inertiaRatio;

I = [Ixx Iyy Izz];

fprintf('\n');
fprintf('===============================\n');
fprintf('Estimated inertia\n');
fprintf('Ixx = %.6f kg*m^2\n',Ixx);
fprintf('Iyy = %.6f kg*m^2\n',Iyy);
fprintf('Izz = %.6f kg*m^2\n',Izz);
fprintf('===============================\n');


%% ==========================================================
%% Estimate motor torque constant Ktau
% ==========================================================

thrustTorqueInputs = ...
    combined.thrusts(:,:,ones(1,3));

thrustTorqueInputs = ...
    thrustTorqueInputs .* model.rotorTorqueDirections;



thrustTorqueInputsZ = ...
    squeeze(sum(thrustTorqueInputs(:,:,3),2));


%%debug
fprintf("Roll torque range:  %.4f  %.4f\n", ...
    min(torque(:,1)), max(torque(:,1)));

fprintf("Roll accel range:   %.4f  %.4f\n", ...
    min(combined.domega(:,1)), max(combined.domega(:,1)));




dwz = combined.domega(:,3);

b = Izz .* dwz;

Ktau = ...
    dot(thrustTorqueInputsZ,b) / ...
    dot(thrustTorqueInputsZ,thrustTorqueInputsZ);

%%


inertia.Ixx = Ixx;
inertia.Iyy = Iyy;
inertia.Izz = Izz;
% inertia.Ktau = Ktau;

inertia.vector = [Ixx Iyy Izz];
% fprintf('Ktau = %.8e\n',Ktau);

end


























































% function inertia = estimateInertia(combined,model,Kf,inertiaRatio)
% 
% thrust = combined.thrust;
% 
% %%
% torque = squeeze(sum(combined.preTorqueGeom,2));
% N = size(thrust, 1);
% 
% disp(size(combined.rpms))
% disp(size(combined.preTorqueGeom))
% disp(size(combined.thrusts))
% disp(size(combined.domega))
% 
% %% ==========================================================
% %% Estimate Roll Inertia (Ixx)
% %% ==========================================================
% 
% pdot = combined.domega(:,1);
% tx   = torque(:,1);
% 
% perc = 50;
% 
% low  = prctile(pdot,perc);
% high = prctile(pdot,100-perc);
% 
% mask = (pdot < low) | (pdot > high);
% 
% fprintf('Roll samples kept : %d / %d\n',sum(mask),length(mask));
% 
% pdot = pdot(mask);
% tx   = tx(mask);
% 
% Ixx = dot(tx,pdot) / dot(pdot,pdot);
% 
% %% ==========================================================
% %% Estimate Pitch Inertia (Iyy)
% %% ==========================================================
% 
% qdot = combined.domega(:,2);
% ty   = torque(:,2);
% 
% low  = prctile(qdot,perc);
% high = prctile(qdot,100-perc);
% 
% mask = (qdot < low) | (qdot > high);
% 
% fprintf('Pitch samples kept : %d / %d\n',sum(mask),length(mask));
% 
% qdot = qdot(mask);
% ty   = ty(mask);
% 
% Iyy = dot(ty,qdot) / dot(qdot,qdot);
% 
% %% ==========================================================
% %% Estimate Yaw Inertia
% %% ==========================================================
% 
% Izz = inertiaRatio * (Ixx + Iyy) / 2;
% 
% %% ==========================================================
% %% Estimate Rotor Drag Coefficient
% %% ==========================================================
% 
% rdot = combined.domega(:,3);
% 
% b = Izz * rdot;
% 
% yawTorque = zeros(N,1);
% 
% for k = 1:N
% 
%     for m = 1:4
% 
%         yawTorque(k) = yawTorque(k) + ...
%             model.rotorTorqueDirections(m,3) * ...
%             thrust(k,m);
% 
%     end
% 
% end
% 
% Ktau = dot(yawTorque,b) / dot(yawTorque,yawTorque);
% 
% %% ==========================================================
% %% Store results
% %% ==========================================================
% 
% inertia.Ixx = Ixx;
% inertia.Iyy = Iyy;
% inertia.Izz = Izz;
% 
% inertia.Ktau = Ktau;
% 
% inertia.thrust = thrust;
% inertia.torque = torque;
% 
% inertia.pdot = combined.domega(:,1);
% inertia.qdot = combined.domega(:,2);
% inertia.rdot = combined.domega(:,3);
% 
% %% ==========================================================
% %% Display Results
% %% ==========================================================
% 
% fprintf('\n');
% fprintf('----------- Estimated Inertia -----------\n');
% fprintf('Ixx   = %.6f kg*m^2\n',Ixx);
% fprintf('Iyy   = %.6f kg*m^2\n',Iyy);
% fprintf('Izz   = %.6f kg*m^2\n',Izz);
% fprintf('Ktau  = %.8f\n',Ktau);
% fprintf('-----------------------------------------\n');
% 
% end
% 
