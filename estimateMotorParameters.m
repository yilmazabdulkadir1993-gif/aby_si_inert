function [KfFull, rmse] = estimateMotorParameters(combined, model, exponents, separate)
fprintf("RPM min = %.2f\n",min(combined.rpms(:)));
fprintf("RPM max = %.2f\n",max(combined.rpms(:)));
if nargin < 4
    separate = true;
end

A = [];
b = [];

numSteps = size(combined.rpms,1);

for step = 1:numSteps

    acc = combined.acceleration(step,:);

    b = [b;
         model.mass * acc(:)];

    row = [];




    currentA = [];

rpm = combined.rpms(step,:);

if separate

    for motor = 1:4

        for exponent = exponents

            currentA = [currentA ...
                model.rotorThrustDirections(motor,:)' * rpm(motor)^exponent];

        end

    end

else

    for exponent = exponents

        accDir = zeros(3,1);

        for motor = 1:4

            accDir = accDir + ...
                model.rotorThrustDirections(motor,:)' ...
                * rpm(motor)^exponent;

        end

        currentA = [currentA accDir];

    end

end

A = [A;
     currentA];

    % rpm = combined.rpms(step,:);
    % 
    % if separate
    % 
    %     for motor = 1:4
    % 
    %         dir = model.rotorThrustDirections(motor,:);
    % 
    %         for exponent = exponents
    % 
    %             row = [row;
    %                    dir(:) * rpm(motor)^exponent];
    % 
    %         end
    % 
    %     end
    % 
    % else
    % 
    %     for exponent = exponents
    % 
    %         accDir = zeros(3,1);
    % 
    %         for motor = 1:4
    % 
    %             accDir = accDir + ...
    %                 model.rotorThrustDirections(motor,:)' ...
    %                 * rpm(motor)^exponent;
    % 
    %         end
    % 
    %         row = [row;
    %                accDir];
    % 
    %     end
    % 
    % end
    % 
    % A = [A;
    %      row'];

end



fprintf("A size : %d x %d\n",size(A,1),size(A,2));
fprintf("b size : %d x %d\n",size(b,1),size(b,2));

K = A\b;

rmse = sqrt(mean((A*K-b).^2));

if separate

    K = reshape(K,[],length(exponents));

    KfFull = zeros(4,3);

    for m = 1:4

        for i = 1:length(exponents)

            KfFull(m,exponents(i)+1) = K(m,i);

        end

    end

else

    KfFull = repmat(K(:)',4,1);
    disp(KfFull)
end

end