clear;
clc;
setup;


%% --- Setup
t = 0:dt:100;

sys = System();
obj = dataset{randi(numel(dataset))};
k_sens = 0;

for k = 1:length(t)
    sys.update();

    if mod(k, 300) == 0 % Perform  a scan
        [newmap, F, a] = sys.scan_object(obj);
        sys.merge_data(newmap, F, a, 1);
        figure(1), hold on;
        plot(sys, 1);
        plot(obj);
    end

    % if mod(k, 150) == 0
    %     figure(1), clf;
    %     plot(sys.manipulator);
    %     plot(obj);
    %     xlim([-5, 12]);
    %     ylim([-3, 12]);
    %     axis equal
    %     title(sprintf('t = %.2f', t(k)));
    % end
end

