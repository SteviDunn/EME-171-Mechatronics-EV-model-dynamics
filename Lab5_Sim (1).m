clc
clear
close
global Kp Ki
% Initial conditions
pL0 = 0; pM0 = 0; d_actual0 = 0; d_ref0 = 0;
x0 = [pL0; pM0; d_actual0; d_ref0];

%Change Kp and Ki for part 2!
Kp =200;
Ki = 310;

%Step input refrence velocity
%%v_ref = 1; Part 2 of the Lab v ref
%v_ref = LA92Oracle(t); %Part 3 of the Lab

% Simulation parameters
%%u_in = 100;                      % Input voltage (step) part 1
%%tspan = [0 4];                %for part 1
%%tspan = [0 2];                   % Simulation time for part 2 -> 0:2
tspan = 0:0.01:300; %Simulation time span with .01 step size for part 3

% Solve the system
[t, s] = ode45(@EVModel, tspan, x0);

ds = zeros(length(t),4);
power = zeros(length(t),2);

for i = 1:length(t)
    [ds(i,:), power(i,:)] = EVModel(t(i), s(i,:));
end

% Extract variables
p_L = s(:, 1);                   % Motor flux linkage
p_M = s(:, 2);                   % Vehicle momentum
v_actual = p_M / 2200;           % Convert momentum to velocity
d_actual = s(:, 3);

vref=zeros(length(t),1); 
for i=1:length(t) 
vref(i) = LA92Oracle(t(i)); 
end 

%%Part 2
% Plot the results
figure;
plot(t, v_actual, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Vehicle Velocity (m/s)');
title('Vehicle Velocity Response');
grid on;

%%Part 3
% Plot the results (zoomed in to 32s to 54s, 4 m/s to 8 m/s)
figure;
plot(t, v_actual, 'b-', 'LineWidth', 1.5); % Plot actual velocity
hold on;
plot(t, vref, 'r--', 'LineWidth', 1.5); % Plot reference velocity
hold off;
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Vehicle Velocity vs. Reference Velocity (Zoomed)');
legend('Actual Velocity', 'Reference Velocity', 'Location', 'Best');
grid on;
axis([32, 54, 4, 8]); % Set the x-axis and y-axis limits


%part3

Pin_acc = zeros(length(t),1);
for i = 1:length(t)
     if power(i,1) > 0
        Pin_acc(i) = power(i,1);
     end
end
Pout_acc = zeros(length(t),1);
for i = 1:length(t)
     if power(i,1) > 0
        Pout_acc(i) = power(i,2);
     end
end
Pin_acc = nonzeros(Pin_acc);
Pout_acc = nonzeros(Pout_acc);

Pin_avg = mean(Pin_acc)
Pout_avg= mean(Pout_acc)
EffAvg = (Pout_avg/Pin_avg)*100 %Average Efficiency in %

MetersperJoule= sum(ds(:,2))/sum(ds(:,3))

MPGe=33.7*10^6*MetersperJoule/1609
