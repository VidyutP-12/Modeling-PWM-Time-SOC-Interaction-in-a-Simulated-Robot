
clear all; close all; clc;

dt = 0.001;             
t_final = 300;         
time = 0:dt:t_final;   
N = length(time);
PWM_duty_cycle = 0.1; 
PWM_frequency = 1000;  
PWM_voltage = 24;      

battery_capacity = 2.5;     
battery_nominal_voltage = 12;
battery_internal_resistance = 0.05; 
battery_initial_soc = 1.0;  

battery_discharge_curve = [1.0, 0.95, 0.90, 0.80, 0.70, 0.60, 0.50, 0.40, 0.30, 0.20, 0.10, 0.05, 0.0; 
                         12.6, 12.4, 12.2, 12.0, 11.8, 11.6, 11.4, 11.2, 11.0, 10.8, 10.5, 10.2, 9.0]; 
peukert_exponent = 1.2;    
nominal_discharge_rate = 0.5; 

R_motor = 0.5;        
L_motor = 0.002;        
Kt = 0.02;              
Ke = 0.02;            
J_motor = 0.0001;     
b_motor = 0.0001;       
motor_efficiency = 0.80; 
max_motor_current = 20;  

m_robot = 5;        
wheel_radius = 0.1; 
gear_ratio = 5;      
frontal_area = 0.05;   
drag_coefficient = 1.2;
air_density = 1.225;   
rolling_resistance = 0.02
gravity = 9.81;    

wind_speed = 1;       
road_grade = 0;      
temperature = 25;     
tire_pressure_factor = 1.0; 

PWM_switching_loss_factor = 0.05; 
PWM_dead_time = 0.000001;        
PWM_rise_fall_time = 0.0000005;   
control_system_current = 0.05;   
gate_driver_current = 0.02;      

pwm_signal = zeros(1, N);
pwm_effective_duty = zeros(1, N);
effective_voltage = zeros(1, N);
pwm_switching_losses = zeros(1, N);
battery_voltage = zeros(1, N);
battery_current = zeros(1, N);
battery_soc = zeros(1, N);
battery_power = zeros(1, N);
battery_temperature = zeros(1, N);
effective_capacity = zeros(1, N);
internal_resistance = zeros(1, N);
battery_open_voltage = zeros(1, N);
total_battery_current = zeros(1, N); 

i_motor = zeros(1, N);
omega_motor = zeros(1, N);
torque_motor = zeros(1, N);
back_emf = zeros(1, N);
motor_power_in = zeros(1, N);
motor_power_out = zeros(1, N);
motor_efficiency_actual = zeros(1, N);
motor_temperature = zeros(1, N);
motor_voltage_applied = zeros(1, N);  

omega_wheel = zeros(1, N);
v_robot = zeros(1, N);
x_robot = zeros(1, N);
torque_wheel = zeros(1, N);
F_drive = zeros(1, N);
F_drag = zeros(1, N);
F_rolling = zeros(1, N);
F_grade = zeros(1, N);
F_total = zeros(1, N);
robot_power = zeros(1, N);
energy_consumed = zeros(1, N);

range_remaining = zeros(1, N);
efficiency_total = zeros(1, N);
acceleration = zeros(1, N);
speed_moving_avg = zeros(1, N);
speed_variance = zeros(1, N);

amp_hours_consumed_total = zeros(1, N);
soc_change_rate = zeros(1, N);
coulomb_efficiency = 0.98;
instantaneous_power_draw = zeros(1, N);

pwm_rms_current = zeros(1, N);
pwm_avg_current = zeros(1, N);
pwm_power_loss = zeros(1, N);

battery_soc(1) = battery_initial_soc;
battery_open_voltage(1) = interp1(battery_discharge_curve(1,:), battery_discharge_curve(2,:), battery_soc(1));
battery_voltage(1) = battery_open_voltage(1); 
motor_temperature(1) = temperature;
battery_temperature(1) = temperature;
effective_capacity(1) = battery_capacity;
internal_resistance(1) = battery_internal_resistance;

for k = 1:N-1
   t = time(k);
  
 
   battery_open_voltage(k) = interp1(battery_discharge_curve(1,:), battery_discharge_curve(2,:), battery_soc(k), 'linear', 'extrap');
 
   temp_factor_resistance = 1 + (battery_temperature(k) - 25) * 0.01;
   temp_factor_capacity = 1 - abs(battery_temperature(k) - 25) * 0.002;
  
  
   internal_resistance(k) = battery_internal_resistance * temp_factor_resistance;
  
 
   back_emf(k) = Ke * omega_motor(k);
  

   pwm_signal(k) = PWM_duty_cycle;
  
  
   pwm_effective_duty(k) = PWM_duty_cycle * (1 - PWM_dead_time * PWM_frequency * 2);
  
  
   pwm_switching_losses(k) = PWM_switching_loss_factor * PWM_duty_cycle * (1 - PWM_duty_cycle);
  
  
   total_battery_current_guess = total_battery_current(max(1, k-1));
  

   for iter = 1:5  
      
       voltage_drop = total_battery_current_guess * internal_resistance(k);
       battery_voltage(k) = battery_open_voltage(k) - voltage_drop;
      
  
       if battery_voltage(k) < 9.0
           battery_voltage(k) = 9.0;
       end
      
      
       effective_voltage(k) = battery_voltage(k) * pwm_effective_duty(k);
      
    
       motor_voltage_net = effective_voltage(k) - back_emf(k);
       if motor_voltage_net < 0
           motor_voltage_net = 0;
       end
       motor_voltage_applied(k) = motor_voltage_net;
      
    
       steady_state_current = motor_voltage_net / R_motor;
      
   
       time_constant = L_motor / R_motor;
       alpha = dt / (dt + time_constant);
       i_motor(k) = (1-alpha) * i_motor(max(1, k-1)) + alpha * steady_state_current;
      
 
       if i_motor(k) > max_motor_current
           i_motor(k) = max_motor_current;
       elseif i_motor(k) < 0
           i_motor(k) = 0;
       end
      

       pwm_rms_current(k) = i_motor(k) * sqrt(pwm_effective_duty(k));
      
  
       pwm_avg_current(k) = i_motor(k) * pwm_effective_duty(k);
      
  
       pwm_power_loss(k) = pwm_switching_losses(k) * battery_voltage(k);
       pwm_loss_current = pwm_power_loss(k) / battery_voltage(k);
      
     
       total_battery_current_new = pwm_avg_current(k) + ...   
                                 pwm_loss_current + ...    
                                 control_system_current + ... 
                                 gate_driver_current;        
      
       
       if abs(total_battery_current_new - total_battery_current_guess) < 0.001
           break;
       end
      
   
       total_battery_current_guess = 0.6 * total_battery_current_guess + 0.4 * total_battery_current_new;
   end
  

   total_battery_current(k) = total_battery_current_guess;
   battery_current(k) = total_battery_current(k);  
  

   temp_factor = 1 - (motor_temperature(k) - 25) * 0.002;
   torque_motor(k) = Kt * i_motor(k) * temp_factor;
  

   load_factor = abs(i_motor(k)) / max_motor_current;
   pwm_efficiency_factor = 0.95 + 0.05 * pwm_effective_duty(k); 
   motor_efficiency_actual(k) = motor_efficiency * (0.5 + 0.5 * load_factor) * pwm_efficiency_factor;
  

   relative_speed = v_robot(k) + wind_speed;
   F_drag(k) = 0.5 * air_density * drag_coefficient * frontal_area * relative_speed * abs(relative_speed);
  

   F_rolling(k) = rolling_resistance * m_robot * gravity * cos(deg2rad(road_grade)) * tire_pressure_factor;
   if v_robot(k) < 0
       F_rolling(k) = -F_rolling(k);
   end
  

   F_grade(k) = m_robot * gravity * sin(deg2rad(road_grade));
  

   torque_wheel(k) = torque_motor(k) * gear_ratio * motor_efficiency_actual(k);
   F_drive(k) = torque_wheel(k) / wheel_radius;
  
 
   F_total(k) = F_drive(k) - F_drag(k) - F_rolling(k) - F_grade(k);
  

   a_robot = F_total(k) / m_robot;
   acceleration(k) = a_robot;
  

   v_robot(k+1) = v_robot(k) + a_robot * dt;
   x_robot(k+1) = x_robot(k) + v_robot(k) * dt;
  

   omega_wheel(k+1) = v_robot(k+1) / wheel_radius;
   omega_motor(k+1) = omega_wheel(k+1) * gear_ratio;
  

   motor_power_in(k) = motor_voltage_applied(k) * i_motor(k);
   motor_power_out(k) = torque_motor(k) * omega_motor(k);
   robot_power(k) = F_drive(k) * v_robot(k);
   battery_power(k) = battery_voltage(k) * battery_current(k);
   instantaneous_power_draw(k) = battery_power(k);
  

   if battery_current(k) > 0.01
       discharge_rate_factor = (battery_current(k) / nominal_discharge_rate) ^ (peukert_exponent - 1);
       effective_capacity(k) = battery_capacity * temp_factor_capacity / discharge_rate_factor;
   else
       effective_capacity(k) = battery_capacity * temp_factor_capacity;
   end
  

   amp_hours_consumed_this_step = battery_current(k) * dt / 3600; 
   amp_hours_consumed_total(k+1) = amp_hours_consumed_total(k) + amp_hours_consumed_this_step;
  

   if effective_capacity(k) > 0
       soc_reduction = (amp_hours_consumed_this_step / coulomb_efficiency) / effective_capacity(k);
       battery_soc(k+1) = battery_soc(k) - soc_reduction;
       soc_change_rate(k) = -soc_reduction / dt;
   else
       battery_soc(k+1) = battery_soc(k);
       soc_change_rate(k) = 0;
   end
  

   self_discharge_rate = 0.00001;
   battery_soc(k+1) = battery_soc(k+1) - self_discharge_rate * dt / 3600;
  
  
   battery_soc(k+1) = max(0, min(1, battery_soc(k+1)));
  
 
   battery_heat_generation = battery_current(k)^2 * internal_resistance(k);
   battery_cooling_rate = (battery_temperature(k) - temperature) * 0.05;
   battery_temperature(k+1) = battery_temperature(k) + (battery_heat_generation * 0.01 - battery_cooling_rate) * dt;
  

   motor_power_loss = motor_power_in(k) - motor_power_out(k) + pwm_power_loss(k);
   temp_rise = motor_power_loss * 0.1;
   cooling_rate = (motor_temperature(k) - temperature) * 0.1;
   motor_temperature(k+1) = motor_temperature(k) + (temp_rise - cooling_rate) * dt;
  

   energy_consumed(k+1) = energy_consumed(k) + battery_power(k) * dt / 3600;
  

   if battery_current(k) > 0.01
       remaining_capacity = battery_soc(k) * effective_capacity(k);
       time_remaining = remaining_capacity / battery_current(k) * 3600;
       if v_robot(k) > 0.01
           range_remaining(k) = v_robot(k) * time_remaining / 1000;
       else
           range_remaining(k) = range_remaining(max(1, k-1));
       end
   else
       range_remaining(k) = range_remaining(max(1, k-1));
   end
  

   if energy_consumed(k) > 0
       distance_km = x_robot(k) / 1000;
       efficiency_total(k) = distance_km / energy_consumed(k) * 1000;
   end
  

   window_size = min(k, round(5/dt));
   start_idx = max(1, k - window_size + 1);
   speed_moving_avg(k) = mean(v_robot(start_idx:k));
  
   if k > 1
       speed_variance(k) = var(v_robot(max(1,k-window_size+1):k));
   end
end

rpm_motor = omega_motor * 60 / (2*pi);
kmh_robot = v_robot * 3.6;
kmh_moving_avg = speed_moving_avg * 3.6;
battery_soc_percent = battery_soc * 100;
acceleration_ms2 = acceleration;

moving_indices = find(v_robot > 0.01);
if ~isempty(moving_indices)
   max_speed_ms = max(v_robot);
   max_speed_kmh = max_speed_ms * 3.6;
   max_speed_time = time(find(v_robot == max_speed_ms, 1));
  
   avg_speed_moving_ms = mean(v_robot(moving_indices));
   avg_speed_moving_kmh = avg_speed_moving_ms * 3.6;
  
   total_distance = x_robot(end);
   total_time = t_final;
   avg_speed_overall_ms = total_distance / total_time;
   avg_speed_overall_kmh = avg_speed_overall_ms * 3.6;
  
   final_speed_ms = v_robot(end);
   final_speed_kmh = final_speed_ms * 3.6;
  

   steady_state_threshold = 0.9 * max_speed_ms;
   steady_state_indices = find(v_robot >= steady_state_threshold);
   if ~isempty(steady_state_indices)
       time_to_steady_state = time(steady_state_indices(1));
       steady_state_speed_ms = mean(v_robot(steady_state_indices));
       steady_state_speed_kmh = steady_state_speed_ms * 3.6;
   else
       time_to_steady_state = NaN;
       steady_state_speed_ms = NaN;
       steady_state_speed_kmh = NaN;
   end
  
   max_acceleration = max(acceleration_ms2);
   time_to_max_accel = time(find(acceleration_ms2 == max_acceleration, 1));
else
 
   max_speed_ms = 0; max_speed_kmh = 0; max_speed_time = 0;
   avg_speed_moving_ms = 0; avg_speed_moving_kmh = 0;
   avg_speed_overall_ms = 0; avg_speed_overall_kmh = 0;
   final_speed_ms = 0; final_speed_kmh = 0;
   time_to_steady_state = NaN; steady_state_speed_ms = 0; steady_state_speed_kmh = 0;
   max_acceleration = 0; time_to_max_accel = 0;
end

fprintf('=== FULLY CORRECTED PWM-SOC ROBOT SIMULATION ===\n');
fprintf('PWM Duty Cycle: %.1f%% (Effective: %.1f%%)\n', PWM_duty_cycle * 100, mean(pwm_effective_duty) * 100);
fprintf('Distance traveled: %.2f meters\n', x_robot(end));
fprintf('Battery SOC: %.1f%% → %.1f%% (Change: %.1f%%)\n', ...
   battery_soc_percent(1), battery_soc_percent(end), battery_soc_percent(1) - battery_soc_percent(end));
fprintf('Energy consumed: %.2f Wh\n', energy_consumed(end));
fprintf('Total Amp-Hours consumed: %.3f Ah\n', amp_hours_consumed_total(end));
fprintf('Estimated range: %.1f meters\n', range_remaining(end)*1000);
fprintf('\n=== PWM-SOC INTERACTION ANALYSIS ===\n');
fprintf('Average Total Battery Current: %.3f A\n', mean(battery_current(battery_current>0.01)));
fprintf('Average Motor Current: %.3f A\n', mean(i_motor(i_motor>0.01)));
fprintf('Average PWM Average Current: %.3f A\n', mean(pwm_avg_current(pwm_avg_current>0.01)));
fprintf('Average PWM RMS Current: %.3f A\n', mean(pwm_rms_current(pwm_rms_current>0.01)));
fprintf('PWM Efficiency Factor: %.3f\n', mean(pwm_effective_duty(pwm_effective_duty>0)));
fprintf('PWM Switching Losses: %.3f W average\n', mean(pwm_power_loss(pwm_power_loss>0)));
fprintf('\n=== CURRENT BREAKDOWN ===\n');
fprintf('Motor Current (avg): %.3f A\n', mean(pwm_avg_current(pwm_avg_current>0.01)));
fprintf('PWM Loss Current (avg): %.3f A\n', mean(pwm_power_loss(pwm_power_loss>0)) / mean(battery_voltage(battery_voltage>0)));
fprintf('Control System Current: %.3f A\n', control_system_current);
fprintf('Gate Driver Current: %.3f A\n', gate_driver_current);
fprintf('Total Battery Current: %.3f A\n', mean(battery_current(battery_current>0.01)));
fprintf('\n=== SOC SENSITIVITY TO PWM ===\n');
fprintf('PWM Duty Cycle: %.1f%%\n', PWM_duty_cycle * 100);
fprintf('SOC Change: %.2f%%\n', battery_soc_percent(1) - battery_soc_percent(end));
fprintf('SOC Change Rate: %.4f%%/s (avg)\n', mean(abs(soc_change_rate(abs(soc_change_rate)>0))) * 100);

theoretical_motor_power = PWM_duty_cycle * 12 * 2; 
actual_battery_power = mean(battery_power(battery_power>0));
fprintf('Theoretical Motor Power: %.1f W\n', theoretical_motor_power);
fprintf('Actual Battery Power: %.1f W\n', actual_battery_power);
fprintf('Power Ratio: %.3f\n', actual_battery_power / theoretical_motor_power);
fprintf('\n=== PWM VALIDATION TEST ===\n');
fprintf('Change PWM_duty_cycle to 0.5 and compare SOC change!\n');
fprintf('Change PWM_duty_cycle to 0.3 and compare SOC change!\n');
fprintf('Lower PWM = Lower effective voltage = Lower motor current = Lower battery drain = Slower SOC decrease\n');
fprintf('Higher PWM = Higher effective voltage = Higher motor current = Higher battery drain = Faster SOC decrease\n');

figure('Position', [50, 50, 1600, 1200]);

subplot(4,4,1);
plot(time, x_robot, 'g-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Distance (m)');
title('Distance Traveled'); grid on;
subplot(4,4,2);
plot(time, v_robot, 'b-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Speed (m/s)');
title('Robot Speed'); grid on;
subplot(4,4,3);
yyaxis left
plot(time, battery_soc_percent, 'c-', 'LineWidth', 2);
ylabel('SOC (%)', 'Color', 'c');
yyaxis right
plot(time, battery_voltage, 'r-', 'LineWidth', 2);
ylabel('Voltage (V)', 'Color', 'r');
xlabel('Time (s)');
title('Battery SOC & Voltage');
grid on;
subplot(4,4,4);
plot(time, battery_current, 'g-', time, i_motor, 'r--', time, pwm_avg_current, 'b:', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Current (A)');
title('Current Comparison');
legend('Total Battery', 'Motor', 'PWM Avg', 'Location', 'best');
grid on;

subplot(4,4,5);
plot(time, pwm_signal*100, 'b-', time, pwm_effective_duty*100, 'r--', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Duty Cycle (%)');
title('PWM Duty Cycle');
legend('Command', 'Effective', 'Location', 'best');
grid on;
subplot(4,4,6);
plot(time, effective_voltage, 'r-', time, motor_voltage_applied, 'b--', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Voltage (V)');
title('Motor Voltage');
legend('PWM Output', 'Applied to Motor', 'Location', 'best');
grid on;
subplot(4,4,7);
plot(time, pwm_power_loss, 'r-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Power Loss (W)');
title('PWM Switching Losses');
grid on;
subplot(4,4,8);
plot(time, pwm_rms_current, 'r-', time, pwm_avg_current, 'b--', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Current (A)');
title('PWM Current Effects');
legend('RMS', 'Average', 'Location', 'best');
grid on;

subplot(4,4,9);
plot(time, amp_hours_consumed_total, 'b-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Amp-Hours');
title('Cumulative Amp-Hours');
grid on;
subplot(4,4,10);
plot(time, abs(soc_change_rate)*100, 'r-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('SOC Change Rate (%/s)');
title('SOC Change Rate');
grid on;
subplot(4,4,11);
plot(time, battery_open_voltage, 'b-', time, battery_voltage, 'r-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Voltage (V)');
title('Battery Voltage');
legend('Open Circuit', 'Terminal', 'Location', 'best');
grid on;
subplot(4,4,12);
plot(time, instantaneous_power_draw, 'g-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Power (W)');
title('Instantaneous Power Draw');
grid on;

subplot(4,4,13);
plot(time, F_drive, 'r-', time, F_total, 'k-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Force (N)');
title('Forces');
legend('Drive', 'Net', 'Location', 'best');
grid on;
subplot(4,4,14);
plot(time, motor_power_in, 'r-', time, battery_power, 'b-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Power (W)');
title('Power Flow');
legend('Motor In', 'Battery Out', 'Location', 'best');
grid on;
subplot(4,4,15);
plot(time, motor_efficiency_actual*100, 'c-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Efficiency (%)');
title('Motor Efficiency');
grid on;
subplot(4,4,16);
plot(time, back_emf, 'g-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Back EMF (V)');
title('Back EMF');
grid on;
sgtitle(sprintf('PWM-SOC Corrected Robot Simulation - PWM: %.0f%%, SOC: %.1f%% → %.1f%%', ...
   PWM_duty_cycle*100, battery_soc_percent(1), battery_soc_percent(end)));

interval = 10;
sample_times = 0:interval:t_final;                      
sample_indices = round(sample_times / dt) + 1;           
sample_indices(sample_indices > length(battery_soc)) = []; 
soc_at_intervals = battery_soc_percent(sample_indices)'; 
time_at_intervals = sample_times';                      
T = table(time_at_intervals, soc_at_intervals, ...
         'VariableNames', {'Time_s', 'Battery_SOC_Percent'});
writetable(T, 'battery_SOC_log.xlsx');  

