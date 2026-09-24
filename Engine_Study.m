clc
clear
%% 1.1 USER INPUTS

engine_displacement = 0.002;       % Engine displacement [m^3]

engine_rpm_min = 1500;             % Minimum engine speed [rpm]
engine_rpm_max = 10000;            % Maximum engine speed [rpm]

target_power_hp = 400;             % Desired peak power [hp]

number_of_cylinders = 4;           % Number of cylinders [-]

fuel_LHV = 43e6;                   % Fuel lower heating value [J/kg]

ambient_pressure = 101325;         % Ambient pressure [Pa]
ambient_temperature = 293.15;      % Ambient temperature [K]

air_gas_constant = 287;            % Air gas constant [J/kg K]
gamma = 1.4;                       % Ratio of specific heats [-]
Cp = 1005;                         % Specific heat capacity at constant pressure [J/kg]

brake_thermal_efficiency = 0.35;   % Brake thermal efficiency [-]

compressor_efficiency = 0.72;      % tbc later

% ENGINE CHARACTERISTICS

rpm = engine_rpm_min:100:engine_rpm_max; % rpm range

VE_rpm = [1500 2000 3000 4000 5000 6000 7000 8000 9000 10000];
VE_data = [0.65 0.72 0.80 0.87 0.92 0.96 0.98 0.95 0.89 0.82];
VE = interp1(VE_rpm, VE_data, rpm, "pchip");

AFR_rpm = [1500 2000 3000 4000 5000 6000 7000 8000 9000 10000];
AFR_data = [13.97 13.52 13.23 12.94 12.79 12.64 12.50 12.50 12.64 12.94];
AFR = interp1(AFR_rpm, AFR_data, rpm, "pchip");

target_power = target_power_hp * 745.7;   % Target power [W]

%% 1.2 NA ENGINE CALCULATION 

air_density_ambient = zeros(size(rpm));
volumetric_flow_NA = zeros(size(rpm));
mass_flow_NA = zeros(size(rpm));
fuel_mass_flow_NA = zeros(size(rpm));
brake_power_NA = zeros(size(rpm));

for k = 1:length(rpm)

    % Ambient Air Density
    air_density_ambient(k) = ambient_pressure / (air_gas_constant * ambient_temperature);

    % Engine Volumetric Flowrate 
    volumetric_flow_NA(k) = VE(k) * engine_displacement * rpm(k) / 120; 
    
    % Engine Massflow 
    mass_flow_NA(k) = air_density_ambient(k) * volumetric_flow_NA(k);

    % Fuel Massflow
    fuel_mass_flow_NA(k) = mass_flow_NA(k) / AFR(k);

    % Naturally Aspirated Brake Power
    brake_power_NA(k) = brake_thermal_efficiency * fuel_mass_flow_NA(k) * fuel_LHV;
end

%% 1.3 ESTABLISHING DESIRED POWER CURVE

% Engine speeds used to define the desired power curve [rpm]
power_curve_rpm = [3000 4000 5000 6000 7000 8000 9000 9500];

% Desired engine power as a fraction of peak power [-]
power_curve_fraction = [0.35 0.50 0.65 0.78 0.88 0.95 0.99 1.00];


% Check that the two input arrays have the same length

if length(power_curve_rpm) ~= length(power_curve_fraction)

    error('power_curve_rpm and power_curve_fraction must have the same length');

end


% Number of points defining the desired power curve

number_of_power_curve_points = length(power_curve_rpm);


% Calculate desired power at each defined point [W]

power_curve_power = zeros(size(power_curve_rpm));

for k = 1:number_of_power_curve_points

    power_curve_power(k) = target_power * power_curve_fraction(k);

end


% Check that the desired power curve reaches peak power

if max(power_curve_fraction) ~= 1.00

    warning('Desired power curve does not reach 100 percent of target peak power');
end

%% 1.4 DESIRED POWER AT ENGINE OPERATING SPEEDS

% Initialise desired power array [W]

desired_power = zeros(size(rpm));

% Calculate desired power at each engine speed

for k = 1:length(rpm)

    desired_power(k) = interp1(power_curve_rpm, power_curve_power, rpm(k), 'pchip', 'extrap');

end

% Prevent negative power requirements

for k = 1:length(rpm)

    if desired_power(k) < 0

        desired_power(k) = 0;

    end

end

% Limit desired power to the specified peak power

for k = 1:length(rpm)

    if desired_power(k) > target_power

        desired_power(k) = target_power;

    end

end


% Display power curve information

fprintf('\n--- DESIRED ENGINE POWER CURVE ---\n');

fprintf('Peak target power = %.1f hp\n', target_power / 745.7);

fprintf('Power curve range = %.1f - %.1f hp\n', min(desired_power) / 745.7, max(desired_power) / 745.7);

%% 1.5 COMPRESSOR REQUIREMENTS

% Compressor isentropic efficiency [-]
compressor_efficiency = 0.72;

% Initialise compressor requirement arrays

required_airmass_flow = zeros(size(rpm));

required_fuel_mass_flow = zeros(size(rpm));

required_exhaust_mass_flow = zeros(size(rpm));

volumetric_flow_required = zeros(size(rpm));

density_required = zeros(size(rpm));

pressure_ratio_required = zeros(size(rpm));

t_compressor_out = zeros(size(rpm));

compressor_specific_work = zeros(size(rpm));

compressor_power_required = zeros(size(rpm));

compressor_outlet_pressure = zeros(size(rpm));

compressor_outlet_density = zeros(size(rpm));


% Calculate compressor requirements at each engine speed

for k = 1:length(rpm)

    % Required fuel mass flow [kg/s]
    required_fuel_mass_flow(k) = desired_power(k) / (brake_thermal_efficiency * fuel_LHV);

    % Required air mass flow [kg/s]
    required_airmass_flow(k) = required_fuel_mass_flow(k) * AFR(k);

    % Required exhaust mass flow [kg/s]
    required_exhaust_mass_flow(k) = required_airmass_flow(k) + required_fuel_mass_flow(k);

    % Volumetric flow the cylinders can ingest at this rpm and VE [m^3/s]
    volumetric_flow_required(k) = VE(k) * engine_displacement * rpm(k) / 120;

    % Intake density required to deliver the required air mass flow
    % through the available engine volumetric flow [kg/m^3]
    density_required(k) = required_airmass_flow(k) / volumetric_flow_required(k);

    % Solve for pressure ratio such that compressor outlet density
    % equals the required intake manifold density.
    %
    % Closure assumption:
    % No intercooler is included in the current model.
    %
    % Compressor outlet conditions are therefore treated as the
    % intake manifold conditions.

    density_error = @(pressure_ratio) (pressure_ratio * ambient_pressure) / (air_gas_constant * ambient_temperature * (1 + (pressure_ratio^((gamma - 1) / gamma) - 1) / compressor_efficiency)) - density_required(k);

    % Iteratively solve for the required compressor pressure ratio [-]
    pressure_ratio_required(k) = fzero(density_error, 2.0);

    % Compressor outlet temperature [K]
    t_compressor_out(k) = ambient_temperature * (1 + (pressure_ratio_required(k)^((gamma - 1) / gamma) - 1) / compressor_efficiency);

    % Compressor outlet absolute pressure [Pa]
    compressor_outlet_pressure(k) = pressure_ratio_required(k) * ambient_pressure;

    % Compressor outlet density [kg/m^3]
    % This should be approximately equal to density_required(k)
    compressor_outlet_density(k) = compressor_outlet_pressure(k) / (air_gas_constant * t_compressor_out(k));

    % Compressor specific work [J/kg]
    compressor_specific_work(k) = Cp * (t_compressor_out(k) - ambient_temperature);

    % Compressor power required [W]
    compressor_power_required(k) = required_airmass_flow(k) * compressor_specific_work(k);

end

fprintf('\n--- 1.5 COMPRESSOR REQUIREMENTS ---\n');
fprintf('Engine speed range = %.0f - %.0f rpm\n', rpm(1), rpm(length(rpm)));
fprintf('Desired power range = %.1f - %.1f hp\n', min(desired_power) / 745.7, max(desired_power) / 745.7);
fprintf('Required air mass flow range = %.4f - %.4f kg/s\n', min(required_airmass_flow), max(required_airmass_flow));
fprintf('Required fuel mass flow range = %.4f - %.4f kg/s\n', min(required_fuel_mass_flow), max(required_fuel_mass_flow));
fprintf('Required exhaust mass flow range = %.4f - %.4f kg/s\n', min(required_exhaust_mass_flow), max(required_exhaust_mass_flow));
fprintf('Required compressor PR range = %.3f - %.3f\n', min(pressure_ratio_required), max(pressure_ratio_required));
fprintf('Compressor outlet temperature range = %.2f - %.2f K\n', min(t_compressor_out), max(t_compressor_out));
fprintf('Compressor specific work range = %.2f - %.2f kJ/kg\n', min(compressor_specific_work) / 1000, max(compressor_specific_work) / 1000);
fprintf('Compressor power range = %.2f - %.2f kW\n', min(compressor_power_required) / 1000, max(compressor_power_required) / 1000);

%% 2.1 DESIGN POINT SELECTION

% The design point is the rpm at which the compressor faces its most
% demanding requirement across the full sweep -- this was previously
% called the "peak power" point, but power alone isn't what sizes the
% compressor; required air mass flow is, since it's what fixes both
% flow area and (via density) pressure ratio. See 1.5 for why this can
% land at a different rpm than peak power itself.

[design_mass_flow, design_index] = max(required_airmass_flow);
design_rpm = rpm(design_index);
design_specific_work = compressor_specific_work(design_index);

fprintf('\n--- 2.1 DESIGN POINT ---\n');
fprintf('Design point rpm = %.0f\n', design_rpm);
fprintf('Design mass flow = %.4f kg/s\n', design_mass_flow);
fprintf('Design specific work = %.2f kJ/kg\n', design_specific_work/1000);

%% 2.2 COMPRESSOR AERODYNAMIC REQUIREMENTS AT THE DESIGN POINT

% NOTE: this section previously referenced peak_power_index and
% peak_power_air_mass_flow, neither of which were ever assigned anywhere
% in the script -- that's the source of the "Unrecognized function or
% variable" error. It now uses design_index/design_mass_flow from 2.1
% directly, which is the same "worst-case operating point" concept,
% just computed explicitly instead of assumed to already exist.

% Compressor inlet temperature at the design operating point [K]
peak_compressor_inlet_temperature = ambient_temperature;

% Compressor outlet temperature at the design operating point [K]
peak_compressor_outlet_temperature = t_compressor_out(design_index);

% Specific compressor work required [J/kg]
peak_compressor_specific_work = Cp * (peak_compressor_outlet_temperature - peak_compressor_inlet_temperature);

% Pressure ratio
peak_power_pressure_ratio = pressure_ratio_required(design_index);

% Compressor pressure increase [Pa]
peak_compressor_pressure_increase = peak_power_pressure_ratio * ambient_pressure - ambient_pressure;

% Compressor power required [W]
peak_power_compressor_power = design_mass_flow * peak_compressor_specific_work;

fprintf('\n--- 2.2 COMPRESSOR AERODYNAMIC REQUIREMENTS ---\n');
fprintf('Compressor inlet temperature  = %.2f K\n', ...
    peak_compressor_inlet_temperature);
fprintf('Compressor outlet temperature = %.2f K\n', ...
    peak_compressor_outlet_temperature);
fprintf('Specific compressor work      = %.2f kJ/kg\n', ...
    peak_compressor_specific_work / 1000);
fprintf('Pressure increase              = %.2f kPa\n', ...
    peak_compressor_pressure_increase / 1000);
fprintf('Compressor power               = %.2f kW\n', ...
    peak_power_compressor_power / 1000);

%% 3.1 COMPRESSOR GEOMETRY FEASIBILITY SEARCH

% Maximum acceptable compressor tip Mach number [-]
maximum_compressor_tip_mach = 1.4;

% Minimum acceptable compressor trim [%]
minimum_compressor_trim = 50;

% Maximum acceptable compressor trim [%]
maximum_compressor_trim = 100;

% Compressor inducer hub-to-tip diameter ratio [-]
hub_to_tip_ratio = 0.40;


% Compressor inlet air properties

compressor_inlet_density = ambient_pressure / (air_gas_constant * ambient_temperature);

compressor_inlet_speed_of_sound = sqrt(gamma * air_gas_constant * ambient_temperature);


% Candidate compressor geometry search space

D2_candidates = 0.030:0.002:0.090;

D1_candidates = 0.015:0.002:0.070;


% Turbocharger speed search range [rpm] -- physically reasonable bounds
% for a small/mid automotive turbocharger shaft speed. Only used via its
% first and last elements to set the omega_scan bracket search bounds
% below, so its intermediate step size doesn't affect the result.
% NOTE: this was another undefined-variable bug (turbo_speed_range was
% used here but never assigned) -- found by the same static check.
turbo_speed_range = 30000:2500:200000;

omega_min = turbo_speed_range(1) * pi / 30;

omega_max = turbo_speed_range(length(turbo_speed_range)) * pi / 30;

omega_scan = linspace(omega_min, omega_max, 60);


% Maximum possible number of compressor geometry candidates

maximum_number_of_compressor_candidates = length(D1_candidates) * length(D2_candidates);


% Initialise arrays to store feasible compressor geometries

feasible_compressor_D1 = NaN(maximum_number_of_compressor_candidates,1);

feasible_compressor_D2 = NaN(maximum_number_of_compressor_candidates,1);

feasible_compressor_trim = NaN(maximum_number_of_compressor_candidates,1);

feasible_compressor_required_turbo_speed = NaN(maximum_number_of_compressor_candidates,length(rpm));

feasible_compressor_tip_mach = NaN(maximum_number_of_compressor_candidates,length(rpm));


% Counter for feasible compressor geometries

number_of_feasible_compressor_geometries = 0;


% Search every candidate compressor geometry

for a = 1:length(D2_candidates)

    D2 = D2_candidates(a);

    for b = 1:length(D1_candidates)

        D1 = D1_candidates(b);


        % Check that inducer diameter is smaller than exducer diameter

        if D1 >= D2

            continue

        end


        % Calculate compressor trim [%]

        trim = 100 * (D1 / D2)^2;


        % Check compressor trim limits

        if trim < minimum_compressor_trim || trim > maximum_compressor_trim

            continue

        end


        % Calculate inducer flow area [m^2]

        A1 = pi / 4 * D1^2 * (1 - hub_to_tip_ratio^2);


        % Assume candidate geometry is initially feasible

        feasible_this_geometry = true;


        % Initialise turbo speed and tip Mach arrays

        turbo_speed_this_geometry = NaN(size(rpm));

        tip_mach_this_geometry = NaN(size(rpm));


        % Test candidate geometry across the complete engine operating range

        for k = 1:length(rpm)


            % Required compressor inlet meridional velocity [m/s]

            Cm1_k = required_airmass_flow(k) / (compressor_inlet_density * A1);


            % Required compressor specific work [J/kg]

            specific_work_k = compressor_specific_work(k);


            % Compressor work equation as a function of shaft angular velocity

            g = @(omega) ((0.68 - ((Cm1_k / (omega * D2 / 2)) / 0.37)^3 + 0.002 / (Cm1_k / (omega * D2 / 2))) * (omega * D2 / 2)^2) - specific_work_k;


            % Calculate error at every turbo speed search point

            g_scan = zeros(size(omega_scan));


            for n = 1:length(omega_scan)

                g_scan(n) = g(omega_scan(n));

            end


            % Search for all sign changes

            sign_change_index = zeros(1,length(omega_scan) - 1);

            number_of_sign_changes = 0;


            for n = 1:length(omega_scan) - 1

                if g_scan(n) == 0

                    number_of_sign_changes = number_of_sign_changes + 1;

                    sign_change_index(number_of_sign_changes) = n;

                elseif g_scan(n) * g_scan(n + 1) < 0

                    number_of_sign_changes = number_of_sign_changes + 1;

                    sign_change_index(number_of_sign_changes) = n;

                end

            end


            % Reject geometry if no solution exists at this engine speed

            if number_of_sign_changes == 0

                feasible_this_geometry = false;

                break

            end


            % Search for a feasible root

            feasible_root_found = false;


            for n = 1:number_of_sign_changes

                phi_lower = NaN;

                phi_upper = NaN;


                phi_lower = Cm1_k / (omega_scan(sign_change_index(n)) * D2 / 2);

                phi_upper = Cm1_k / (omega_scan(sign_change_index(n) + 1) * D2 / 2);


                % Calculate the corresponding angular velocity

                omega_lower = omega_scan(sign_change_index(n));

                omega_upper = omega_scan(sign_change_index(n) + 1);


                % Solve for the turbocharger angular velocity

                omega_sol = fzero(g,[omega_lower omega_upper]);


                % Calculate compressor tip speed [m/s]

                U2_k = omega_sol * D2 / 2;


                % Calculate compressor tip Mach number [-]

                mach_k = U2_k / compressor_inlet_speed_of_sound;


                % Check tip Mach constraint

                if mach_k <= maximum_compressor_tip_mach

                    feasible_root_found = true;

                    turbo_speed_this_geometry(k) = omega_sol * 30 / pi;

                    tip_mach_this_geometry(k) = mach_k;

                    break

                end

            end


            % Reject geometry if no feasible root was found

            if ~feasible_root_found

                feasible_this_geometry = false;

                break

            end

        end


        % Store geometry if it is feasible across the complete engine range

        if feasible_this_geometry

            number_of_feasible_compressor_geometries = number_of_feasible_compressor_geometries + 1;


            feasible_compressor_D1(number_of_feasible_compressor_geometries) = D1;

            feasible_compressor_D2(number_of_feasible_compressor_geometries) = D2;

            feasible_compressor_trim(number_of_feasible_compressor_geometries) = trim;

            for k = 1:length(rpm)

                feasible_compressor_required_turbo_speed(number_of_feasible_compressor_geometries,k) = turbo_speed_this_geometry(k);

                feasible_compressor_tip_mach(number_of_feasible_compressor_geometries,k) = tip_mach_this_geometry(k);

            end

        end

    end

end


% Remove unused rows from the candidate arrays

if number_of_feasible_compressor_geometries > 0

    feasible_compressor_D1 = feasible_compressor_D1(1:number_of_feasible_compressor_geometries);

    feasible_compressor_D2 = feasible_compressor_D2(1:number_of_feasible_compressor_geometries);

    feasible_compressor_trim = feasible_compressor_trim(1:number_of_feasible_compressor_geometries);

    feasible_compressor_required_turbo_speed = feasible_compressor_required_turbo_speed(1:number_of_feasible_compressor_geometries,:);

    feasible_compressor_tip_mach = feasible_compressor_tip_mach(1:number_of_feasible_compressor_geometries,:);

end


% Check whether any feasible compressor geometries exist

if number_of_feasible_compressor_geometries == 0

    error('No compressor geometries are feasible across the complete engine operating range. Widen the geometry range or review the compressor constraints.');

end


%% 3.2 COMPRESSOR FEASIBLE GEOMETRY RESULTS

fprintf('\n--- 3.2 FEASIBLE COMPRESSOR GEOMETRIES ---\n');

fprintf('Number of feasible compressor geometries = %d\n', number_of_feasible_compressor_geometries);


% Determine feasible diameter ranges

minimum_feasible_D1 = min(feasible_compressor_D1);

maximum_feasible_D1 = max(feasible_compressor_D1);

minimum_feasible_D2 = min(feasible_compressor_D2);

maximum_feasible_D2 = max(feasible_compressor_D2);


% Determine feasible trim range

minimum_feasible_trim = min(feasible_compressor_trim);

maximum_feasible_trim = max(feasible_compressor_trim);


% Determine turbo speed range across all feasible compressor geometries

minimum_feasible_turbo_speed = Inf;

maximum_feasible_turbo_speed = -Inf;


for a = 1:number_of_feasible_compressor_geometries

    for k = 1:length(rpm)

        current_turbo_speed = feasible_compressor_required_turbo_speed(a,k);

        if current_turbo_speed < minimum_feasible_turbo_speed

            minimum_feasible_turbo_speed = current_turbo_speed;

        end

        if current_turbo_speed > maximum_feasible_turbo_speed

            maximum_feasible_turbo_speed = current_turbo_speed;

        end

    end

end


% Determine tip Mach range across all feasible geometries

minimum_feasible_tip_mach = Inf;

maximum_feasible_tip_mach = -Inf;


for a = 1:number_of_feasible_compressor_geometries

    for k = 1:length(rpm)

        current_tip_mach = feasible_compressor_tip_mach(a,k);

        if current_tip_mach < minimum_feasible_tip_mach

            minimum_feasible_tip_mach = current_tip_mach;

        end

        if current_tip_mach > maximum_feasible_tip_mach

            maximum_feasible_tip_mach = current_tip_mach;

        end

    end

end


fprintf('Feasible D1 range = %.2f - %.2f mm\n', minimum_feasible_D1 * 1000, maximum_feasible_D1 * 1000);

fprintf('Feasible D2 range = %.2f - %.2f mm\n', minimum_feasible_D2 * 1000, maximum_feasible_D2 * 1000);

fprintf('Feasible trim range = %.1f - %.1f %%\n', minimum_feasible_trim, maximum_feasible_trim);

fprintf('Required turbo speed range = %.0f - %.0f rpm\n', minimum_feasible_turbo_speed, maximum_feasible_turbo_speed);

fprintf('Tip Mach range = %.2f - %.2f\n', minimum_feasible_tip_mach, maximum_feasible_tip_mach);

%% 3.3 FEASIBLE COMPRESSOR GEOMETRY TABLE

fprintf('\n--- FEASIBLE COMPRESSOR GEOMETRY CANDIDATES ---\n');

fprintf('Candidate     D1 [mm]     D2 [mm]     Trim [%%]\n');


for a = 1:number_of_feasible_compressor_geometries

    fprintf('%5d       %8.2f    %8.2f      %7.1f\n', a, feasible_compressor_D1(a) * 1000, feasible_compressor_D2(a) * 1000, feasible_compressor_trim(a));

end

%% 3.4 COMPRESSOR AXIAL GEOMETRY

% Compressor exducer flow coefficient [-]
compressor_exducer_flow_coefficient = 0.30;


% Initialise matrices
%
% Rows = feasible compressor geometry candidates
% Columns = engine operating points

compressor_exducer_velocity = NaN(number_of_feasible_compressor_geometries, length(rpm));

compressor_exducer_area = NaN(number_of_feasible_compressor_geometries, length(rpm));

compressor_exducer_blade_height = NaN(number_of_feasible_compressor_geometries, length(rpm));


% Initialise arrays containing the maximum geometry requirement
% for each compressor candidate

maximum_compressor_exducer_blade_height = NaN(number_of_feasible_compressor_geometries,1);

maximum_compressor_exducer_blade_height_rpm = NaN(number_of_feasible_compressor_geometries,1);

maximum_compressor_total_height = NaN(number_of_feasible_compressor_geometries,1);

maximum_compressor_inducer_blade_height = NaN(number_of_feasible_compressor_geometries,1);


% Calculate geometry at every engine operating point

for a = 1:number_of_feasible_compressor_geometries

    current_D2 = feasible_compressor_D2(a);

    for k = 1:length(rpm)

        % Turbocharger speed required by this compressor geometry
        % at this engine operating point

        current_turbo_speed = feasible_compressor_required_turbo_speed(a,k);

        % Convert turbocharger speed to angular velocity [rad/s]

        current_angular_velocity = current_turbo_speed * pi / 30;

        % Compressor exducer tip speed [m/s]

        current_tip_speed = current_angular_velocity * current_D2 / 2;

        % Compressor exducer meridional velocity [m/s]

        compressor_exducer_velocity(a,k) = compressor_exducer_flow_coefficient * current_tip_speed;

        % Compressor exducer flow area [m^2]

        compressor_exducer_area(a,k) = required_airmass_flow(k) / (compressor_outlet_density(k) * compressor_exducer_velocity(a,k));

        % Compressor exducer blade height [m]

        compressor_exducer_blade_height(a,k) = compressor_exducer_area(a,k) / (pi * current_D2);

    end


    % Find the maximum required blade height for this geometry

    maximum_compressor_exducer_blade_height(a) = 0;
    maximum_compressor_exducer_blade_height_rpm(a) = rpm(1);
    
    for k = 1:length(rpm)
        
        if compressor_exducer_blade_height(a,k) > maximum_compressor_exducer_blade_height(a)

        maximum_compressor_exducer_blade_height(a) = compressor_exducer_blade_height(a,k);

        maximum_compressor_exducer_blade_height_rpm(a) = rpm(k);
        end
    end


    % Total compressor wheel axial height

    maximum_compressor_total_height(a) = 5 * maximum_compressor_exducer_blade_height(a);


    % Remaining axial blade height

    maximum_compressor_inducer_blade_height(a) = maximum_compressor_total_height(a) - maximum_compressor_exducer_blade_height(a);

end


% Determine overall geometry ranges
%
% NOTE: minimum/maximum_compressor_total_height below used to reassign
% the SAME name as the per-candidate vector computed at line ~805
% (maximum_compressor_total_height(a), one axial height per feasible
% compressor candidate). That collision has been harmless so far because
% nothing after this point indexed it per-candidate -- but 5.4 needs
% exactly that per-candidate vector, and reading it after this
% reassignment would have silently returned the wrong thing (the overall
% max for every candidate, for index 1) rather than erroring outright.
% Renamed the summary scalars so the per-candidate vector survives.

minimum_compressor_blade_height = min(maximum_compressor_exducer_blade_height);

maximum_compressor_blade_height = max(maximum_compressor_exducer_blade_height);

overall_minimum_compressor_total_height = min(maximum_compressor_total_height);

overall_maximum_compressor_total_height = max(maximum_compressor_total_height);

minimum_compressor_inducer_height = min(maximum_compressor_inducer_blade_height);

maximum_compressor_inducer_height = max(maximum_compressor_inducer_blade_height);


fprintf('\n--- 3.4 COMPRESSOR AXIAL GEOMETRY ---\n');

fprintf('Exducer flow coefficient = %.2f\n', compressor_exducer_flow_coefficient);

fprintf('Maximum b2 range = %.2f - %.2f mm\n', minimum_compressor_blade_height * 1000, maximum_compressor_blade_height * 1000);

fprintf('Total wheel height range = %.2f - %.2f mm\n', overall_minimum_compressor_total_height * 1000, overall_maximum_compressor_total_height * 1000);

fprintf('Inducer blade height range = %.2f - %.2f mm\n', minimum_compressor_inducer_height * 1000, maximum_compressor_inducer_height * 1000);

%% 3.4.1 COMPRESSOR EXDUCER GEOMETRY DIAGNOSTICS

fprintf('\n--- COMPRESSOR EXDUCER GEOMETRY DIAGNOSTICS ---\n');

for a = 1:number_of_feasible_compressor_geometries

    maximum_b2 = maximum_compressor_exducer_blade_height(a);

    maximum_b2_rpm = maximum_compressor_exducer_blade_height_rpm(a);

    maximum_b2_index = 1;

    for k = 1:length(rpm)

        if rpm(k) == maximum_b2_rpm

            maximum_b2_index = k;

        end

    end

    current_D2 = feasible_compressor_D2(a);

    current_turbo_speed = feasible_compressor_required_turbo_speed(a,maximum_b2_index);

    current_angular_velocity = current_turbo_speed * pi / 30;

    current_U2 = current_angular_velocity * current_D2 / 2;

    current_Cm2 = compressor_exducer_velocity(a,maximum_b2_index);

    current_A2 = compressor_exducer_area(a,maximum_b2_index);

    current_density = compressor_outlet_density(maximum_b2_index);

    fprintf('Candidate %3d: D2 = %.1f mm, b2 = %.2f mm, RPM = %.0f, U2 = %.1f m/s, Cm2 = %.1f m/s, A2 = %.6e m^2, density = %.3f kg/m^3\n', a, current_D2 * 1000, maximum_b2 * 1000, maximum_b2_rpm, current_U2, current_Cm2, current_A2, current_density);

end

% NOTE: Section 3.5 ("COMPRESSOR SIZING RESULTS") has been removed. It
% referenced compressor_flow_coefficient, compressor_tip_speed,
% compressor_design_feasible, turbo_speed_range, number_of_turbo_speeds
% and number_of_engine_speeds -- leftovers from the earlier
% turbo-speed-grid version of Section 3.1, none of which exist in the
% current D1/D2-candidate search. Its range/table content already
% appears in 3.2 and 3.3; its one non-duplicate item (a count of how
% many turbo speeds were feasible at each engine rpm) doesn't map onto
% the current method, since 3.1 now keeps only the single feasible
% speed per surviving (D1,D2) candidate rather than a full grid of every
% speed tried at every rpm.

%% 4.1 COMPRESSOR TO TURBINE POWER REQUIREMENT

% Number of feasible compressor geometries
number_of_feasible_compressor_geometries = length(feasible_compressor_D1);

% Initialise compressor shaft power requirement
%
% Rows    = compressor geometry candidates
% Columns = engine operating speeds

feasible_compressor_power = NaN(number_of_feasible_compressor_geometries, length(rpm));

% Copy the required compressor power onto every feasible geometry
%
% The compressor power is determined by the engine operating requirement.
% The geometry determines the turbo speed required to produce that work.

for a = 1:number_of_feasible_compressor_geometries

    for k = 1:length(rpm)

        feasible_compressor_power(a,k) = compressor_power_required(k);

    end

end


fprintf('\n--- 4.1 COMPRESSOR SHAFT POWER REQUIREMENT ---\n');

fprintf('Number of feasible compressor geometries = %d\n', number_of_feasible_compressor_geometries);

fprintf('Required compressor shaft power range = %.2f - %.2f kW\n', min(compressor_power_required) / 1000, max(compressor_power_required) / 1000);

%% 4.2 TURBINE INLET STATE

turbine_inlet_temperature = zeros(size(rpm));
turbine_inlet_pressure = zeros(size(rpm));
turbine_inlet_density = zeros(size(rpm));
fuel_energy_rate = zeros(size(rpm));
non_brake_energy_rate = zeros(size(rpm));
exhaust_energy_rate = zeros(size(rpm));
turbine_pressure_ratio = zeros(size(rpm));
turbine_power_required = zeros(size(rpm));
turbine_required_specific_work = zeros(size(rpm));

exhaust_specific_heat = 1150;
exhaust_energy_fraction = 0.5; 
mechanical_efficiency = 0.97;

% Turbine isentropic efficiency [-] -- mirrors compressor_efficiency's
% role on the compressor side; typical radial turbine value.
% NOTE: this and exhaust_gamma below were previously used in the
% pressure-ratio equation at line ~950 without ever being defined
% anywhere in the script -- that's a second "undefined variable" bug of
% the same kind as peak_power_index, just further downstream. Found by
% the full-file static check, not yet hit during a run.
turbine_efficiency = 0.70;

% Ratio of specific heats for exhaust gas [-]. Genuinely lower than air's
% 1.4 -- higher CO2/H2O content and higher temperature both reduce gamma
% for combustion products relative to cold air.
exhaust_gamma = 1.33;

for k = 1:length(rpm)

    fuel_energy_rate(k) = required_fuel_mass_flow(k) * fuel_LHV;

    non_brake_energy_rate(k) = fuel_energy_rate(k) - desired_power(k);

    exhaust_energy_rate(k) = non_brake_energy_rate(k) * exhaust_energy_fraction;

    turbine_inlet_temperature(k) = ambient_temperature +exhaust_energy_rate(k) / (required_exhaust_mass_flow(k) * exhaust_specific_heat);

    turbine_power_required(k) = compressor_power_required(k) / mechanical_efficiency;

    turbine_required_specific_work(k) = turbine_power_required(k)/ required_exhaust_mass_flow(k);

    turbine_pressure_ratio(k) = (1 - turbine_required_specific_work(k) / (turbine_efficiency * exhaust_specific_heat * turbine_inlet_temperature(k)))^(-exhaust_gamma / (exhaust_gamma - 1));

    turbine_inlet_pressure(k) = turbine_pressure_ratio(k) * ambient_pressure;

    turbine_inlet_density(k) = turbine_inlet_pressure(k) / (air_gas_constant * turbine_inlet_temperature(k));

end

fprintf('\n--- 4.2 TURBINE INLET STATE ---\n');

fprintf('Engine speed range = %.0f - %.0f rpm\n', rpm(1), rpm(length(rpm)));

fprintf('Turbine inlet temperature range = %.1f - %.1f K\n', min(turbine_inlet_temperature), max(turbine_inlet_temperature));

fprintf('Turbine inlet pressure range = %.2f - %.2f bar absolute\n', min(turbine_inlet_pressure) / 1e5, max(turbine_inlet_pressure) / 1e5);

fprintf('Turbine pressure ratio range = %.3f - %.3f\n', min(turbine_pressure_ratio), max(turbine_pressure_ratio));

fprintf('Turbine power requirement range = %.2f - %.2f kW\n', min(turbine_power_required) / 1000, max(turbine_power_required) / 1000);

fprintf('Turbine specific work range = %.2f - %.2f kJ/kg\n', min(turbine_required_specific_work) / 1000, max(turbine_required_specific_work) / 1000);

%% 4.3 TURBINE MATCHING REQUIREMENTS

% Initialise arrays for turbine matching requirements

required_turbine_speed = NaN(number_of_feasible_compressor_geometries, length(rpm));

required_turbine_power = NaN(number_of_feasible_compressor_geometries, length(rpm));

required_turbine_work = NaN(number_of_feasible_compressor_geometries, length(rpm));


% Transfer compressor-determined shaft speed and power to turbine

for a = 1:number_of_feasible_compressor_geometries

    for k = 1:length(rpm)

        % Turbine must operate at the same shaft speed as the compressor
        required_turbine_speed(a,k) = feasible_compressor_required_turbo_speed(a,k);

        % Turbine must provide the compressor shaft power
        required_turbine_power(a,k) = feasible_compressor_power(a,k);

        % Specific turbine work required [J/kg]
        required_turbine_work(a,k) = required_turbine_power(a,k) / required_exhaust_mass_flow(k);

    end

end

%% 4.4 TURBINE OPERATING REQUIREMENTS FOR COMPRESSOR CANDIDATES

number_of_feasible_compressor_geometries = length(feasible_compressor_D1);

number_of_engine_speed_points = length(rpm);


% Initialise turbine operating speed matrix [rpm]

turbine_candidate_speed = zeros(number_of_feasible_compressor_geometries, number_of_engine_speed_points);


% Initialise turbine thermodynamic requirement matrices

turbine_candidate_power = zeros(number_of_feasible_compressor_geometries, number_of_engine_speed_points);

turbine_candidate_specific_work = zeros(number_of_feasible_compressor_geometries, number_of_engine_speed_points);

turbine_candidate_mass_flow = zeros(number_of_feasible_compressor_geometries, number_of_engine_speed_points);

turbine_candidate_inlet_temperature = zeros(number_of_feasible_compressor_geometries, number_of_engine_speed_points);

turbine_candidate_inlet_pressure = zeros(number_of_feasible_compressor_geometries, number_of_engine_speed_points);

turbine_candidate_pressure_ratio = zeros(number_of_feasible_compressor_geometries, number_of_engine_speed_points);

turbine_candidate_inlet_density = zeros(number_of_feasible_compressor_geometries, number_of_engine_speed_points);

for c = 1:number_of_feasible_compressor_geometries

    for k = 1:number_of_engine_speed_points

        % Turbocharger shaft speed imposed by compressor candidate [rpm]

        turbine_candidate_speed(c,k) = feasible_compressor_required_turbo_speed(c,k);


        % Turbine power required [W]

        turbine_candidate_power(c,k) = turbine_power_required(k);


        % Turbine specific work required [J/kg]

        turbine_candidate_specific_work(c,k) = turbine_required_specific_work(k);


        % Exhaust mass flow available to turbine [kg/s]

        turbine_candidate_mass_flow(c,k) = required_exhaust_mass_flow(k);


        % Turbine inlet temperature [K]

        turbine_candidate_inlet_temperature(c,k) = turbine_inlet_temperature(k);


        % Turbine inlet pressure [Pa]

        turbine_candidate_inlet_pressure(c,k) = turbine_inlet_pressure(k);


        % Turbine pressure ratio [-]

        turbine_candidate_pressure_ratio(c,k) = turbine_pressure_ratio(k);


        % Turbine candidate density [kg/m^3]

        turbine_candidate_inlet_density(c,k) = turbine_inlet_density(k);

    end

end


fprintf('\n--- 4.4 TURBINE OPERATING REQUIREMENTS ---\n');

fprintf('Number of compressor candidates = %d\n', number_of_feasible_compressor_geometries);

fprintf('Number of engine speed points = %d\n', number_of_engine_speed_points);

fprintf('Turbo speed range across candidates = %.0f - %.0f rpm\n', min(turbine_candidate_speed(:)), max(turbine_candidate_speed(:)));

fprintf('Turbine power requirement range = %.2f - %.2f kW\n', min(turbine_candidate_power(:)) / 1000, max(turbine_candidate_power(:)) / 1000);

fprintf('Turbine specific work range = %.2f - %.2f kJ/kg\n', min(turbine_candidate_specific_work(:)) / 1000, max(turbine_candidate_specific_work(:)) / 1000);

%% 4.5 TURBINE VELOCITY TRIANGLE

turbine_D1_candidates = 0.030:0.002:0.090;

turbine_zero_exit_swirl = true;
turbine_absolute_inlet_angle_candidates = 45:5:75;

number_of_turbine_D1_candidates = length(turbine_D1_candidates);
number_of_turbine_inlet_angle_candidates = length(turbine_absolute_inlet_angle_candidates);

turbine_rotor_speed = zeros(number_of_feasible_compressor_geometries, number_of_turbine_D1_candidates, number_of_turbine_inlet_angle_candidates, number_of_engine_speed_points);
turbine_inlet_tip_speed = zeros(size(turbine_rotor_speed));
turbine_absolute_inlet_velocity = zeros(size(turbine_rotor_speed));
turbine_tangential_inlet_velocity = zeros(size(turbine_rotor_speed));
turbine_meridional_inlet_velocity = zeros(size(turbine_rotor_speed));
turbine_relative_inlet_velocity = zeros(size(turbine_rotor_speed));
turbine_relative_inlet_angle = zeros(size(turbine_rotor_speed));

for c = 1:number_of_feasible_compressor_geometries

    for d = 1:number_of_turbine_D1_candidates

        current_turbine_D1 = turbine_D1_candidates(d);

        for a = 1:number_of_turbine_inlet_angle_candidates

            current_inlet_angle = turbine_absolute_inlet_angle_candidates(a);

            for k = 1:number_of_engine_speed_points

                turbine_rotor_speed(c,d,a,k) = turbine_candidate_speed(c,k) * pi / 30;

                turbine_inlet_tip_speed(c,d,a,k) = turbine_rotor_speed(c,d,a,k) * current_turbine_D1 / 2;

                current_specific_work = turbine_candidate_specific_work(c,k);

                turbine_tangential_inlet_velocity(c,d,a,k) = current_specific_work / turbine_inlet_tip_speed(c,d,a,k);

                turbine_meridional_inlet_velocity(c,d,a,k) = turbine_tangential_inlet_velocity(c,d,a,k) / tand(current_inlet_angle);

                turbine_absolute_inlet_velocity(c,d,a,k) = sqrt(turbine_tangential_inlet_velocity(c,d,a,k)^2 + turbine_meridional_inlet_velocity(c,d,a,k)^2);

                turbine_relative_inlet_velocity(c,d,a,k) = sqrt(turbine_meridional_inlet_velocity(c,d,a,k)^2 + (turbine_tangential_inlet_velocity(c,d,a,k) - turbine_inlet_tip_speed(c,d,a,k))^2);

                turbine_relative_inlet_angle(c,d,a,k) = atan2d(turbine_meridional_inlet_velocity(c,d,a,k), turbine_inlet_tip_speed(c,d,a,k) - turbine_tangential_inlet_velocity(c,d,a,k));

            end
        end
    end
end

fprintf('\n--- 4.5 TURBINE VELOCITY TRIANGLE ---\n');
fprintf('Turbine D1 range = %.1f - %.1f mm\n', min(turbine_D1_candidates) * 1000, max(turbine_D1_candidates) * 1000);
fprintf('Inlet angle range = %.0f - %.0f deg\n', min(turbine_absolute_inlet_angle_candidates), max(turbine_absolute_inlet_angle_candidates));
fprintf('Number of D1 candidates = %d\n', number_of_turbine_D1_candidates);
fprintf('Number of inlet angle candidates = %d\n', number_of_turbine_inlet_angle_candidates);


%% 4.6 TURBINE INDUCER GEOMETRY

turbine_inducer_flow_area = zeros(number_of_feasible_compressor_geometries, number_of_turbine_D1_candidates, number_of_turbine_inlet_angle_candidates, number_of_engine_speed_points);
turbine_inducer_blade_height = zeros(size(turbine_inducer_flow_area));

for c = 1:number_of_feasible_compressor_geometries

    for d = 1:number_of_turbine_D1_candidates

        current_turbine_D1 = turbine_D1_candidates(d);

        for a = 1:number_of_turbine_inlet_angle_candidates

            for k = 1:number_of_engine_speed_points

                current_mass_flow = turbine_candidate_mass_flow(c,k);
                current_density = turbine_candidate_inlet_density(c,k);
                current_Cm1 = turbine_meridional_inlet_velocity(c,d,a,k);

                turbine_inducer_flow_area(c,d,a,k) = current_mass_flow / (current_density * current_Cm1);

                turbine_inducer_blade_height(c,d,a,k) = turbine_inducer_flow_area(c,d,a,k) / (pi * current_turbine_D1);

            end
        end
    end
end

fprintf('\n--- 4.6 TURBINE INDUCER GEOMETRY ---\n');
fprintf('Turbine inlet diameter range = %.1f - %.1f mm\n', min(turbine_D1_candidates) * 1000, max(turbine_D1_candidates) * 1000);
fprintf('Turbine inlet blade height range = %.2f - %.2f mm\n', min(turbine_inducer_blade_height(:)) * 1000, max(turbine_inducer_blade_height(:)) * 1000);
fprintf('Turbine inlet flow area range = %.6f - %.6f m^2\n', min(turbine_inducer_flow_area(:)), max(turbine_inducer_flow_area(:)));


% TURBINE INDUCER BLADE HEIGHT BY INLET ANGLE

fprintf('\n--- 4.6A TURBINE INDUCER BLADE HEIGHT BY INLET ANGLE ---\n');
fprintf('Angle (deg)    Minimum b1 (mm)    Maximum b1 (mm)\n');
fprintf('---------------------------------------------------\n');

for a = 1:number_of_turbine_inlet_angle_candidates

    current_angle = turbine_absolute_inlet_angle_candidates(a);

    current_b1_values = turbine_inducer_blade_height(:,:,a,:);

    minimum_b1 = min(current_b1_values(:)) * 1000;
    maximum_b1 = max(current_b1_values(:)) * 1000;

    fprintf('%8.0f %18.2f %18.2f\n', current_angle, minimum_b1, maximum_b1);

end


% MAXIMUM TURBINE INDUCER BLADE HEIGHT

[maximum_turbine_inducer_blade_height, maximum_b1_index] = max(turbine_inducer_blade_height(:));

[maximum_c, maximum_d, maximum_a, maximum_k] = ind2sub(size(turbine_inducer_blade_height), maximum_b1_index);

fprintf('\n--- 4.6B MAXIMUM TURBINE INDUCER BLADE HEIGHT ---\n');
fprintf('Maximum b1 = %.2f mm\n', maximum_turbine_inducer_blade_height * 1000);
fprintf('Compressor candidate = %d\n', maximum_c);
fprintf('Turbine D1 = %.1f mm\n', turbine_D1_candidates(maximum_d) * 1000);
fprintf('Inlet angle = %.0f deg\n', turbine_absolute_inlet_angle_candidates(maximum_a));
fprintf('Engine speed = %d rpm\n', rpm(maximum_k));
fprintf('Mass flow = %.4f kg/s\n', turbine_candidate_mass_flow(maximum_c,maximum_k));
fprintf('Inlet density = %.3f kg/m^3\n', turbine_candidate_inlet_density(maximum_c,maximum_k));
fprintf('U1 = %.1f m/s\n', turbine_inlet_tip_speed(maximum_c,maximum_d,maximum_a,maximum_k));
fprintf('Ctheta1 = %.1f m/s\n', turbine_tangential_inlet_velocity(maximum_c,maximum_d,maximum_a,maximum_k));
fprintf('Cm1 = %.1f m/s\n', turbine_meridional_inlet_velocity(maximum_c,maximum_d,maximum_a,maximum_k));

% TURBINE INDUCER MACH NUMBER ANALYSIS

turbine_inlet_speed_of_sound = zeros(number_of_feasible_compressor_geometries, number_of_turbine_D1_candidates, number_of_turbine_inlet_angle_candidates, number_of_engine_speed_points);

turbine_absolute_inlet_mach = zeros(size(turbine_inlet_speed_of_sound));
turbine_relative_inlet_mach = zeros(size(turbine_inlet_speed_of_sound));

for c = 1:number_of_feasible_compressor_geometries

    for d = 1:number_of_turbine_D1_candidates

        for a = 1:number_of_turbine_inlet_angle_candidates

            for k = 1:number_of_engine_speed_points

                current_temperature = turbine_candidate_inlet_temperature(c,k);

                turbine_inlet_speed_of_sound(c,d,a,k) = sqrt(exhaust_gamma * air_gas_constant * current_temperature);

                turbine_absolute_inlet_mach(c,d,a,k) = turbine_absolute_inlet_velocity(c,d,a,k) / turbine_inlet_speed_of_sound(c,d,a,k);

                turbine_relative_inlet_mach(c,d,a,k) = turbine_relative_inlet_velocity(c,d,a,k) / turbine_inlet_speed_of_sound(c,d,a,k);

            end
        end
    end
end


% MAXIMUM MACH NUMBER BY INLET ANGLE

fprintf('\n--- 4.6C1 MAXIMUM TURBINE INDUCER MACH NUMBER BY INLET ANGLE ---\n');
fprintf('Angle (deg)    Max absolute Mach    Max relative Mach\n');
fprintf('------------------------------------------------------\n');

for a = 1:number_of_turbine_inlet_angle_candidates

    current_angle = turbine_absolute_inlet_angle_candidates(a);

    current_absolute_mach = turbine_absolute_inlet_mach(:,:,a,:);
    current_relative_mach = turbine_relative_inlet_mach(:,:,a,:);

    maximum_absolute_mach = max(current_absolute_mach(:));
    maximum_relative_mach = max(current_relative_mach(:));

    fprintf('%8.0f %20.3f %20.3f\n', current_angle, maximum_absolute_mach, maximum_relative_mach);

end


% MINIMUM AND MAXIMUM MACH NUMBER

minimum_absolute_mach = min(turbine_absolute_inlet_mach(:));
maximum_absolute_mach = max(turbine_absolute_inlet_mach(:));

minimum_relative_mach = min(turbine_relative_inlet_mach(:));
maximum_relative_mach = max(turbine_relative_inlet_mach(:));

fprintf('\n--- 4.6C2 TURBINE INDUCER MACH NUMBER RANGE ---\n');
fprintf('Absolute inlet Mach range = %.3f - %.3f\n', minimum_absolute_mach, maximum_absolute_mach);
fprintf('Relative inlet Mach range = %.3f - %.3f\n', minimum_relative_mach, maximum_relative_mach);


% MAXIMUM ABSOLUTE INLET MACH

[maximum_absolute_mach_value, maximum_absolute_mach_index] = max(turbine_absolute_inlet_mach(:));

[maximum_c, maximum_d, maximum_a, maximum_k] = ind2sub(size(turbine_absolute_inlet_mach), maximum_absolute_mach_index);

fprintf('\n--- 4.6C3 MAXIMUM ABSOLUTE INLET MACH ---\n');
fprintf('Maximum absolute Mach = %.3f\n', maximum_absolute_mach_value);
fprintf('Compressor candidate = %d\n', maximum_c);
fprintf('Turbine D1 = %.1f mm\n', turbine_D1_candidates(maximum_d) * 1000);
fprintf('Inlet angle = %.0f deg\n', turbine_absolute_inlet_angle_candidates(maximum_a));
fprintf('Engine speed = %d rpm\n', rpm(maximum_k));
fprintf('Absolute inlet velocity = %.1f m/s\n', turbine_absolute_inlet_velocity(maximum_c,maximum_d,maximum_a,maximum_k));
fprintf('Speed of sound = %.1f m/s\n', turbine_inlet_speed_of_sound(maximum_c,maximum_d,maximum_a,maximum_k));


% MAXIMUM RELATIVE INLET MACH

[maximum_relative_mach_value, maximum_relative_mach_index] = max(turbine_relative_inlet_mach(:));

[maximum_c, maximum_d, maximum_a, maximum_k] = ind2sub(size(turbine_relative_inlet_mach), maximum_relative_mach_index);

fprintf('\n--- 4.6C4 MAXIMUM RELATIVE INLET MACH ---\n');
fprintf('Maximum relative Mach = %.3f\n', maximum_relative_mach_value);
fprintf('Compressor candidate = %d\n', maximum_c);
fprintf('Turbine D1 = %.1f mm\n', turbine_D1_candidates(maximum_d) * 1000);
fprintf('Inlet angle = %.0f deg\n', turbine_absolute_inlet_angle_candidates(maximum_a));
fprintf('Engine speed = %d rpm\n', rpm(maximum_k));
fprintf('Relative inlet velocity = %.1f m/s\n', turbine_relative_inlet_velocity(maximum_c,maximum_d,maximum_a,maximum_k));
fprintf('Speed of sound = %.1f m/s\n', turbine_inlet_speed_of_sound(maximum_c,maximum_d,maximum_a,maximum_k));

% TURBINE INDUCER VELOCITY DIAGNOSTIC

fprintf('\n--- 4.6D TURBINE INDUCER VELOCITY DIAGNOSTIC ---\n');


% MAXIMUM ABSOLUTE MACH DIAGNOSTIC

[maximum_absolute_mach_value, maximum_absolute_mach_index] = max(turbine_absolute_inlet_mach(:));

[diagnostic_c, diagnostic_d, diagnostic_a, diagnostic_k] = ind2sub(size(turbine_absolute_inlet_mach), maximum_absolute_mach_index);

diagnostic_D1 = turbine_D1_candidates(diagnostic_d);
diagnostic_angle = turbine_absolute_inlet_angle_candidates(diagnostic_a);
diagnostic_rpm = rpm(diagnostic_k);

diagnostic_turbine_speed = turbine_candidate_speed(diagnostic_c,diagnostic_k);
diagnostic_specific_work = turbine_candidate_specific_work(diagnostic_c,diagnostic_k);
diagnostic_mass_flow = turbine_candidate_mass_flow(diagnostic_c,diagnostic_k);
diagnostic_density = turbine_candidate_inlet_density(diagnostic_c,diagnostic_k);
diagnostic_temperature = turbine_candidate_inlet_temperature(diagnostic_c,diagnostic_k);

diagnostic_U1 = turbine_inlet_tip_speed(diagnostic_c,diagnostic_d,diagnostic_a,diagnostic_k);
diagnostic_Ctheta1 = turbine_tangential_inlet_velocity(diagnostic_c,diagnostic_d,diagnostic_a,diagnostic_k);
diagnostic_Cm1 = turbine_meridional_inlet_velocity(diagnostic_c,diagnostic_d,diagnostic_a,diagnostic_k);
diagnostic_C1 = turbine_absolute_inlet_velocity(diagnostic_c,diagnostic_d,diagnostic_a,diagnostic_k);

diagnostic_W1 = turbine_relative_inlet_velocity(diagnostic_c,diagnostic_d,diagnostic_a,diagnostic_k);
diagnostic_relative_angle = turbine_relative_inlet_angle(diagnostic_c,diagnostic_d,diagnostic_a,diagnostic_k);

diagnostic_speed_of_sound = turbine_inlet_speed_of_sound(diagnostic_c,diagnostic_d,diagnostic_a,diagnostic_k);
diagnostic_absolute_mach = turbine_absolute_inlet_mach(diagnostic_c,diagnostic_d,diagnostic_a,diagnostic_k);
diagnostic_relative_mach = turbine_relative_inlet_mach(diagnostic_c,diagnostic_d,diagnostic_a,diagnostic_k);

diagnostic_flow_area = turbine_inducer_flow_area(diagnostic_c,diagnostic_d,diagnostic_a,diagnostic_k);
diagnostic_b1 = turbine_inducer_blade_height(diagnostic_c,diagnostic_d,diagnostic_a,diagnostic_k);


fprintf('\nMAXIMUM ABSOLUTE MACH CASE\n');
fprintf('Compressor candidate = %d\n', diagnostic_c);
fprintf('Engine speed = %d rpm\n', diagnostic_rpm);
fprintf('Turbine D1 = %.1f mm\n', diagnostic_D1 * 1000);
fprintf('Inlet angle = %.0f deg\n', diagnostic_angle);
fprintf('Turbine speed = %.0f rpm\n', diagnostic_turbine_speed);
fprintf('Turbine specific work = %.1f kJ/kg\n', diagnostic_specific_work / 1000);
fprintf('Mass flow = %.4f kg/s\n', diagnostic_mass_flow);
fprintf('Inlet density = %.3f kg/m^3\n', diagnostic_density);
fprintf('Inlet temperature = %.1f K\n', diagnostic_temperature);
fprintf('Speed of sound = %.1f m/s\n', diagnostic_speed_of_sound);
fprintf('U1 = %.1f m/s\n', diagnostic_U1);
fprintf('Ctheta1 = %.1f m/s\n', diagnostic_Ctheta1);
fprintf('Cm1 = %.1f m/s\n', diagnostic_Cm1);
fprintf('Absolute velocity C1 = %.1f m/s\n', diagnostic_C1);
fprintf('Relative velocity W1 = %.1f m/s\n', diagnostic_W1);
fprintf('Relative inlet angle = %.1f deg\n', diagnostic_relative_angle);
fprintf('Absolute inlet Mach = %.3f\n', diagnostic_absolute_mach);
fprintf('Relative inlet Mach = %.3f\n', diagnostic_relative_mach);
fprintf('Inducer flow area = %.6f m^2\n', diagnostic_flow_area);
fprintf('Inducer blade height b1 = %.2f mm\n', diagnostic_b1 * 1000);


% MAXIMUM RELATIVE MACH DIAGNOSTIC

[maximum_relative_mach_value, maximum_relative_mach_index] = max(turbine_relative_inlet_mach(:));

[diagnostic_c, diagnostic_d, diagnostic_a, diagnostic_k] = ind2sub(size(turbine_relative_inlet_mach), maximum_relative_mach_index);

diagnostic_D1 = turbine_D1_candidates(diagnostic_d);
diagnostic_angle = turbine_absolute_inlet_angle_candidates(diagnostic_a);
diagnostic_rpm = rpm(diagnostic_k);

diagnostic_turbine_speed = turbine_candidate_speed(diagnostic_c,diagnostic_k);
diagnostic_specific_work = turbine_candidate_specific_work(diagnostic_c,diagnostic_k);
diagnostic_mass_flow = turbine_candidate_mass_flow(diagnostic_c,diagnostic_k);
diagnostic_density = turbine_candidate_inlet_density(diagnostic_c,diagnostic_k);
diagnostic_temperature = turbine_candidate_inlet_temperature(diagnostic_c,diagnostic_k);

diagnostic_U1 = turbine_inlet_tip_speed(diagnostic_c,diagnostic_d,diagnostic_a,diagnostic_k);
diagnostic_Ctheta1 = turbine_tangential_inlet_velocity(diagnostic_c,diagnostic_d,diagnostic_a,diagnostic_k);
diagnostic_Cm1 = turbine_meridional_inlet_velocity(diagnostic_c,diagnostic_d,diagnostic_a,diagnostic_k);
diagnostic_C1 = turbine_absolute_inlet_velocity(diagnostic_c,diagnostic_d,diagnostic_a,diagnostic_k);

diagnostic_W1 = turbine_relative_inlet_velocity(diagnostic_c,diagnostic_d,diagnostic_a,diagnostic_k);
diagnostic_relative_angle = turbine_relative_inlet_angle(diagnostic_c,diagnostic_d,diagnostic_a,diagnostic_k);

diagnostic_speed_of_sound = turbine_inlet_speed_of_sound(diagnostic_c,diagnostic_d,diagnostic_a,diagnostic_k);
diagnostic_absolute_mach = turbine_absolute_inlet_mach(diagnostic_c,diagnostic_d,diagnostic_a,diagnostic_k);
diagnostic_relative_mach = turbine_relative_inlet_mach(diagnostic_c,diagnostic_d,diagnostic_a,diagnostic_k);

diagnostic_flow_area = turbine_inducer_flow_area(diagnostic_c,diagnostic_d,diagnostic_a,diagnostic_k);
diagnostic_b1 = turbine_inducer_blade_height(diagnostic_c,diagnostic_d,diagnostic_a,diagnostic_k);


fprintf('\nMAXIMUM RELATIVE MACH CASE\n');
fprintf('Compressor candidate = %d\n', diagnostic_c);
fprintf('Engine speed = %d rpm\n', diagnostic_rpm);
fprintf('Turbine D1 = %.1f mm\n', diagnostic_D1 * 1000);
fprintf('Inlet angle = %.0f deg\n', diagnostic_angle);
fprintf('Turbine speed = %.0f rpm\n', diagnostic_turbine_speed);
fprintf('Turbine specific work = %.1f kJ/kg\n', diagnostic_specific_work / 1000);
fprintf('Mass flow = %.4f kg/s\n', diagnostic_mass_flow);
fprintf('Inlet density = %.3f kg/m^3\n', diagnostic_density);
fprintf('Inlet temperature = %.1f K\n', diagnostic_temperature);
fprintf('Speed of sound = %.1f m/s\n', diagnostic_speed_of_sound);
fprintf('U1 = %.1f m/s\n', diagnostic_U1);
fprintf('Ctheta1 = %.1f m/s\n', diagnostic_Ctheta1);
fprintf('Cm1 = %.1f m/s\n', diagnostic_Cm1);
fprintf('Absolute velocity C1 = %.1f m/s\n', diagnostic_C1);
fprintf('Relative velocity W1 = %.1f m/s\n', diagnostic_W1);
fprintf('Relative inlet angle = %.1f deg\n', diagnostic_relative_angle);
fprintf('Absolute inlet Mach = %.3f\n', diagnostic_absolute_mach);
fprintf('Relative inlet Mach = %.3f\n', diagnostic_relative_mach);
fprintf('Inducer flow area = %.6f m^2\n', diagnostic_flow_area);
fprintf('Inducer blade height b1 = %.2f mm\n', diagnostic_b1 * 1000);

%% 4.7 TURBINE EXDUCER GEOMETRY

turbine_exducer_D2_candidates = 0.016:0.002:0.100;

turbine_exducer_hub_to_tip_ratio = 0.50;

number_of_turbine_exducer_D2_candidates = length(turbine_exducer_D2_candidates);

% NOTE: the previous version of this section preallocated six 5-D arrays
% sized (compressor candidates x D1 candidates x inlet angles x D2
% candidates x rpm points) = ~87 million elements EACH (~4 GB total).
% Nothing downstream ever read from them -- sections 4.7B onward
% recompute everything they need using the much smaller 2-D/3-D arrays
% below. That block has been removed; this is the fix for the "4.7
% takes forever / nothing returns" issue.

% 4.7B TURBINE EXDUCER DIAMETER CANDIDATES

fprintf('\n--- 4.7B TURBINE EXDUCER DIAMETER CANDIDATES ---\n');
fprintf('D2 (mm)    Hub D2 (mm)    Blade width (mm)    Annular area (m^2)\n');
fprintf('----------------------------------------------------------------\n');

for e = 1:number_of_turbine_exducer_D2_candidates

    current_D2 = turbine_exducer_D2_candidates(e);

    if current_D2 >= max(turbine_D1_candidates)
        continue
    end

    current_hub_D2 = turbine_exducer_hub_to_tip_ratio * current_D2;
    current_area = pi / 4 * (current_D2^2 - current_hub_D2^2);
    current_blade_width = current_D2 - current_hub_D2;

    fprintf('%7.1f %15.1f %19.1f %20.6f\n', current_D2 * 1000, current_hub_D2 * 1000, current_blade_width * 1000, current_area);

end

% 4.7C TURBINE EXDUCER MACH NUMBER

number_of_turbine_exducer_D2_candidates = length(turbine_exducer_D2_candidates);
number_of_engine_speed_points = length(rpm);

turbine_exducer_exit_temperature_2D = zeros(1, number_of_engine_speed_points);
turbine_exducer_exit_density_2D = zeros(1, number_of_engine_speed_points);
turbine_exducer_speed_of_sound = zeros(1, number_of_engine_speed_points);

turbine_exducer_meridional_velocity_2D = zeros(number_of_turbine_exducer_D2_candidates, number_of_engine_speed_points);
turbine_exducer_meridional_mach = zeros(number_of_turbine_exducer_D2_candidates, number_of_engine_speed_points);

for k = 1:number_of_engine_speed_points

    turbine_exducer_exit_temperature_2D(k) = turbine_inlet_temperature(k) - turbine_required_specific_work(k) / exhaust_specific_heat;

    turbine_exducer_exit_density_2D(k) = ambient_pressure / (air_gas_constant * turbine_exducer_exit_temperature_2D(k));

    turbine_exducer_speed_of_sound(k) = sqrt(exhaust_gamma * air_gas_constant * turbine_exducer_exit_temperature_2D(k));

    for e = 1:number_of_turbine_exducer_D2_candidates

        current_D2 = turbine_exducer_D2_candidates(e);
        current_hub_D2 = turbine_exducer_hub_to_tip_ratio * current_D2;

        current_exducer_area = pi / 4 * (current_D2^2 - current_hub_D2^2);

        turbine_exducer_meridional_velocity_2D(e,k) = required_exhaust_mass_flow(k) / (turbine_exducer_exit_density_2D(k) * current_exducer_area);

        turbine_exducer_meridional_mach(e,k) = turbine_exducer_meridional_velocity_2D(e,k) / turbine_exducer_speed_of_sound(k);

    end
end

valid_exducer_mach = turbine_exducer_meridional_mach(turbine_exducer_meridional_mach > 0);

minimum_exducer_mach = min(valid_exducer_mach);
maximum_exducer_mach = max(valid_exducer_mach);

fprintf('\n--- 4.7C TURBINE EXDUCER MACH NUMBER ---\n');
fprintf('Turbine Exducer Meridional Mach Range = %.3f - %.3f\n', minimum_exducer_mach, maximum_exducer_mach);

% TURBINE EXDUCER MACH NUMBER SUMMARY

maximum_exducer_mach_by_D2 = zeros(1, number_of_turbine_exducer_D2_candidates);

for e = 1:number_of_turbine_exducer_D2_candidates

    maximum_exducer_mach_by_D2(e) = max(turbine_exducer_meridional_mach(e,:));

end

fprintf('\n--- 4.7C TURBINE EXDUCER MACH BY D2 ---\n');
fprintf('D2 (mm)    Maximum Exducer Mach\n');
fprintf('--------------------------------\n');

for e = 1:number_of_turbine_exducer_D2_candidates

    fprintf('%6.1f       %8.3f\n', turbine_exducer_D2_candidates(e) * 1000, maximum_exducer_mach_by_D2(e));

end

%% 4.7D TURBINE EXDUCER MEAN RADIUS AND RELATIVE MACH NUMBER

turbine_exducer_mean_diameter = zeros(1, number_of_turbine_exducer_D2_candidates);
turbine_exducer_mean_radius = zeros(1, number_of_turbine_exducer_D2_candidates);

for e = 1:number_of_turbine_exducer_D2_candidates

    current_D2 = turbine_exducer_D2_candidates(e);
    current_hub_D2 = turbine_exducer_hub_to_tip_ratio * current_D2;

    turbine_exducer_mean_diameter(e) = (current_D2 + current_hub_D2) / 2;

    turbine_exducer_mean_radius(e) = turbine_exducer_mean_diameter(e) / 2;

end

turbine_exducer_mean_blade_speed = zeros(number_of_feasible_compressor_geometries, number_of_turbine_exducer_D2_candidates, number_of_engine_speed_points);
turbine_exducer_relative_velocity = zeros(number_of_feasible_compressor_geometries, number_of_turbine_exducer_D2_candidates, number_of_engine_speed_points);
turbine_exducer_relative_mach = zeros(number_of_feasible_compressor_geometries, number_of_turbine_exducer_D2_candidates, number_of_engine_speed_points);

for c = 1:number_of_feasible_compressor_geometries

    for k = 1:number_of_engine_speed_points

        turbine_exducer_rotor_speed = feasible_compressor_required_turbo_speed(c,k) * pi / 30;

        for e = 1:number_of_turbine_exducer_D2_candidates

            turbine_exducer_mean_blade_speed(c,e,k) = turbine_exducer_rotor_speed * turbine_exducer_mean_radius(e);

            turbine_exducer_relative_velocity(c,e,k) = sqrt(turbine_exducer_meridional_velocity_2D(e,k)^2 + turbine_exducer_mean_blade_speed(c,e,k)^2);

            turbine_exducer_relative_mach(c,e,k) = turbine_exducer_relative_velocity(c,e,k) / turbine_exducer_speed_of_sound(k);

        end

    end

end

maximum_exducer_relative_mach_by_D2 = zeros(1, number_of_turbine_exducer_D2_candidates);
maximum_exducer_relative_mach_compressor_candidate = zeros(1, number_of_turbine_exducer_D2_candidates);

for e = 1:number_of_turbine_exducer_D2_candidates

    current_maximum_mach = -Inf;
    current_compressor_candidate = 0;

    for c = 1:number_of_feasible_compressor_geometries

        current_candidate_maximum_mach = max(turbine_exducer_relative_mach(c,e,:));

        if current_candidate_maximum_mach > current_maximum_mach

            current_maximum_mach = current_candidate_maximum_mach;
            current_compressor_candidate = c;

        end

    end

    maximum_exducer_relative_mach_by_D2(e) = current_maximum_mach;

    maximum_exducer_relative_mach_compressor_candidate(e) = current_compressor_candidate;

end

fprintf('\n');
fprintf('4.7D TURBINE EXDUCER MEAN-LINE RELATIVE MACH NUMBER\n');
fprintf('----------------------------------------------------\n');
fprintf('D2 (mm)    Mean Diameter (mm)    Maximum Relative Mach\n');

for e = 1:number_of_turbine_exducer_D2_candidates

    fprintf('%7.1f    %17.1f    %21.3f\n', ...
        turbine_exducer_D2_candidates(e) * 1000, ...
        turbine_exducer_mean_diameter(e) * 1000, ...
        maximum_exducer_relative_mach_by_D2(e));

end

% NOTE: this section previously also contained a "4.7E TURBINE EXDUCER
% MACH COMPONENTS" block that referenced turbine_exducer_tip_speed(e,k),
% a variable never defined anywhere in the script. On a clean run (clear
% workspace) this throws "Undefined function or variable". It only
% appeared to work when run section-by-section because a stale variable
% of that name was left over in the workspace from earlier debugging.
% Its outputs were never used downstream, so it has been removed rather
% than patched.

%% 4.7F MAXIMUM EXDUCER RELATIVE MACH OPERATING POINT

fprintf('\n--- 4.7F MAXIMUM RELATIVE MACH OPERATING POINT ---\n');

% turbine_exducer_relative_mach is a 3-D array (compressor candidate c,
% D2 candidate e, engine speed k). The previous version indexed it as
% turbine_exducer_relative_mach(e,:), which MATLAB treats as a 2-D
% index -- it collapses dimensions 2 and 3 together, so the returned
% linear index could exceed length(rpm)=86, which is exactly the
% "index must not exceed 86" error. The fix below reuses the (c,e) pair
% that 4.7D already correctly identified as the worst case for each D2,
% and pulls out its rpm-indexed Mach profile with squeeze().

for e = 1:number_of_turbine_exducer_D2_candidates

    c_best = maximum_exducer_relative_mach_compressor_candidate(e);

    if c_best == 0
        fprintf('D2 = %5.1f mm | No feasible compressor candidate at this D2\n', turbine_exducer_D2_candidates(e) * 1000);
        continue
    end

    mach_profile_e = squeeze(turbine_exducer_relative_mach(c_best, e, :));

    [current_max_mach, current_index] = max(mach_profile_e);

    fprintf('D2 = %5.1f mm | Max Mrel = %6.3f | Engine RPM = %5.0f | Compressor candidate #%d\n', ...
        turbine_exducer_D2_candidates(e) * 1000, current_max_mach, rpm(current_index), c_best);

end

%% 4.7G TURBINE INDUCER / EXDUCER GEOMETRIC COMPATIBILITY

number_of_turbine_D1_candidates = length(turbine_D1_candidates);
number_of_turbine_exducer_D2_candidates = length(turbine_exducer_D2_candidates);

turbine_D1_D2_geometrically_feasible = false(number_of_turbine_D1_candidates, number_of_turbine_exducer_D2_candidates);

for d1 = 1:number_of_turbine_D1_candidates

    for d2 = 1:number_of_turbine_exducer_D2_candidates

        current_D1 = turbine_D1_candidates(d1);
        current_D2 = turbine_exducer_D2_candidates(d2);

        if current_D2 < current_D1

            turbine_D1_D2_geometrically_feasible(d1,d2) = true;

        end
    end
end

fprintf('\n--- 4.7E TURBINE D1 / D2 GEOMETRIC COMPATIBILITY ---\n');

fprintf('D1/D2');

for d2 = 1:number_of_turbine_exducer_D2_candidates
    fprintf('%8.1f', turbine_exducer_D2_candidates(d2) * 1000);
end

fprintf('\n');

for d1 = 1:number_of_turbine_D1_candidates

    fprintf('%5.1f', turbine_D1_candidates(d1) * 1000);

    for d2 = 1:number_of_turbine_exducer_D2_candidates

        if turbine_D1_D2_geometrically_feasible(d1,d2)

            fprintf('%8s', 'PASS');

        else

            fprintf('%8s', 'FAIL');

        end
    end

    fprintf('\n');

end

maximum_geometrically_allowed_D2 = zeros(1, number_of_turbine_D1_candidates);

for d1 = 1:number_of_turbine_D1_candidates

    maximum_geometrically_allowed_D2(d1) = 0;

    for d2 = 1:number_of_turbine_exducer_D2_candidates

        if turbine_D1_D2_geometrically_feasible(d1,d2)

            maximum_geometrically_allowed_D2(d1) = turbine_exducer_D2_candidates(d2);

        end
    end
end

fprintf('\n--- 4.7E MAXIMUM D2 FOR EACH D1 ---\n');
fprintf('D1 (mm)    Maximum compatible D2 (mm)\n');
fprintf('--------------------------------------\n');

for d1 = 1:number_of_turbine_D1_candidates

    fprintf('%6.1f             %8.1f\n', turbine_D1_candidates(d1) * 1000, maximum_geometrically_allowed_D2(d1) * 1000);

end

%% 4.7H TURBINE D1/D2 GEOMETRIC COMPATIBILITY

number_of_turbine_D1_candidates = length(turbine_D1_candidates);
number_of_turbine_exducer_D2_candidates = length(turbine_exducer_D2_candidates);

turbine_D1_D2_geometrically_feasible = false(number_of_turbine_D1_candidates, number_of_turbine_exducer_D2_candidates);

for d1 = 1:number_of_turbine_D1_candidates

    current_D1 = turbine_D1_candidates(d1);

    for d2 = 1:number_of_turbine_exducer_D2_candidates

        current_D2 = turbine_exducer_D2_candidates(d2);

        if current_D2 < current_D1
            turbine_D1_D2_geometrically_feasible(d1,d2) = true;
        end

    end
end


% DISPLAY D1/D2 COMPATIBILITY

fprintf('\n');
fprintf('4.7E TURBINE D1/D2 GEOMETRIC COMPATIBILITY\n');
fprintf('-------------------------------------------\n');
fprintf('PASS = D2 < D1\n');
fprintf('FAIL = D2 >= D1\n\n');

fprintf('D1/D2');

for d2 = 1:number_of_turbine_exducer_D2_candidates
    fprintf('%8.0f', turbine_exducer_D2_candidates(d2) * 1000);
end

fprintf('\n');

for d1 = 1:number_of_turbine_D1_candidates

    fprintf('%5.0f', turbine_D1_candidates(d1) * 1000);

    for d2 = 1:number_of_turbine_exducer_D2_candidates

        if turbine_D1_D2_geometrically_feasible(d1,d2)
            fprintf('%8s', 'PASS');
        else
            fprintf('%8s', 'FAIL');
        end

    end

    fprintf('\n');

end

%% 4.7I COMBINED TURBINE FEASIBILITY

number_of_turbine_D1_candidates = length(turbine_D1_candidates);
number_of_turbine_D2_candidates = length(turbine_exducer_D2_candidates);

turbine_combined_feasibility = false(number_of_turbine_D1_candidates, number_of_turbine_D2_candidates);

for d1 = 1:number_of_turbine_D1_candidates

    for d2 = 1:number_of_turbine_D2_candidates

        if turbine_D1_D2_geometrically_feasible(d1,d2)

            if maximum_exducer_relative_mach_by_D2(d2) < 1.0

                turbine_combined_feasibility(d1,d2) = true;

            end

        end

    end

end


% COUNT FEASIBLE TURBINE GEOMETRIES

number_of_feasible_turbine_geometries = sum(turbine_combined_feasibility(:));


fprintf('\n');
fprintf('4.7F COMBINED TURBINE FEASIBILITY\n');
fprintf('---------------------------------\n');
fprintf('Geometry criterion: D2 < D1\n');
fprintf('Aerodynamic criterion: Maximum mean-line Mrel,2 < 1.0\n\n');

fprintf('Number of feasible D1/D2 combinations: %d\n', number_of_feasible_turbine_geometries);

%% 5.0 TURBINE GEOMETRY AND FEASIBILITY

number_of_turbine_D1_candidates = length(turbine_D1_candidates);
number_of_turbine_exducer_D2_candidates = length(turbine_exducer_D2_candidates);

maximum_exducer_relative_mach_limit = 1.0;
maximum_inducer_relative_mach = 1.0;

% CORRECTED: previously, the compressor candidate paired with each D2
% was pre-selected in Section 4.7D by "worst-case exducer Mach" BEFORE
% this search even started -- so every final turbine geometry inherited
% whichever single compressor candidate happened to be worst-case,
% regardless of whether that compressor was actually a good choice for
% the ASSEMBLY's total inertia. That has nothing to do with what this
% project is optimising for (fastest spool while meeting the power
% curve), and it collapsed all 108 feasible compressor candidates down
% to just 1 in the final results.
%
% This search now varies the compressor candidate (c) as a genuine
% third free dimension alongside D1 and D2, exactly like D1 and D2
% already are. Every (c, D1, D2) combination that's aerodynamically
% feasible gets kept here -- the actual "which is best" choice belongs
% in 5.8 (total inertia), where it can be made on genuine inertia
% comparisons across real compressor variety, not a Mach-based
% pre-selection that was never about mass or inertia.
%
% turbine_exducer_relative_mach(c,d2,:) already exists for every
% compressor candidate, not just the previous worst-case pick, so this
% needs no new computation -- just searching data that was already
% there.

feasible_turbine_D1 = [];
feasible_turbine_D2 = [];
feasible_turbine_compressor_candidate = [];
feasible_turbine_angle = [];
feasible_turbine_b1 = [];

for c = 1:number_of_feasible_compressor_geometries

    for d2 = 1:number_of_turbine_exducer_D2_candidates

        % Exducer feasibility for THIS specific compressor candidate,
        % across the full rpm range -- vectorised, no worst-case lookup
        exducer_mach_profile = squeeze(turbine_exducer_relative_mach(c, d2, :));

        if ~all(exducer_mach_profile < maximum_exducer_relative_mach_limit)
            continue
        end

        for d1 = 1:number_of_turbine_D1_candidates

            current_D1 = turbine_D1_candidates(d1);

            if ~turbine_D1_D2_geometrically_feasible(d1,d2)
                continue
            end

            % Inlet angle: same as before, a fixed design choice per
            % (c,D1,D2), tested across the full rpm range, surviving
            % angle chosen by minimum resulting blade height
            best_angle_index = 0;
            best_b1 = Inf;

            for a = 1:number_of_turbine_inlet_angle_candidates

                relative_mach_profile = squeeze(turbine_relative_inlet_mach(c, d1, a, :));

                if ~all(relative_mach_profile < maximum_inducer_relative_mach)
                    continue
                end

                worst_b1_this_angle = max(squeeze(turbine_inducer_blade_height(c, d1, a, :)));

                if worst_b1_this_angle < best_b1
                    best_angle_index = a;
                    best_b1 = worst_b1_this_angle;
                end

            end

            if best_angle_index > 0

                feasible_turbine_D1 = [feasible_turbine_D1 current_D1];
                feasible_turbine_D2 = [feasible_turbine_D2 turbine_exducer_D2_candidates(d2)];
                feasible_turbine_compressor_candidate = [feasible_turbine_compressor_candidate c];
                feasible_turbine_angle = [feasible_turbine_angle turbine_absolute_inlet_angle_candidates(best_angle_index)];
                feasible_turbine_b1 = [feasible_turbine_b1 best_b1];

            end

        end

    end

end

number_of_feasible_turbine_geometries = length(feasible_turbine_D1);

fprintf('\n');
fprintf('5.0 TURBINE FEASIBLE GEOMETRIES\n');
fprintf('--------------------------------\n');

fprintf('Geometry criterion: D2 < D1\n');
fprintf('Aerodynamic criteria: Mrel,2 < 1.0 (exducer); Mrel,1 < 1.0 (inducer) -- both checked across the full rpm range, independently for every compressor candidate (not pre-selected by worst-case)\n\n');

if number_of_feasible_turbine_geometries == 0
    error('No turbine geometries survive both exducer and inducer feasibility across the full rpm range at any inlet angle, for any compressor candidate -- widen candidate ranges or review constraints.');
end

fprintf('Number of feasible turbine geometries: %d\n', number_of_feasible_turbine_geometries);

fprintf('D1 range: %.1f to %.1f mm\n', min(feasible_turbine_D1) * 1000, max(feasible_turbine_D1) * 1000);

fprintf('D2 range: %.1f to %.1f mm\n', min(feasible_turbine_D2) * 1000, max(feasible_turbine_D2) * 1000);

fprintf('Committed inlet angle range: %.0f to %.0f deg\n', min(feasible_turbine_angle), max(feasible_turbine_angle));

unique_compressor_candidates = unique(feasible_turbine_compressor_candidate);
fprintf('Number of DISTINCT compressor candidates genuinely used: %d (out of %d total feasible compressor designs from 3.2)\n', length(unique_compressor_candidates), number_of_feasible_compressor_geometries);
if length(unique_compressor_candidates) <= 10
    fprintf('Distinct compressor candidate indices used: ');
    fprintf('%d ', unique_compressor_candidates);
    fprintf('\n');
end

%% 5.1 TURBINE MERIDIONAL SILHOUETTE

number_of_silhouette_points = 101;

turbine_silhouette_x = zeros(number_of_feasible_turbine_geometries, number_of_silhouette_points);
turbine_silhouette_outer_radius = zeros(number_of_feasible_turbine_geometries, number_of_silhouette_points);
turbine_silhouette_inner_radius = zeros(number_of_feasible_turbine_geometries, number_of_silhouette_points);

for c = 1:number_of_feasible_turbine_geometries

    D1 = feasible_turbine_D1(c);
    D2 = feasible_turbine_D2(c);

    % b1 (worst-case axial inducer blade height across the full rpm
    % range, at this geometry's own committed D1 and inlet angle) was
    % already determined in 5.0 alongside the angle selection itself --
    % no re-search needed here, and no more maxing across unrelated D1
    % or angle candidates the way earlier versions of this section did.
    b1 = feasible_turbine_b1(c);

    % CORRECTED: total axial wheel height, not b1 alone. An earlier
    % version of this project used H = 3*b1 for the turbine's total
    % axial height (inducer blade height is only part of the wheel's
    % full length) -- that multiplier was dropped when this silhouette
    % was rebuilt, understating the wheel's true axial extent by ~3x,
    % which is almost certainly the main reason turbine mass came out
    % too low. Restoring it here, consistent with the compressor's own
    % still-standing 5*b2 convention from Section 3.4.
    total_height = 3 * b1;

    Dh2 = turbine_exducer_hub_to_tip_ratio * D2;

    r1_outer = D1 / 2;
    r2_outer = D2 / 2;

    r1_inner = 0.25 * D1;
    r2_inner = Dh2 / 2;

    for i = 1:number_of_silhouette_points

        xi = (i - 1) / (number_of_silhouette_points - 1);

        turbine_silhouette_x(c,i) = xi * total_height;

        curve_factor = 3 * xi^2 - 2 * xi^3;

        turbine_silhouette_outer_radius(c,i) = r1_outer + (r2_outer - r1_outer) * curve_factor;

        turbine_silhouette_inner_radius(c,i) = r1_inner + (r2_inner - r1_inner) * curve_factor;

    end

end

fprintf('\n');
fprintf('5.1 TURBINE MERIDIONAL SILHOUETTE\n');
fprintf('----------------------------------\n');

fprintf('Number of turbine geometries: %d\n', number_of_feasible_turbine_geometries);
fprintf('Silhouette points per geometry: %d\n', number_of_silhouette_points);

fprintf('Outer radius range: %.1f to %.1f mm\n', min(turbine_silhouette_outer_radius(:)) * 1000, max(turbine_silhouette_outer_radius(:)) * 1000);

fprintf('Inner radius range: %.1f to %.1f mm\n', min(turbine_silhouette_inner_radius(:)) * 1000, max(turbine_silhouette_inner_radius(:)) * 1000);

%% 5.2 TURBINE VOLUME AND MASS

% The turbine wheel is modelled as two parts:
%  - a solid hub/backdisk, bounded by the INNER (hub-line) meridional
%    profile from 5.1 -- a solid body of revolution, carries most of
%    the rotor's mass
%  - blade material, now derived directly from blade count and
%    thickness (per your request) rather than an assumed fill fraction
%    of the annular envelope. Each blade is treated as a thin radial
%    plate: circumferential thickness t_blade, spanning the full
%    radial gap (r_outer - r_inner) at each axial slice, N_blades of
%    them arranged around the wheel. This needs no "% filled" guess at
%    all -- volume follows directly from N_blades * t_blade * radial
%    width * axial length, summed over slices.
%
% N_blades = 10, t_blade = 1.5 mm are carried over from typical values
% used earlier in this project's own history (a reasonable count/
% thickness for a small automotive radial turbine wheel) -- if you have
% a real spec sheet for a reference turbo (you have compressor map data
% uploaded already), swap these for its actual blade count/thickness
% and this becomes fully non-arbitrary.
%
% NOTE on material density: earlier versions of this project used
% 7800 kg/m^3 (a generic steel-like value) for BOTH the compressor and
% turbine wheels. That's wrong for the turbine specifically -- it sees
% exhaust gas at 900+ degC (Section 4.2), which needs a nickel
% superalloy, not steel. Using 8190 kg/m^3 (Inconel 713C-class, a
% common real turbine wheel material) here. The compressor side should
% actually use an aluminium value (~2770 kg/m^3) instead of 7800 when
% we get to 5.5 -- flagging now so it isn't missed.

turbine_material_density = 8190;   % kg/m^3, Inconel 713C-class superalloy

turbine_number_of_blades = 10;     % [-] typical radial turbine blade count
turbine_blade_thickness = 0.0015;  % [m] typical cast Inconel blade thickness

turbine_hub_volume = zeros(number_of_feasible_turbine_geometries, 1);
turbine_blade_volume = zeros(number_of_feasible_turbine_geometries, 1);
turbine_total_volume = zeros(number_of_feasible_turbine_geometries, 1);

for c = 1:number_of_feasible_turbine_geometries

    x = turbine_silhouette_x(c,:);
    r_outer = turbine_silhouette_outer_radius(c,:);
    r_inner = turbine_silhouette_inner_radius(c,:);

    % CORRECTED: the "hub" is not solid all the way to the centreline --
    % it's bored out to carry the shaft. The bore radius is constant
    % along the wheel's own axial length (a real bore doesn't taper),
    % sized by this wheel's own hub-to-tip ratio at the inducer end --
    % which is exactly r_inner(1), the smallest value the hub-line
    % profile ever takes.
    bore_radius = min(r_inner(1), r_inner(end));   % smaller of the two endpoints -- the turbine hub-line SHRINKS from inducer to exducer (D2<D1 always), opposite of the compressor, so r_inner(1) alone was the wrong (larger) endpoint

    hub_vol = 0;
    blade_vol = 0;

    for i = 1:(number_of_silhouette_points - 1)

        dx = x(i+1) - x(i);

        % Hub -- annulus of revolution, bore_radius to r_inner(x), NOT a
        % solid disk from the centreline
        area1_hub = pi * (r_inner(i)^2 - bore_radius^2);
        area2_hub = pi * (r_inner(i+1)^2 - bore_radius^2);
        hub_vol = hub_vol + 0.5 * (area1_hub + area2_hub) * dx;

        % Blade material -- N thin radial plates, exact rectangular
        % volume per slice, no fill-fraction approximation needed
        radial_width_1 = r_outer(i) - r_inner(i);
        radial_width_2 = r_outer(i+1) - r_inner(i+1);
        blade_vol = blade_vol + turbine_number_of_blades * turbine_blade_thickness * 0.5*(radial_width_1 + radial_width_2) * dx;

    end

    turbine_hub_volume(c) = hub_vol;
    turbine_blade_volume(c) = blade_vol;
    turbine_total_volume(c) = hub_vol + blade_vol;

end

turbine_hub_mass = turbine_hub_volume * turbine_material_density;
turbine_blade_mass = turbine_blade_volume * turbine_material_density;
turbine_total_mass = turbine_hub_mass + turbine_blade_mass;

fprintf('\n');
fprintf('5.2 TURBINE VOLUME AND MASS\n');
fprintf('----------------------------\n');
fprintf('Material density = %.0f kg/m^3 (Inconel-class)\n', turbine_material_density);
fprintf('Blade count = %d, blade thickness = %.2f mm\n', turbine_number_of_blades, turbine_blade_thickness*1000);
fprintf('Hub volume range = %.3e - %.3e m^3\n', min(turbine_hub_volume), max(turbine_hub_volume));
fprintf('Blade volume range = %.3e - %.3e m^3\n', min(turbine_blade_volume), max(turbine_blade_volume));
fprintf('Total turbine mass range = %.4f - %.4f kg\n', min(turbine_total_mass), max(turbine_total_mass));

%% 5.3 TURBINE POLAR MOMENT OF INERTIA

% Same discretised slices as 5.2, so volume and inertia come from
% exactly the same geometry -- deliberately, so they can't drift apart.
%
% Hub slice: CORRECTED -- the hub is bored out for the shaft, so it's an
% annulus (bore_radius to r_inner(x)), not a solid disk. Exact annulus
% formula: dJ = 0.5*dm*(R_outer^2 + R_inner^2), same form already used
% for the blade envelope's cross-section, applied here since the bore
% isn't necessarily thin relative to the hub-line radius (the disk
% formula, which assumes mass all the way to r=0, no longer applies).
%
% Blade slice: each blade is a thin radial plate (thickness t_blade,
% much smaller than its radial extent), so its exact polar inertia
% comes directly from integrating r^2 over its radial extent:
%   dJ_one_blade = rho * t_blade * dx * INTEGRAL[r_inner to r_outer] r^2 dr
%                = rho * t_blade * dx * (r_outer^3 - r_inner^3) / 3
% times N_blades. This is exact for a thin blade (no area-fraction
% approximation needed, unlike the annulus formula used when blade
% material was being spread uniformly around the full circumference).

turbine_hub_inertia = zeros(number_of_feasible_turbine_geometries, 1);
turbine_blade_inertia = zeros(number_of_feasible_turbine_geometries, 1);
turbine_total_inertia = zeros(number_of_feasible_turbine_geometries, 1);

for c = 1:number_of_feasible_turbine_geometries

    x = turbine_silhouette_x(c,:);
    r_outer = turbine_silhouette_outer_radius(c,:);
    r_inner = turbine_silhouette_inner_radius(c,:);

    bore_radius = min(r_inner(1), r_inner(end));   % smaller of the two endpoints -- the turbine hub-line SHRINKS from inducer to exducer (D2<D1 always), opposite of the compressor, so r_inner(1) alone was the wrong (larger) endpoint

    hub_J = 0;
    blade_J = 0;

    for i = 1:(number_of_silhouette_points - 1)

        dx = x(i+1) - x(i);

        % Hub slice -- annulus, exact formula (bore to hub-line)
        r1 = r_inner(i); r2 = r_inner(i+1);
        area1 = pi*(r1^2 - bore_radius^2); area2 = pi*(r2^2 - bore_radius^2);
        slice_vol = 0.5*(area1+area2)*dx;
        slice_mass = slice_vol * turbine_material_density;
        mean_router_sq = 0.5*(r1^2 + r2^2);
        mean_rinner_sq = bore_radius^2;
        hub_J = hub_J + 0.5 * slice_mass * (mean_router_sq + mean_rinner_sq);

        % Blade slice -- exact radial integral of r^2 dr, times N blades
        rt1 = r_outer(i); rt2 = r_outer(i+1);
        radial_integral_1 = (rt1^3 - r1^3) / 3;
        radial_integral_2 = (rt2^3 - r2^3) / 3;
        blade_J = blade_J + turbine_number_of_blades * turbine_material_density * turbine_blade_thickness * dx * 0.5*(radial_integral_1 + radial_integral_2);

    end

    turbine_hub_inertia(c) = hub_J;
    turbine_blade_inertia(c) = blade_J;
    turbine_total_inertia(c) = hub_J + blade_J;

end

fprintf('\n');
fprintf('5.3 TURBINE POLAR MOMENT OF INERTIA\n');
fprintf('-------------------------------------\n');
fprintf('Hub inertia range = %.3e - %.3e kg m^2\n', min(turbine_hub_inertia), max(turbine_hub_inertia));
fprintf('Blade inertia range = %.3e - %.3e kg m^2\n', min(turbine_blade_inertia), max(turbine_blade_inertia));
fprintf('Total turbine inertia range = %.3e - %.3e kg m^2\n', min(turbine_total_inertia), max(turbine_total_inertia));

%% 5.4 COMPRESSOR MERIDIONAL SILHOUETTE

% The compressor impeller is a genuinely different shape from the
% turbine wheel, so this isn't a copy of 5.1. A centrifugal compressor
% impeller is unshrouded: the solid backplate/hub grows from the
% inducer hub bore (hub_to_tip_ratio * D1/2) all the way OUT TO THE
% FULL IMPELLER OD (D2/2) at the exducer -- there's no separate, reduced
% hub diameter at the exducer the way the turbine has. That's a real
% geometric difference between the two machines, not an inconsistency.
% The blade envelope is the gap between this hub line and a separate
% "tip" line that starts at the full inducer blade tip (D1/2) and
% CONVERGES down to meet the hub line by the exducer (D2/2) -- matching
% this project's own 3.4 results, where exducer blade height (b2, a few
% mm) is much smaller than inducer blade height (b1, tens of mm): a real
% unshrouded impeller has plenty of blade height at the inlet eye and
% almost none by the time flow has turned radial and reached the OD.

number_of_compressor_silhouette_points = 101;

compressor_silhouette_x = zeros(number_of_feasible_compressor_geometries, number_of_compressor_silhouette_points);
compressor_silhouette_hub_radius = zeros(number_of_feasible_compressor_geometries, number_of_compressor_silhouette_points);
compressor_silhouette_tip_radius = zeros(number_of_feasible_compressor_geometries, number_of_compressor_silhouette_points);

for a = 1:number_of_feasible_compressor_geometries

    D1 = feasible_compressor_D1(a);
    D2 = feasible_compressor_D2(a);

    total_height = maximum_compressor_total_height(a);

    r1_hub = hub_to_tip_ratio * D1 / 2;   % inducer hub bore radius
    r2_hub = D2 / 2;                       % exducer -- FULL OD, no separate hub reduction

    r1_tip = D1 / 2;                       % inducer blade tip radius (full inducer OD)
    r2_tip = D2 / 2;                       % exducer -- converges to the hub line (unshrouded impeller)

    for i = 1:number_of_compressor_silhouette_points

        xi = (i - 1) / (number_of_compressor_silhouette_points - 1);

        compressor_silhouette_x(a,i) = xi * total_height;

        curve_factor = 3 * xi^2 - 2 * xi^3;   % same smooth ease used in 5.1

        compressor_silhouette_hub_radius(a,i) = r1_hub + (r2_hub - r1_hub) * curve_factor;

        compressor_silhouette_tip_radius(a,i) = r1_tip + (r2_tip - r1_tip) * curve_factor;

    end

end

fprintf('\n');
fprintf('5.4 COMPRESSOR MERIDIONAL SILHOUETTE\n');
fprintf('--------------------------------------\n');
fprintf('Number of compressor geometries: %d\n', number_of_feasible_compressor_geometries);
fprintf('Silhouette points per geometry: %d\n', number_of_compressor_silhouette_points);
fprintf('Hub radius range: %.1f to %.1f mm\n', min(compressor_silhouette_hub_radius(:))*1000, max(compressor_silhouette_hub_radius(:))*1000);
fprintf('Tip radius range: %.1f to %.1f mm\n', min(compressor_silhouette_tip_radius(:))*1000, max(compressor_silhouette_tip_radius(:))*1000);

%% 5.5 COMPRESSOR VOLUME AND MASS

% Same hub+blade split as the turbine (5.2), same exact blade-count
% derivation (rather than an assumed fill fraction), applied to the
% compressor's own silhouette.
%
% Material: earlier versions of this project used 7800 kg/m^3 (generic
% steel-like value) for the compressor too. That's wrong for the same
% reason flagged back in 5.2 -- automotive compressor wheels are cast
% aluminium, not steel, since they never see combustion temperatures
% (compressor outlet here is only ~110-160 degC, per Sections 1.5/2.2).
% Using 2770 kg/m^3 (typical cast aluminium alloy) instead.
%
% Blade count/thickness: 6 main blades + 6 splitter blades (12 total)
% and 1.0 mm thickness are typical for a small automotive centrifugal
% compressor wheel -- same status as the turbine's assumed values: real,
% statable, and swappable for actual spec data if you have it.

compressor_material_density = 2770;   % kg/m^3, cast aluminium alloy

compressor_number_of_blades = 12;     % [-] 6 main + 6 splitter, typical
compressor_blade_thickness = 0.0010;  % [m] typical cast aluminium blade

compressor_hub_volume = zeros(number_of_feasible_compressor_geometries, 1);
compressor_blade_volume = zeros(number_of_feasible_compressor_geometries, 1);
compressor_total_volume = zeros(number_of_feasible_compressor_geometries, 1);

for a = 1:number_of_feasible_compressor_geometries

    x = compressor_silhouette_x(a,:);
    r_hub = compressor_silhouette_hub_radius(a,:);
    r_tip = compressor_silhouette_tip_radius(a,:);

    % CORRECTED: the hub/backplate is bored out for the shaft, not solid
    % to the centreline. Bore radius is constant along the wheel's own
    % axial length, sized by this wheel's own hub-to-tip ratio at the
    % inducer end -- r_hub(1), the smallest value the hub-line profile
    % takes (it grows monotonically from there out to the full OD).
    bore_radius = r_hub(1);

    hub_vol = 0;
    blade_vol = 0;

    for i = 1:(number_of_compressor_silhouette_points - 1)

        dx = x(i+1) - x(i);

        % Hub/backplate -- annulus of revolution, bore_radius to
        % r_hub(x), NOT a solid disk from the centreline
        area1_hub = pi * (r_hub(i)^2 - bore_radius^2);
        area2_hub = pi * (r_hub(i+1)^2 - bore_radius^2);
        hub_vol = hub_vol + 0.5 * (area1_hub + area2_hub) * dx;

        % Blade material -- N thin radial plates spanning the gap
        % between hub and tip lines (zero at the exducer, where the two
        % lines converge)
        radial_width_1 = r_tip(i) - r_hub(i);
        radial_width_2 = r_tip(i+1) - r_hub(i+1);
        blade_vol = blade_vol + compressor_number_of_blades * compressor_blade_thickness * 0.5*(radial_width_1 + radial_width_2) * dx;

    end

    compressor_hub_volume(a) = hub_vol;
    compressor_blade_volume(a) = blade_vol;
    compressor_total_volume(a) = hub_vol + blade_vol;

end

compressor_hub_mass = compressor_hub_volume * compressor_material_density;
compressor_blade_mass = compressor_blade_volume * compressor_material_density;
compressor_total_mass = compressor_hub_mass + compressor_blade_mass;

fprintf('\n');
fprintf('5.5 COMPRESSOR VOLUME AND MASS\n');
fprintf('--------------------------------\n');
fprintf('Material density = %.0f kg/m^3 (cast aluminium)\n', compressor_material_density);
fprintf('Blade count = %d, blade thickness = %.2f mm\n', compressor_number_of_blades, compressor_blade_thickness*1000);
fprintf('Hub volume range = %.3e - %.3e m^3\n', min(compressor_hub_volume), max(compressor_hub_volume));
fprintf('Blade volume range = %.3e - %.3e m^3\n', min(compressor_blade_volume), max(compressor_blade_volume));
fprintf('Total compressor mass range = %.4f - %.4f kg\n', min(compressor_total_mass), max(compressor_total_mass));

%% 5.6 COMPRESSOR POLAR MOMENT OF INERTIA

% CORRECTED: hub is bored out for the shaft -- annulus, not solid disk.
% Exact formula dJ = 0.5*dm*(R_outer^2+R_inner^2), same as 5.3.
% Blade: exact radial integral of r^2 dr, times N blades (unchanged).

compressor_hub_inertia = zeros(number_of_feasible_compressor_geometries, 1);
compressor_blade_inertia = zeros(number_of_feasible_compressor_geometries, 1);
compressor_total_inertia = zeros(number_of_feasible_compressor_geometries, 1);

for a = 1:number_of_feasible_compressor_geometries

    x = compressor_silhouette_x(a,:);
    r_hub = compressor_silhouette_hub_radius(a,:);
    r_tip = compressor_silhouette_tip_radius(a,:);

    bore_radius = r_hub(1);

    hub_J = 0;
    blade_J = 0;

    for i = 1:(number_of_compressor_silhouette_points - 1)

        dx = x(i+1) - x(i);

        % Hub slice -- annulus, exact formula (bore to hub-line)
        r1 = r_hub(i); r2 = r_hub(i+1);
        area1 = pi*(r1^2 - bore_radius^2); area2 = pi*(r2^2 - bore_radius^2);
        slice_vol = 0.5*(area1+area2)*dx;
        slice_mass = slice_vol * compressor_material_density;
        mean_router_sq = 0.5*(r1^2 + r2^2);
        mean_rinner_sq = bore_radius^2;
        hub_J = hub_J + 0.5 * slice_mass * (mean_router_sq + mean_rinner_sq);

        % Blade slice -- exact radial integral of r^2 dr, times N blades
        rt1 = r_tip(i); rt2 = r_tip(i+1);
        radial_integral_1 = (rt1^3 - r1^3) / 3;
        radial_integral_2 = (rt2^3 - r2^3) / 3;
        blade_J = blade_J + compressor_number_of_blades * compressor_material_density * compressor_blade_thickness * dx * 0.5*(radial_integral_1 + radial_integral_2);

    end

    compressor_hub_inertia(a) = hub_J;
    compressor_blade_inertia(a) = blade_J;
    compressor_total_inertia(a) = hub_J + blade_J;

end

fprintf('\n');
fprintf('5.6 COMPRESSOR POLAR MOMENT OF INERTIA\n');
fprintf('----------------------------------------\n');
fprintf('Hub inertia range = %.3e - %.3e kg m^2\n', min(compressor_hub_inertia), max(compressor_hub_inertia));
fprintf('Blade inertia range = %.3e - %.3e kg m^2\n', min(compressor_blade_inertia), max(compressor_blade_inertia));
fprintf('Total compressor inertia range = %.3e - %.3e kg m^2\n', min(compressor_total_inertia), max(compressor_total_inertia));

%% 5.7 TURBOCHARGER SHAFT

% The shaft is a separate component from both wheels -- different
% material (steel, not the wheels' Inconel/aluminium), and simple
% enough geometry (a uniform solid cylinder) that it needs no hub/blade
% split.
%
% Diameter: sized by where the shaft actually connects to each wheel --
% each wheel's own INDUCER hub bore (the narrowest solid section near
% the centre), not the backplate OD. A real shaft is a thin rod through
% the middle of the assembly, not as wide as either wheel's full body.
% Uses the smaller of the two connecting diameters, since the shaft
% can't be wider than the narrower of the two hub bores it passes
% through.
%
% Length: CORRECTED. Since the wheels' hubs are now modelled as bored
% out for the shaft (not solid), the shaft genuinely runs the FULL
% length of the rotating assembly -- through the compressor wheel's own
% bore, across the core gap between the wheel backs, and through the
% turbine wheel's own bore. Total length = compressor wheel's own axial
% height + core gap + turbine wheel's own axial height. The core gap
% itself (bearing span, thrust collar clearance) still isn't derivable
% from anything else in this model -- kept as a stated assumption
% (150 mm, per your correction), same status as the blade counts used
% earlier. What changed is that the wheel heights now get ADDED to it,
% not treated as already counted elsewhere.
%
% Material: steel is actually the CORRECT, realistic choice here --
% unlike the wheels, this is one place the project's original generic
% 7800 kg/m^3 value didn't need correcting, since turbocharger shafts
% genuinely are hardened steel. Using 7850 kg/m^3 (a standard steel
% value) for precision.
%
% Indexed by turbine geometry (1 to number_of_feasible_turbine_
% geometries), since that's the final assembly index this project is
% converging on -- each turbine geometry already has exactly one paired
% compressor candidate via feasible_turbine_compressor_candidate(c).

turbocharger_core_gap = 0.150;   % [m] shaft length between wheel backs -- stated assumption, not derived
turbo_shaft_material_density = 7850;   % [kg/m^3] hardened steel

turbo_shaft_length = zeros(number_of_feasible_turbine_geometries, 1);
turbo_shaft_diameter = zeros(number_of_feasible_turbine_geometries, 1);
turbo_shaft_volume = zeros(number_of_feasible_turbine_geometries, 1);
turbo_shaft_mass = zeros(number_of_feasible_turbine_geometries, 1);
turbo_shaft_inertia = zeros(number_of_feasible_turbine_geometries, 1);

for c = 1:number_of_feasible_turbine_geometries

    compressor_index = feasible_turbine_compressor_candidate(c);

    compressor_inducer_hub_diameter = hub_to_tip_ratio * feasible_compressor_D1(compressor_index);

    turbine_inducer_hub_diameter = turbine_exducer_hub_to_tip_ratio * feasible_turbine_D1(c);

    % Shaft diameter is a FRACTION of the smaller bore, not equal to it
    % (per your clarification) -- the bore is sized to hollow out the
    % wheel for the shaft, but the shaft itself doesn't fill that bore
    % completely; there's a wall of wheel material around it. shaft_to_
    % hub_ratio = 0.35 is a stated assumption: real automotive turbo
    % shafts are typically 6-10 mm in their bearing span, and with your
    % hub bore diameters running roughly 21-28 mm, that's a ratio of
    % about 0.3-0.35 -- used here as a reasonable, flagged starting
    % point rather than a literature-derived constant.
    shaft_to_hub_ratio = 0.35;

    turbo_shaft_diameter(c) = shaft_to_hub_ratio * min(compressor_inducer_hub_diameter, turbine_inducer_hub_diameter);

    % Full length: through the compressor wheel's own bore, across the
    % core, through the turbine wheel's own bore
    compressor_wheel_height = maximum_compressor_total_height(compressor_index);
    turbine_wheel_height = 3 * feasible_turbine_b1(c);   % matches the corrected total_height convention in 5.1 (H = 3*b1), not b1 alone

    turbo_shaft_length(c) = compressor_wheel_height + turbocharger_core_gap + turbine_wheel_height;

    shaft_radius = turbo_shaft_diameter(c) / 2;

    turbo_shaft_volume(c) = pi * shaft_radius^2 * turbo_shaft_length(c);

    turbo_shaft_mass(c) = turbo_shaft_volume(c) * turbo_shaft_material_density;

    % Solid cylinder about its own axis -- exact formula, same form as
    % the wheel hub disks (this genuinely IS a uniform solid cylinder,
    % not an approximation)
    turbo_shaft_inertia(c) = 0.5 * turbo_shaft_mass(c) * shaft_radius^2;

end

fprintf('\n');
fprintf('5.7 TURBOCHARGER SHAFT\n');
fprintf('------------------------\n');
fprintf('Material density = %.0f kg/m^3 (hardened steel)\n', turbo_shaft_material_density);
fprintf('Core gap (assumption) = %.1f mm\n', turbocharger_core_gap*1000);
fprintf('Shaft-to-hub-bore ratio (assumption) = %.2f\n', shaft_to_hub_ratio);
fprintf('Full shaft length range (compressor bore + core + turbine bore) = %.1f - %.1f mm\n', min(turbo_shaft_length)*1000, max(turbo_shaft_length)*1000);
fprintf('Shaft diameter range = %.1f - %.1f mm\n', min(turbo_shaft_diameter)*1000, max(turbo_shaft_diameter)*1000);
fprintf('Shaft mass range = %.4f - %.4f kg\n', min(turbo_shaft_mass), max(turbo_shaft_mass));
fprintf('Shaft inertia range = %.3e - %.3e kg m^2\n', min(turbo_shaft_inertia), max(turbo_shaft_inertia));

%% 5.8 TOTAL TURBOCHARGER INERTIA

% Combines all three components computed so far, per assembly (indexed
% by turbine geometry, since that's the index that already carries its
% own paired compressor candidate via feasible_turbine_compressor_
% candidate(c)):
%   J_total = J_turbine + J_compressor + J_shaft
%
% IMPORTANT CAVEAT: minimum total inertia is a PROXY for fastest spool,
% not the same thing. Spool time depends on available accelerating
% torque as well as inertia (Section 5.9, still to come) -- a slightly
% higher-inertia assembly with substantially more turbine torque could
% still spool faster than the minimum-inertia one. The minimum-inertia
% assembly identified here is a strong reference candidate, not
% necessarily the final answer -- that comes once torque and the spool-
% time integration are in place.

turbocharger_total_inertia = zeros(number_of_feasible_turbine_geometries, 1);

for c = 1:number_of_feasible_turbine_geometries

    compressor_index = feasible_turbine_compressor_candidate(c);

    turbocharger_total_inertia(c) = turbine_total_inertia(c) + compressor_total_inertia(compressor_index) + turbo_shaft_inertia(c);

end

[minimum_total_inertia, minimum_inertia_index] = min(turbocharger_total_inertia);

min_compressor_index = feasible_turbine_compressor_candidate(minimum_inertia_index);

fprintf('\n');
fprintf('5.8 TOTAL TURBOCHARGER INERTIA\n');
fprintf('--------------------------------\n');
fprintf('Total assembly inertia range = %.3e - %.3e kg m^2\n', min(turbocharger_total_inertia), max(turbocharger_total_inertia));
fprintf('\n');
fprintf('--- Minimum-inertia assembly (reference candidate, not yet the final answer) ---\n');
fprintf('Total inertia = %.3e kg m^2\n', minimum_total_inertia);
fprintf('  Turbine contribution   = %.3e kg m^2 (%.1f%%)\n', turbine_total_inertia(minimum_inertia_index), 100*turbine_total_inertia(minimum_inertia_index)/minimum_total_inertia);
fprintf('  Compressor contribution = %.3e kg m^2 (%.1f%%)\n', compressor_total_inertia(min_compressor_index), 100*compressor_total_inertia(min_compressor_index)/minimum_total_inertia);
fprintf('  Shaft contribution      = %.3e kg m^2 (%.1f%%)\n', turbo_shaft_inertia(minimum_inertia_index), 100*turbo_shaft_inertia(minimum_inertia_index)/minimum_total_inertia);
fprintf('Turbine D1 = %.1f mm, D2 = %.1f mm, inlet angle = %.0f deg\n', feasible_turbine_D1(minimum_inertia_index)*1000, feasible_turbine_D2(minimum_inertia_index)*1000, feasible_turbine_angle(minimum_inertia_index));
fprintf('Paired compressor candidate = #%d (D1 = %.1f mm, D2 = %.1f mm)\n', min_compressor_index, feasible_compressor_D1(min_compressor_index)*1000, feasible_compressor_D2(min_compressor_index)*1000);
fprintf('Turbine mass = %.4f kg, Compressor mass = %.4f kg, Shaft mass = %.4f kg\n', turbine_total_mass(minimum_inertia_index), compressor_total_mass(min_compressor_index), turbo_shaft_mass(minimum_inertia_index));

%% 5.9 SPOOL TIME

% Scenario: "throttle stab from idle" -- shaft starts at rest (omega=0),
% target is the matched shaft speed at a given engine rpm under full
% load (desired_power(k)). This models a step to WOT, not gradual
% acceleration.
%
% Torque model: turbine efficiency varies with velocity ratio
% nu_s = U/C0 (blade speed / spouting velocity), following the
% parabolic shape documented in the literature (e.g. Watson & Janota;
% Moraal & Kolmanovsky), peaking near nu_s = 0.707. Since nu_s is
% exactly proportional to omega for a fixed wheel during one spool-up
% event, nu_s(omega)/nu_s,matched = omega/omega_matched = x, the same
% speed ratio used everywhere else in this model -- no need to compute
% C0 separately. Anchored to reproduce this model's existing matched-
% point efficiency (turbine_efficiency=0.70) exactly at x=1, so nothing
% about the matched-point results in 5.0-5.8 changes:
%   T_turbine(x) = T0*(2-x),  T0 = turbine_power_required(k)/omega_matched
%
% CAPPED below x_threshold: this parabolic correlation is validated by
% the literature over roughly nu_s = 0.5-1.0+ -- below that, letting the
% torque multiplier keep climbing toward its uncapped value of 2x at
% x=0 is extrapolating well past where any of this has been tested.
% Below x_threshold, the multiplier is held constant at its value AT
% x_threshold rather than extrapolated further:
%   T_turbine(x) = T0*(2-x)             for x >= x_threshold
%   T_turbine(x) = T0*(2-x_threshold)   for x <  x_threshold
% This keeps the genuine physical result (torque does not collapse the
% way efficiency does, because torque = power/omega) without overclaiming
% precision in the part of the curve with no supporting data.
%
% Resisting side (compressor + the ~3% friction already implicit in
% mechanical_efficiency) is kept exactly as before: T0*x^2.
%
% The cap makes the ODE piecewise, which costs the single closed-form
% solution used previously. Rather than hand-derive and stitch together
% two analytic segments (a real error risk to take on by hand), this
% uses fine fixed-step numerical integration instead -- vectorised
% across all candidates simultaneously (looping only over time steps,
% not candidates), which keeps this fast regardless of candidate count,
% exactly as established when this section was first built.

spool_fraction_threshold = 0.95;   % "spooled up" = 95% of matched shaft speed
x_threshold = 0.5;                 % velocity ratio below which the torque correction is capped, not extrapolated further

spool_evaluation_rpm = [1500 2500 3500 4500 5500 6500 7500 8500 9500 10000];
number_of_spool_points = length(spool_evaluation_rpm);

spool_evaluation_index = zeros(1, number_of_spool_points);
for i = 1:number_of_spool_points
    spool_evaluation_index(i) = find(rpm == spool_evaluation_rpm(i), 1);
end

integration_dt = 0.002;    % [s] fixed time step, chosen fine enough for stability across the fastest-spooling candidates
integration_t_max = 40;    % [s] comfortably above the slowest spool times seen in earlier runs
integration_steps = round(integration_t_max / integration_dt);

spool_time = NaN(number_of_feasible_turbine_geometries, number_of_spool_points);   % NaN = did not reach threshold within t_max

J_all = turbocharger_total_inertia;   % (n x 1), same for every rpm point

for i = 1:number_of_spool_points

    k = spool_evaluation_index(i);

    T0_power = turbine_power_required(k);   % universal across candidates, function of k only

    omega_matched_all = zeros(number_of_feasible_turbine_geometries, 1);
    for c = 1:number_of_feasible_turbine_geometries
        compressor_index = feasible_turbine_compressor_candidate(c);
        omega_matched_all(c) = feasible_compressor_required_turbo_speed(compressor_index, k) * pi / 30;
    end

    T0_torque_all = T0_power ./ omega_matched_all;              % matched-point torque, per candidate
    rate_all = T0_torque_all ./ (J_all .* omega_matched_all);   % dx/dt = rate * [capped_multiplier - x^2]

    x_state = zeros(number_of_feasible_turbine_geometries, 1);   % start from rest, all candidates at once
    reached = false(number_of_feasible_turbine_geometries, 1);

    for step = 1:integration_steps

        capped_multiplier = 2 - max(x_state, x_threshold);

        dxdt = rate_all .* (capped_multiplier - x_state.^2);

        x_new = x_state + integration_dt * dxdt;

        crossing = (~reached) & (x_new >= spool_fraction_threshold);

        if any(crossing)
            frac = (spool_fraction_threshold - x_state(crossing)) ./ (x_new(crossing) - x_state(crossing));
            spool_time(crossing, i) = (step - 1) * integration_dt + frac * integration_dt;
            reached(crossing) = true;
        end

        x_state = x_new;

        if all(reached)
            break
        end

    end

end

mean_spool_time = mean(spool_time, 2);

[best_mean_spool_time, best_spool_index] = min(mean_spool_time);

best_spool_compressor_index = feasible_turbine_compressor_candidate(best_spool_index);

fprintf('\n');
fprintf('5.9 SPOOL TIME\n');
fprintf('----------------\n');
fprintf('Scenario: throttle stab from idle (omega=0) to %.0f%% of matched shaft speed\n', spool_fraction_threshold*100);
fprintf('Torque correction capped below velocity ratio x = %.2f (uncapped beyond that, per the literature)\n', x_threshold);
fprintf('Evaluated at engine speeds [rpm]: ');
fprintf('%d ', spool_evaluation_rpm);
fprintf('\n\n');

for i = 1:number_of_spool_points
    fprintf('  %5d rpm: spool time range = %.4f - %.4f s\n', spool_evaluation_rpm(i), min(spool_time(:,i)), max(spool_time(:,i)));
end

fprintf('\n');
fprintf('--- Best assembly by mean spool time across all %d points ---\n', number_of_spool_points);
fprintf('Mean spool time = %.4f s\n', best_mean_spool_time);
fprintf('Total inertia = %.3e kg m^2 (rank by inertia alone: see 5.8)\n', turbocharger_total_inertia(best_spool_index));
fprintf('Turbine D1 = %.1f mm, D2 = %.1f mm, inlet angle = %.0f deg\n', feasible_turbine_D1(best_spool_index)*1000, feasible_turbine_D2(best_spool_index)*1000, feasible_turbine_angle(best_spool_index));
fprintf('Paired compressor candidate = #%d (D1 = %.1f mm, D2 = %.1f mm)\n', best_spool_compressor_index, feasible_compressor_D1(best_spool_compressor_index)*1000, feasible_compressor_D2(best_spool_compressor_index)*1000);

if best_spool_index == minimum_inertia_index
    fprintf('\nThis MATCHES the minimum-inertia assembly from 5.8 -- inertia alone was a good proxy for this design.\n');
else
    fprintf('\nThis DIFFERS from the minimum-inertia assembly from 5.8 -- torque availability changed the ranking, confirming inertia alone was not sufficient to predict fastest spool.\n');
end

%% 6.0 QUANTITIES NEEDED ONLY FOR PLOTTING

% NOTE: boost_pressure and corrected_mass_flow were used in the plotting
% section below without ever being computed -- the same "undefined
% variable" bug found elsewhere, here it just never crashed because
% MATLAB doesn't evaluate figure code until you reach it.

% Boost pressure [Pa] -- gauge pressure rise across the compressor,
% already available from Section 1.5's compressor_outlet_pressure(k).
boost_pressure = compressor_outlet_pressure - ambient_pressure;

% Corrected mass flow [kg/s] -- standard SAE-style correction to sea
% level standard-day reference conditions (15 degC, 101.325 kPa), so
% flow can be compared across different inlet conditions. Here inlet
% conditions are ambient and constant across rpm, so this is mostly a
% constant scale factor on required_airmass_flow -- included for
% completeness/convention rather than because it reveals new rpm-
% dependent behaviour in this particular model.
reference_temperature = 288.15;
reference_pressure = 101325;
corrected_mass_flow = required_airmass_flow * sqrt(ambient_temperature/reference_temperature) / (ambient_pressure/reference_pressure);

%% PLOTTING SECTION

figure
plot(rpm, brake_power_NA / 1000, 'LineWidth', 1.5);

xlabel('Engine Speed [rpm]');
ylabel('Brake Power [kW]');
xlim([0, 11000]);
ylim([0, 400]);
title('Naturally Aspirated VS Target Engine Power');
grid on

figure
plot(rpm, mass_flow_NA, 'LineWidth', 1.5);
hold on
plot(rpm, required_airmass_flow, 'LineWidth', 1.5);
xlabel('Engine Speed [rpm]');
ylabel('Air Mass Flow [kg/s]');
xlim([0, 11000]);
ylim([0, 0.3]);
title('Naturally Aspirated vs Required Airflow');
legend('Naturally Aspirated', 'Required for Target Power');
grid on

figure
plot(rpm, pressure_ratio_required, 'LineWidth', 1.5);
xlabel('Engine Speed [rpm]');
ylabel('Required Pressure Ratio');
xlim([0, 11000]);
ylim([0, 15]);
title('Required Compressor Pressure Ratio vs Engine Speed');
grid on

figure
plot(rpm, required_airmass_flow, 'LineWidth', 1.5);
hold on
plot(rpm, corrected_mass_flow, 'LineWidth', 1.5);
grid on
xlabel('Engine Speed [rpm]');
ylabel('Mass Flow [kg/s]');
xlim([0, 11000]);
ylim([0, 0.28]);
title('Required vs Corrected Compressor Mass Flow');
legend('Required Flow','Corrected Flow');

figure
plot(rpm, boost_pressure / 100000, 'LineWidth', 1.5);
xlabel('Engine Speed [rpm]');
ylabel('Boost Pressure [bar]');
title('Required Boost Pressure vs Engine Speed');
grid on

figure
plot(rpm, t_compressor_out - 273.15, 'LineWidth', 1.5);
xlabel('Engine Speed [rpm]');
ylabel('Compressor Outlet Temperature [°C]');
xlim([0, 11000]);
title('Compressor Outlet Temperature vs Engine Speed');
grid on

figure
plot(rpm, compressor_power_required / 1000, 'LineWidth', 1.5);
xlabel('Engine Speed [rpm]');
ylabel('Compressor Power [kW]');
xlim([0, 11000]);
title('Required Compressor Power vs Engine Speed');
grid on

%% 7.0 PORTFOLIO PLOTS

% Four additional figures built specifically for presenting this project
% (e.g. in a portfolio): the design space explored, the physical
% cross-section of the selected assembly, its transient response, and
% the key finding that minimum inertia and minimum spool time are not
% the same design.

%% 7.1 FEASIBLE DESIGN SPACE (colored by total inertia)

figure
scatter(feasible_turbine_D1*1000, feasible_turbine_D2*1000, 6, turbocharger_total_inertia, 'filled');
hold on
plot(feasible_turbine_D1(minimum_inertia_index)*1000, feasible_turbine_D2(minimum_inertia_index)*1000, ...
    'p', 'MarkerSize', 16, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k');
plot(feasible_turbine_D1(best_spool_index)*1000, feasible_turbine_D2(best_spool_index)*1000, ...
    'p', 'MarkerSize', 16, 'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'k');
hold off
xlabel('Turbine D1 [mm]');
ylabel('Turbine D2 [mm]');
cb = colorbar;
cb.Label.String = 'Total Turbocharger Inertia [kg m^2]';
title(sprintf('Feasible Turbine Design Space (%d geometries)', number_of_feasible_turbine_geometries));
legend('All feasible geometries', 'Minimum inertia (5.8)', 'Minimum mean spool time (5.9)', 'Location', 'bestoutside');
grid on

%% 7.2 SELECTED TURBOCHARGER ASSEMBLY CROSS-SECTION

% Reconstructed from the meridional silhouette data already computed for
% the design selected by minimum mean spool time (5.9). Shows the actual
% physical picture established through this project: a solid backplate/
% hub bored out for the shaft, blade material occupying the outer
% envelope, connected by the shaft across the core gap.

selected_turbine_index = best_spool_index;
selected_compressor_index = best_spool_compressor_index;

t_x = turbine_silhouette_x(selected_turbine_index, :);
t_outer = turbine_silhouette_outer_radius(selected_turbine_index, :);
t_inner = turbine_silhouette_inner_radius(selected_turbine_index, :);

c_x = compressor_silhouette_x(selected_compressor_index, :);
c_tip = compressor_silhouette_tip_radius(selected_compressor_index, :);
c_hub = compressor_silhouette_hub_radius(selected_compressor_index, :);

% Place the compressor to the left of the turbine, separated by the
% core gap, for a single combined assembly view
compressor_offset = -(max(c_x) + turbocharger_core_gap);
c_x_plot = c_x + compressor_offset;
t_x_plot = t_x;

shaft_radius_plot = turbo_shaft_diameter(selected_turbine_index) / 2 * 1000;
shaft_x_start = min(c_x_plot) * 1000;
shaft_x_end = max(t_x_plot) * 1000;

figure
hold on

plot(t_x_plot*1000, t_outer*1000, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Turbine blade tip');
plot(t_x_plot*1000, -t_outer*1000, 'b-', 'LineWidth', 1.5, 'HandleVisibility', 'off');
plot(t_x_plot*1000, t_inner*1000, 'b--', 'LineWidth', 1, 'DisplayName', 'Turbine hub boundary');
plot(t_x_plot*1000, -t_inner*1000, 'b--', 'LineWidth', 1, 'HandleVisibility', 'off');

plot(c_x_plot*1000, c_tip*1000, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Compressor blade tip');
plot(c_x_plot*1000, -c_tip*1000, 'r-', 'LineWidth', 1.5, 'HandleVisibility', 'off');
plot(c_x_plot*1000, c_hub*1000, 'r--', 'LineWidth', 1, 'DisplayName', 'Compressor hub boundary');
plot(c_x_plot*1000, -c_hub*1000, 'r--', 'LineWidth', 1, 'HandleVisibility', 'off');

plot([shaft_x_start shaft_x_end], [shaft_radius_plot shaft_radius_plot], 'k-', 'LineWidth', 1, 'DisplayName', 'Shaft');
plot([shaft_x_start shaft_x_end], [-shaft_radius_plot -shaft_radius_plot], 'k-', 'LineWidth', 1, 'HandleVisibility', 'off');

hold off
axis equal
xlabel('Axial Position [mm]');
ylabel('Radius [mm]');
title('Selected Turbocharger Assembly Cross-Section');
legend('Location', 'bestoutside');
grid on

%% 7.3 SPOOL TIME VS ENGINE SPEED FOR THE SELECTED DESIGN

figure
plot(spool_evaluation_rpm, spool_time(best_spool_index, :), '-o', 'LineWidth', 1.5, 'MarkerFaceColor', 'b');
xlabel('Engine Speed [rpm]');
ylabel('Spool Time to 95% Matched Speed [s]');
title('Spool Time vs Engine Speed -- Selected Design');
grid on

%% 7.4 INERTIA VS SPOOL TIME ACROSS ALL FEASIBLE DESIGNS

% The key finding of this project, visualised: minimum inertia and
% minimum spool time are NOT the same design (5.9's closing check).

figure
scatter(turbocharger_total_inertia, mean_spool_time, 6, 'filled', 'MarkerFaceAlpha', 0.25);
hold on
plot(turbocharger_total_inertia(minimum_inertia_index), mean_spool_time(minimum_inertia_index), ...
    'p', 'MarkerSize', 16, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k');
plot(turbocharger_total_inertia(best_spool_index), mean_spool_time(best_spool_index), ...
    'p', 'MarkerSize', 16, 'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'k');
hold off
xlabel('Total Turbocharger Inertia [kg m^2]');
ylabel('Mean Spool Time [s]');
title('Inertia vs Spool Time Across All Feasible Designs');
legend('All feasible geometries', 'Minimum inertia (5.8)', 'Minimum mean spool time (5.9)', 'Location', 'best');
grid on