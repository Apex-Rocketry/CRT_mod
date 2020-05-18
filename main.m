%% Preâmbulo
% Limpando variáveis antigas e adicionando funções customizadas
clear all
clc

path(pathdef)
addpath('Functions')

%% FOGUETE
% Define os componentes e distribução de massa

% ex.: descrição do foguete BRAVA A-22 

% Nariz
nosecone = {'nose','von karman',0.24,0.075,0.140,0};
nose_coupler = {'cylinder','no',0.025,0.068,0.054,0.24};

% Seção da Eletrônica
top_airframe = {'tube','yes',0.275,0.071,0.075,0.294,nosecone{3}};
eletronics_ring = {'cylinder','no',0.02,0.0069,0.028,0.446};
eletronics = {'tube','no',0.2,0.038,0.039,0.205,0.266};
parachute_upper_ring = {'cylinder','no',0.068,0.059,0.213,0.49};
threaded_bar1 = {'pm',0.036,0.09,0.49+0.48/2};
threaded_bar2 = {'pm',0.036,-0.09,0.49+0.48/2};


% Seção do Paraquedas
mid_airframe = {'tube','yes',0.68,0.071,0.075,0.764,top_airframe{3}+top_airframe{7}};
parachute = {'parachute',0.8,pi*0.3^2/4,0.265,0.56+0.22};
parachute_lower_ring = {'cylinder','no',0.03,0.069,0.033,1.01};
launch_lug = {'pm',0.013,0.075,1.01};
threaded_bar3 = {'pm',0.04,0.09,1.04+0.347/2};
threaded_bar4 = {'pm',0.04,-0.09,1.04+0.347/2};
fin_upper_support = {'cylinder','no',0.06,0.069,0.114,1.04};

% Seção do Motor (+ Aletas)
bottom_airframe = {'tube','yes',0.18,0.071,0.075,0.09,mid_airframe{3}+mid_airframe{7}};
fin_bottom_support = {'cylinder','no',0.06,0.072,0.101,1.31};
fins = {'finset',3,0.115,0.03,0.13,0.115,0.003,0.301,0.075,0.075,1.19};

% Motor
motor_bulkhead = {'cylinder','no',0.022,0.048,0.27,1.07};
motor_nozzle = {'cylinder','no',0.085,0.037,0.396,1.31};
motor = import_eng('Motor-A-Nakka.eng');

% INTAB junta todos os componentes e cria o foguete. Todos os
% componentes criados na seção anterior devem ser passados como input para
% a função intab_builder.
INTAB = intab_builder(nosecone,nose_coupler,...
                      top_airframe,eletronics_ring,eletronics,...
                      parachute_upper_ring,threaded_bar1,threaded_bar2,...
                      mid_airframe,parachute,parachute_lower_ring,...
                      launch_lug,threaded_bar3,threaded_bar4,...
                      fin_upper_support,bottom_airframe,fin_bottom_support,fins,...
                      motor_bulkhead,motor_nozzle,motor);

%% ATMOSFERA
% Define propriedades atmosféricas (retiradas do INMET)

% ex.: BRAVA A-22 com janela de voo na primeira semana de abril 2020

 altitude = [2:20:2000]';
 
% Dados do site do INMET (médias para os dados da janela de voo)
 avg_wind_direction = 60; 
 avg_temperature =  10;
 avg_windspeed = 2; 
 avg_windshear = 10;

% Parâmetros de referência
 height = 2; 
 roughness = 0.1; 
 reference = [height, roughness];
 
% Parâmetros de aleatoriedade 
 rand_temperature = 0.05;
 rand_velocity = 0.25;
 rand_direction = 10;
 random = [rand_velocity, rand_direction, rand_temperature];
 
 plot_atm = 'yes';
 
 % Função CAIO_atmosphere calcula a atmosfera simplificada a partir dos
 % dados do INMET e dos parâmetros de referência e randômicos. 
 ATM = CAIO_atmosphere(altitude, avg_windspeed, avg_wind_direction,...
                       avg_temperature, reference, random, plot_atm);
               
 INTAB4 = f214read(ATM);
 
 
%% Lançamento
% Define as condições de lançamento

Parachute_altitude_action = 0; 
Base_length = 3;      
Base_declination = 5; 
Base_angle = avg_wind_direction;


%% Simulação
% Define função de simulação e quantidade de lançamentos simulados

% Função rocketflight_monte simula lançamento com um estágio e aplica o
% método estatístico de monte carlo para introduzir aleatoriedade em alguns
% parâmetros aerodinâmicos do foguete.

iterations = 100; % número de lançamentos simulados
[Ascbig,Desbig,Landing,Apogee] = rocketflight_monte(INTAB, ...
                                                    INTAB4,...
                                                    Parachute_altitude_action, ...
                                                    Base_length, ...
                                                    Base_declination, ...
                                                    Base_angle, ...
                                                    iterations);


%% Pós-Processamento
% Tratamento de dados para criar plots dos resultados

% A função flight_variables salva várias variáveis diferentes do voo em um
% arquivo csv (chamado aqui "FlightData01").

[headers,RDT]=flight_variables('FlightData01',...
                                Ascbig{1},...
                                INTAB,...
                                INTAB4,...
                                Parachute_altitude_action,...
                                Base_length,...
                                Base_declination,...
                                Base_angle); 
                            

% Relendo dados do voo salvo pela função flight_variables                            
flight01_data = readtable('FlightData01.csv');

time = table2array(flight01_data(:,{'Time_s_'}));
vx = table2array(flight01_data(:,{'VelocityVx_m_s_'}));
vy = table2array(flight01_data(:,{'VelocityVy_m_s_'}));
vz = table2array(flight01_data(:,{'VelocityVz_m_s_'}));
ax = table2array(flight01_data(:,{'AccelerationAx_m_s2_'}));
ay = table2array(flight01_data(:,{'AccelerationAy_m_s2_'}));
az = table2array(flight01_data(:,{'AccelerationAz_m_s2_'}));
fx = table2array(flight01_data(:,{'ForceX_N_'}));
fy = table2array(flight01_data(:,{'ForceY_N_'}));
fz = table2array(flight01_data(:,{'ForceZ_N_'}));
v_mag = sqrt(vx.*vx + vy.*vy + vz.*vz);
a_mag = sqrt(ax.*ax + ay.*ay + az.*az);
f_mag = sqrt(fx.*fx + fy.*fy + fz.*fz);
figure(6);
plot(time, v_mag, '-k')
hold on
grid on
plot(time, a_mag, '-b')
plot(time, f_mag, '-r')
xlabel('Time (s)')
lg = legend('v_{mag} (m/s)','a_{mag} (m/s2)','f_{mag} (N)');
lg.FontSize = 14;
                            
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                         COMENTÁRIOS ADICIONAIS                          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Building your rocket %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % INTAB = intab_builder(rocket parts);
    % The inputs are the rocket parts, as described below
    % The output is the rocket data on the INTAB format

    %% Motor
        % Motor = {'motor', 'Name', Ttable, length, diameter, position}
        % Ttable = {time,thrust,mass}
        % 'position' should be 0 when developing a new motor, as it will be
        % replaced on the intab_builder function. It represents the distance
        % between the foremost point of the rocket and the motor position.
        % import_eng is a custom script that imports data from a .eng file 
        % to the motor variable. The .eng motor file should be contained in
        % the same directory from which the simulation is being run.

    %% Nose cone
        % Nose = {'nose', type, length, diameter, mass, position}
        % type can be 'ogive', 'conical' or 'parabolic'
        % type = 'von karman' is also possible, due to an customized ad-on to 
        % to the function Barrowman_calc.m (works when using the customized
        % 'Functions' directory).
        % position for the nose cone should be zero. position always means
        % distance relative to the rocket's nose tip.

    %% Cylinder
        % Cil = {'cylinder', surface, length, diameter, mass, position}
        % 'surface' is 'yes' if it forms part of the rocket's body
        % aerodynamic surface, or 'no' if it does not.

    %% Tube
        % T = {'tube', surface, length, inner diameter,
        %       outer diameter, mass, position}

    %% Conical transition
        % CT = {'cone_trans', surface, upstream diameter,
        %       downstream diameter, maximum body diameter, 
        %       length, mass, position}

    %% Point mass
        % PM = {'pm', mass, r_position, l_position}
        % r_position is the radial position from the center axis
        % l_position is the longitudinal position from the nose tip

    %% Fins set
        % FS = {'finset', number of fins, root chord, tip chord, sweep length,
        %       span, thickness, mass, body diameter at fin, 
        %       maximum body diameter, position}

    %% Parachute
        % P = {'parachute', Cd, area, mass, position}    
        % If dual deploy is used, drogue must be before main
        
    %% INTAB format
        % INTAB = {INTAB1, INTAB2, INTAB3, landa, paratab}
        % INTAB1 = {time, thrust, mass, Ix, Iy, Iz, 0, 0, 0, Cg, Cdar}
        % INTAB2 = Drag data using drag_datcom
        % INTAB3 = [Normal force, Cp] Using Barrowman_calc
        % landa = [length, area]; Rocket data
        % paratab = [Cd, area];  Parachute data


%% Customized Atmosphere with Interspersed Oscilations (CAIO_atmosphere)
    % The custom atmosphere model implemented here is based on a (very
    % crude) logarithm velocity distribution, with additional pertubations
    % being accounted for via randomized scalars applied to the relevant
    % atmospheric variables.
    
    %% Altitude vector
        % altitude = [lb_h : step_h : max_h]
        % lb_h is the launch base height (must be =/= 0)
        % step_h is the incremente in height
        % max_h is the maximum height (must be higher the the expected
        % apogee).
        
    %% INMET DATA
        % http://www.inmet.gov.br/portal/index.php?r=estacoes/estacoesAutomaticas
        % is a repository of historic meteorological data.
        % Given a launch site and a launch window, a sample of the relevant
        % atmospheric data should be taken for the closest meteorological
        % station to the launch site, using the past year's data for the
        % same dates determined by the launch window.
    
        % avg_wind_direction is the average (for all measurements within
        % the launch window) of the wind direction (relative to true
        % north) (INMET: Vento -> Dir.).
        
        % avg_temperature is the average (for all measurements within the
        % launch window) of the ground wind's mean temperature. (INMET:
        % Temperatura -> 0.5*(Min+Max)).
        
        % avg_windspeed is the average (for all measurements within the
        % launch window) of the mean wind velocity (INMET: Vento -> Vel.).
        
        % avg_windshear is the average (for all measurements within the
        % launch window) of the shear wind velocity (INMET: Vento -> Raj.).
 
    %% Reference Parameters
        % Parameters for the logarithm model of the atmosphere.
        
        % ref_height is the reference height at which ground velocity data
        % is know. Must be =/= 0, tipically 2m for INMET data.
        
        % ref_roughness is the friction coefficient between ground and
        % atmosphere. Typical value is ~0.1.
        % https://en.wikipedia.org/wiki/Roughness_length
        
    
    %% Random Parameters
        % Parameters for the randomization of the atmospheric profiles.
        
        % rand_temperature is the maximum percentual variation allowed for
        % in the temperature value. The final temperature profile will
        % oscilate from it's average value randomly by this % factor.
        
        % rand_velocity is the maximum percentual variation allowed for in
        % the wind velocity profile.
        
        % rand_direction is the maximum dregrees (from true north)
        % variation allowed for in the wind direction profile.
    
