% Create a new Simulink model
modelName = 'FlightSimulationV2';
new_system(modelName);
open_system(modelName);

% Define block positions
x = 100; y = 50; dx = 200; dy = 100;

% Add Input Blocks
add_block('built-in/Constant', [modelName, '/Initial Conditions'], 'Position', [x, y, x+60, y+30]);

% Add Subsystems
add_block('built-in/Subsystem', [modelName, '/Kinematics'], 'Position', [x+dx, y, x+dx+100, y+50]);
add_block('built-in/Subsystem', [modelName, '/Aerodynamics'], 'Position', [x+2*dx, y, x+2*dx+100, y+50]);
add_block('built-in/Subsystem', [modelName, '/IMU Model'], 'Position', [x+3*dx, y, x+3*dx+100, y+50]);
add_block('built-in/Subsystem', [modelName, '/Motor Model'], 'Position', [x+4*dx, y, x+4*dx+100, y+50]);

% Add Output Blocks
add_block('built-in/Scope', [modelName, '/Scope'], 'Position', [x+5*dx, y, x+5*dx+60, y+30]);

% Connect Blocks
add_line(modelName, 'Initial Conditions/1', 'Kinematics/1');
add_line(modelName, 'Kinematics/1', 'Aerodynamics/1');
add_line(modelName, 'Aerodynamics/1', 'IMU Model/1');
add_line(modelName, 'IMU Model/1', 'Motor Model/1');
add_line(modelName, 'Motor Model/1', 'Scope/1');

% Save and open the model
save_system(modelName);
open_system(modelName);

% Run the simulation
simOut = sim(modelName); 