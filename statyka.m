% Skrypt służący do wstępnej symulacji obiektu układu zbiorników
% rozpatrywanego w zadaniu 13. w projekcie z przedmiotu Sztuczna
% Inteligencja w Automatyce.

global tanks;
load( 'tanksParameters.mat' );

dt = 20; % sekundy

time = [ 0 : dt : 3000 ];
simulationLength = length(time);

controlValues = [ 0.1 : 0.1 : 200 ];
outputSSValues = zeros( length( controlValues ), 1 );

inputTrajectory = tanks.sstate.F1 * ones( simulationLength, 1 );
for i = dt * 20 : simulationLength
   inputTrajectory(i) = inputTrajectory(i) + 0;
end

disturbanceTrajectory = tanks.sstate.FD * ones( simulationLength, 1 );
% for i = 1 : simulationLength
%    disturbanceTrajectory(i) = disturbanceTrajectory(i) + 0;
% end

inputBuffer = tanks.sstate.F1 * ones( tanks.const.tau / dt, 1 );

tanksState = [ 10102, tanks.const.C2*(tanks.sstate.h2)^2 ];

plotInput = zeros( simulationLength, 1 );
plotOutput = zeros( simulationLength, 1 );

opts = odeset();

for j = 1 : length( controlValues )

   inputTrajectory = controlValues(j) * ones( simulationLength, 1 );
   inputBuffer = tanks.sstate.F1 * ones( tanks.const.tau / dt, 1 );
   tanksState = [ 10102, tanks.const.C2*(tanks.sstate.h2)^2 ];

   display( controlValues(j) );

   % ======== ZMIENNE WYŚWIETLAJĄCE POSTĘP SYMULACJI ================= %
   simulation_step = 0;                                                %
   letters_written = 0;                                                %
   finish_time = simulationLength;                                     %
   fprintf('\n');                                                      %
   % ================================================================= %

   for i = 1 : simulationLength
   
      % ======== ZMIENNE WYŚWIETLAJĄCE POSTĘP SYMULACJI ================= %
      if i > simulation_step*(finish_time/100)                            %
         while letters_written > 0                                        %
            fprintf('\b');                                                %
            letters_written = letters_written - 1;                        %
         end                                                              %
         letters_written = fprintf('Progress:    %d%%',simulation_step);  %
         simulation_step = simulation_step + 1;                           %
      end                                                                 %
      % ================================================================= %
   
      timeInterval = [ time(i), time(i) + dt ];
      u = inputBuffer( length(inputBuffer) );
      z = disturbanceTrajectory(i);
      inputBuffer = [ inputTrajectory(i); inputBuffer(1 : length(inputBuffer)-1 ) ];
   
   
      [ tout, xout ] = ode45( @tanksFunction, timeInterval, tanksState, opts, u, z );
      tanksState = xout( size(xout, 1), : );
     
      plotInput = inputTrajectory(i); 
      plotOutput(i) = sqrt( tanksState(2) / tanks.const.C2 );
      
   end
   
   fprintf('\n\n');

   outputSSValues(j) = plotOutput( length(plotOutput) );
end

fprintf( 'Done!\n\n' );
