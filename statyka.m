% Skrypt służący do wykreślania charakterystyk statycznych obiektu rozpatrywanego
% w zadaniu 13. w projekcie z przedmiotu Sztuczna Inteligencja w Automatyce.

global tanks;
load( 'tanksParameters.mat' );

% Zmienne czasu wykorzystywane w symulacji obiektu. Wszystkie wartości podane
% są w sekundach.
dt = 20;
time = [ 0 : dt : 3000 ];
simulationLength = length(time);

% Wartości sterowania dla których obiekt jest symulowany aż do uzyskania
% stanu ustalonego.
controlValues = [ 0.1 : 0.1 : 200 ];
controlOutputSSValues = zeros( length( controlValues ), 1 );

% Wartości zakłócenia dla których obiekt jest symulowany aż do uzyskania
% stanu ustalonego.
disturbanceValues = [ 0.1 : 0.1 : 40 ];
disturbanceOutputSSValues = zeros( length( disturbanceValues ), 1 );

% Wektor do logowania wyjścia symulowanego obiektu.
output = zeros( simulationLength, 1 );

% Struktura opcji przekazywana dla procedury ode45. Dla rozważanego obiektu
% wystarczające są wartości domyślne.
opts = odeset();

% Główna pętla algorytmu - dla wszystkich wartości wejścia i zakłócenia
% po kolei symulowany jest obiekt. Symulacja trwa dostatecznie długo,
% aby pewne było osiągnięcie stanu ustalonego obiektu.
display('Pozyskiwanie charakterystyk statycznych obiektu:');
for j = 1 : length(controlValues) + length(disturbanceValues)

   % Rozejście warunkowe do inicjalizacji wektorów wejściowych dla każdej
   % symulacji odpowiednimi wartościami - stałym zakłóceniem, kiedy
   % symulacja następuje dla kolejno zmienianych wartości sterowania
   % i stałego sterowania dla podlegającego zmianie zakłócenia.
   if j <= length(controlValues)
      inputTrajectory = controlValues(j) * ones( simulationLength, 1 );
      disturbanceTrajectory = tanks.sstate.FD * ones( simulationLength, 1 );
      inputBuffer = tanks.sstate.F1 * ones( tanks.const.tau / dt, 1 );
      tanksState = [ tanks.sstate.V1, tanks.sstate.V2 ];
   else
      inputTrajectory = tanks.sstate.F1 * ones( simulationLength, 1 );
      disturbanceTrajectory = ...
         disturbanceValues( j - length(controlValues) )*ones( simulationLength, 1 );
      inputBuffer = tanks.sstate.F1 * ones( tanks.const.tau / dt, 1 );
      tanksState = [ tanks.sstate.V1, tanks.sstate.V2 ];
   end

   fprintf('Symulacja %d z %d\n', ...
      j, length(controlValues) + length(disturbanceValues) );

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
  
      % Rozwiązywanie równań różniczkowych obiektu. 
      timeInterval = [ time(i), time(i) + dt ];
      u = inputBuffer( length(inputBuffer) );
      z = disturbanceTrajectory(i);
      inputBuffer = [ inputTrajectory(i); inputBuffer(1 : length(inputBuffer)-1 ) ];
   
      [ tout, xout ] = ode45( @tanksFunction, timeInterval, tanksState, opts, u, z );
      tanksState = xout( size(xout, 1), : );
     
      output(i) = sqrt( tanksState(2) / tanks.const.C2 );
      
   end
   
   fprintf('\n\n');

   % Zapisywanie wartości wyjścia obiektu do odpowiednich wektorów (zmiana tylko
   % sterowania i tylko zakłócenia).
   if j <= length(controlValues)
      controlOutputSSValues(j) = output( length(output) );
   else
      disturbanceOutputSSValues( j - length(controlValues) ) = ...
         output( length(output) );
   end
end

fprintf( 'Done!\n\n' );

save( 'charakterystykiStatyczne.mat',...
   'controlValues', 'controlOutputSSValues',...
   'disturbanceValues', 'disturbanceOutputSSValues'...
);
