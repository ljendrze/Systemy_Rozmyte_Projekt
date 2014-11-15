% Skrypt służący do wstępnej symulacji obiektu układu zbiorników rozpatrywanego
% w zadaniu 13. w projekcie z przedmiotu Sztuczna Inteligencja w Automatyce.

% Zmienna typu string z nazwą pliku do którego zostaną zapisane kolejne pomiary.
filename = 'obiekt_zmiana_wejscia.mat';

% Inicjalizacja komórki zawierającej wyniki kolejnych symulacji.
trajectoriesPlotArguments = {...
   struct( 'Legend', 'wejscie', ...
           'PlotType', 'stairs', ...
           'LineStyle', '-', ...
           'Color', [0; 0; 1] ), ...
   struct( 'Legend', 'nieliniowy', ...
           'LineStyle', '-', ...
           'PlotType', 'plot', ...
           'Color', [0; 0; 1] ), ...
};

% TODO
% Opcjonalna edycja opcji wykresu.

simulations = {...
   struct( 'PlotFontName', 'Liberation Mono', ...
           'PlotFontSize', '10', ...
           'AxesFontName', 'Liberation Mono', ...
           'AxesFontSize', '12' );
};

for i = 1 : length( trajectoriesPlotArguments )
   simulations{2,i} = trajectoriesPlotArguments{i};
end

save(filename, 'simulations');
clear simulations trajectoriesPlotArgument;

% Zmienna globalna odnosząca się do struktury opisującej parametry
% zbiorników w treści zadania. Posiada trzy pola, każde z nich jest
% kolejną strukturą:
%    - sstate - zawiera dane określające podany w zadaniu punkt pracy
%    - const - zawiera stałe opisujące obiekt ( stałe proporcjonalne
%              do przepływu, pole przekroju zbiornika o prostych ścianach
%    - units - zawiera jednostki każdej z wielkości opisujących obiekt
global tanks;

% Wczytywanie pamametrów obiektu. Zmienna wczytywana z tego pliku jest
% zmienną globalną 'tanks'.
load( 'tanksParameters.mat' );

% Krok symulacji. Obiekt opisany jest zestawem nieliniowych, a później
% liniowych ciągłych równań różniczkowych, dlatego niezbędne dla solwerów
% jest ustawienie kroku, z jakim odbywać się będzie symulacja tych równań.

% TODO:
% Zgodnie z zaleceniami Pana Marusaka warto by w regulatorze wykorzystywać model
% dyskretny pozyskany przy okresie próbkowania równym 5s - 10s. Obiekt musi być
% próbkowany dokładnie, za pomocą solwera ode45, jednak można wymusić, aby
% wektor czasu składał się z kolejnych chwil próbkowania modelu regulatora.
% Procedura ode45 posługując się zmiennym parametrem kroku zadba o to, żeby
% pomiary obiektu były dokładne.
dt = 5; % sekundy

% Czas symulacji, wektor z kolejnymi chwilami.
time = [ 0 : dt : 3000 ];
simulationLength = length(time);

% Trajektorie zmiennej wejściowej obiektu - strumienia dopływającej wody.
% Kolumny macierzy są kolejnymi trajektoriami zmieniającego się strumienia
% wody dopływającej.
inputValues = [ 0.55, 0.7, 0.85, 1, 1.15, 1.3, 1.45 ];
inputTrajectories = tanks.sstate.F1 * ones( simulationLength, ...
                                            length( inputValues ) );
for i = 1 : length( inputValues )
   for j = 100 / dt : simulationLength
      inputTrajectories(j,i) = inputValues(i)*inputTrajectories(j,i);
   end
end

% Wejście obiektu jest opóźnione o czas 'tau = 60s', dlatego konieczne jest
% wprowadzenie bufora opóźniającego wpływ sygnału wejściowego.
% Bufor ten jest wykorzystywany równolegle przez wszystkie modele.
inputBuffer = tanks.sstate.F1*ones( tanks.const.tau / dt, 1 );

% Zmienne obiektu rzeczywistego opisanego układem nieliniowych równań
% różniczkowych. Wymaganymi zmiennymi do symulacji jest wektor zmiennych stanu
% i wartość wyjścia modelu.
tanksStateREAL = [ tanks.sstate.V1, tanks.const.C2*(tanks.sstate.h2)^2 ];
tanksOutputREAL = tanks.sstate.h2;

% Wektory wynikowe pojedynczych symulacji, do zapisu do pliku.
plotInput = zeros( simulationLength, 1 );
plotOutputREAL = zeros( simulationLength, 1 );

% ======== ZMIENNE WYŚWIETLAJĄCE POSTĘP SYMULACJI ================= %
simulationStep = 0;                                                 %
lettersWritten = 0;                                                 %
finishTime = simulationLength;                                      %
fprintf('\n');                                                      %
% ================================================================= %

opts = odeset( 'RelTol', 1e-3, 'AbsTol', 1e-6);

for i = 1 : length( inputValues )
   fprintf('Simulation %d out of %d.', i, length( inputValues ) );

   inputBuffer = tanks.sstate.F1*ones( tanks.const.tau / dt, 1 );
   tanksStateREAL = [ tanks.sstate.V1, tanks.const.C2*(tanks.sstate.h2)^2 ];
   tanksOutputREAL = tanks.sstate.h2;

   plotInput = zeros( simulationLength, 1 );
   plotOutputREAL = zeros( simulationLength, 1 );
   
   inputTrajectory = inputTrajectories(:,i);
   disturbanceTrajectory = tanks.sstate.FD*ones( size( inputTrajectory ) );

   % ======== ZMIENNE WYŚWIETLAJĄCE POSTĘP SYMULACJI ================= %
   simulationStep = 0;                                                 %
   lettersWritten = 0;                                                 %
   finishTime = simulationLength;                                      %
   fprintf('\n');                                                      %
   % ================================================================= %

   u = tanks.sstate.F1;
   z = tanks.sstate.FD;

   for j = 1 : simulationLength
      % ======== ZMIENNE WYŚWIETLAJĄCE POSTĘP SYMULACJI ================= %
      if j > simulationStep*(finishTime/100)                              %
         while lettersWritten > 0                                         %
            fprintf('\b');                                                %
            lettersWritten = lettersWritten - 1;                          %
         end                                                              %
         lettersWritten = fprintf('Progress:    %d%%',simulationStep);    %
         simulationStep = simulationStep + 1;                             %
      end                                                                 %
      % ================================================================= %
   
      t = [ time(j), time(j) + dt ];
   
      [ tout, xout ] = ode45( @tanksFunction, t, tanksStateREAL, opts, u, z );
      tanksStateREAL = xout( size(xout, 1), : );

      u = inputBuffer( length(inputBuffer) );
      z = disturbanceTrajectory(j);
      inputBuffer = [ inputTrajectory(j); inputBuffer(1 : length(inputBuffer)-1 ) ];
   
      plotInput(j) = inputTrajectory(j); 
      plotOutputREAL(j) = sqrt( tanksStateREAL(2) / tanks.const.C2 );
      
   end
   load( filename, 'simulations' );
   rows = size( simulations, 1 );
   simulations{rows+1, 1} = plotInput;
   simulations{rows+1, 2} = plotOutputREAL;
   save( filename, 'simulations' );

   fprintf('\n\n');
end

fprintf('All simulations completed!\n');

%% Rysowanie wyników symulacji

filename = 'obiekt_zmiana_wejscia.mat';
load(filename);
dt = 5;
time = [ 0 : dt : 3000 ];

for i = 3 : size( simulations, 1 )
   tmp = simulations{i,1};
   tmp = tmp( length(tmp) );
   labels{i-2} = num2str( tmp );
end

addpath('customToolbox');
colors = distinguishable_colors( length(labels) );
rmpath('customToolbox');

figure(1);
hold on;
grid on;
for i = 3 : size( simulations, 1 )
   plot( time, simulations{i,2}, ...
         'Color', colors(i-2,:),...
         'LineStyle', simulations{2,2}.LineStyle );
end
legend( labels, 'Location', 'Best');

figure(2);
hold on;
grid on;
for i = 3 : size( simulations, 1 )
   stairs( time, simulations{i,1}, ...
           'Color', colors(i-2,:),...
           'LineStyle', simulations{2,1}.LineStyle );
end
legend( labels, 'Location', 'Best');
