% Skrypt służący do wstępnej symulacji obiektu układu zbiorników rozpatrywanego
% w zadaniu 13. w projekcie z przedmiotu Sztuczna Inteligencja w Automatyce.

addpath('customToolbox/fuzzy');

% Zmienna typu string z nazwą pliku do którego zostaną zapisane kolejne pomiary.
filename = 'porownanie_liniowego_i_dyskretnego.mat';

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
   struct( 'Legend', 'liniowy', ...
           'LineStyle', '-', ...
           'PlotType', 'plot', ...
           'Color', [1; 0; 0] ), ...
   struct( 'Legend', 'rozmyty', ...
           'LineStyle', '-', ...
           'PlotType', 'plot', ...
           'Color', [0; 1; 0] ) ...
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

% Wczytywanie modelu rozmytego. Jego parametry są zapisane w formie macierzy
% komórkowej w której każdy model lokalny stanowi kolejny element macierzy 
% (właściwie wektora). 
importedData = load( 'modele/trapezoidFuzzyModel.mat' );
fuzzyModel = importedData.fuzzyModel;

load( 'modele/linearModel.mat' );

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
time = [ 0 : dt : 2000 ];
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

% ======== ZMIENNE WYŚWIETLAJĄCE POSTĘP SYMULACJI ================= %
simulationStep = 0;                                                 %
lettersWritten = 0;                                                 %
finishTime = simulationLength;                                      %
fprintf('\n');                                                      %
% ================================================================= %

opts = odeset( 'RelTol', 1e-3, 'AbsTol', 1e-6);

for i = 1 : length( inputValues )
   fprintf('Simulation %d out of %d.', i, length( inputValues ) );

   % Zmienne obiektu rzeczywistego opisanego układem nieliniowych równań
   % różniczkowych. Wymaganymi zmiennymi do symulacji jest wektor zmiennych stanu
   % i wartość wyjścia modelu.
   u = tanks.sstate.F1;
   z = tanks.sstate.FD;
   tanksStateREAL = [ tanks.sstate.V1, tanks.sstate.V2 ];
   tanksOutputREAL = tanks.sstate.h2;
   % Wejście obiektu jest opóźnione o czas 'tau = 60s', dlatego konieczne jest
   % wprowadzenie bufora opóźniającego wpływ sygnału wejściowego.
   % Bufor ten jest wykorzystywany równolegle przez model nieliniowy.
   inputBuffer = tanks.sstate.F1*ones( tanks.const.tau / dt, 1 );

   % Zmienne modelu opisanego układem liniowych równań różnicowych.
   % Wymaganymi zmiennymi do symulacji jest wektor zmiennych stanu
   % i wartość wyjścia modelu, a także wektory przechowujące informacje
   % o przeszłych wartościach wejścia i wyjścia.
   tanksOutputLINEAR = 0;
   tanksInputPastLINEAR = zeros( length(linearModel.B), 1 ); % 14x1
   tanksOutputPastLINEAR = zeros( length(linearModel.A), 1 ); % 2x1

   % Zmienne modelu rozmytego: wektor przeszłych wartości wejścia i przeszłych
   % wartości wyjścia.
   tanksOutputFUZZY = tanks.sstate.h2;
   tanksInputPastFUZZY = tanks.sstate.F1 * ones( length(fuzzyModel{1}.B), 1 ); % 14x1
   tanksOutputPastFUZZY = tanks.sstate.h2 * ones( length(fuzzyModel{1}.A), 1 ); % 2x1
   
   % Wektory wynikowe pojedynczych symulacji, do zapisu do pliku.
   plotInput = zeros( simulationLength, 1 );
   plotOutputREAL = zeros( simulationLength, 1 );
   plotOutputLINEAR = tanks.sstate.h2 * ones( simulationLength, 1 );
   plotOutputFUZZY = zeros( simulationLength, 1 );

   inputTrajectory = inputTrajectories(:,i);
   disturbanceTrajectory = tanks.sstate.FD*ones( size( inputTrajectory ) );

   % ======== ZMIENNE WYŚWIETLAJĄCE POSTĘP SYMULACJI ================= %
   simulationStep = 0;                                                 %
   lettersWritten = 0;                                                 %
   finishTime = simulationLength;                                      %
   fprintf('\n');                                                      %
   % ================================================================= %

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
   
      tanksOutputLINEAR = sum( linearModel.A .* tanksOutputPastLINEAR ) + ...
         sum( linearModel.B .* tanksInputPastLINEAR );
      tanksOutputPastLINEAR = [ tanksOutputLINEAR; ...
         tanksOutputPastLINEAR( 1 : length(tanksOutputPastLINEAR) - 1 ) ];
      tanksInputPastLINEAR = [ inputTrajectory(j) - tanks.sstate.F1;...
         tanksInputPastLINEAR( 1 : length(tanksInputPastLINEAR) -1 ) ];

      [ localOutputs, weights ] = evaluateFuzzyModelGPC( fuzzyModel, ...
         tanksOutputFUZZY, tanksInputPastFUZZY, tanksOutputPastFUZZY );
      tanksOutputFUZZY = sum( weights .* localOutputs );
      tanksInputPastFUZZY = [ inputTrajectory(j); ...
         tanksInputPastFUZZY( 1 : length(tanksInputPastFUZZY) - 1 ) ];
      tanksOutputPastFUZZY = [ tanksOutputFUZZY; tanksOutputPastFUZZY(1) ];

      plotInput(j) = inputTrajectory(j); 
      plotOutputREAL(j) = sqrt( tanksStateREAL(2) / tanks.const.C2 );
      plotOutputLINEAR(j) = plotOutputLINEAR(j) + tanksOutputLINEAR;
      plotOutputFUZZY(j) = tanksOutputFUZZY;
      
   end
   load( filename, 'simulations' );
   rows = size( simulations, 1 );
   simulations{rows+1, 1} = plotInput;
   simulations{rows+1, 2} = plotOutputREAL;
   simulations{rows+1, 3} = plotOutputLINEAR;
   simulations{rows+1, 4} = plotOutputFUZZY;
   save( filename, 'simulations' );

   fprintf('\n\n');
end

fprintf('All simulations completed!\n');
rysujWykresySymulacyjne( filename );
fprintf('Done!\n');

rmpath('customToolbox/fuzzy');
