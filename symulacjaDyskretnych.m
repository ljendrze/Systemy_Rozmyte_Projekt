% Projekt dotyczący systemów rozmytych z przedmiotu Sztuczna Inteligencja w
% Automacyte. Skrypt służy do porównania działania dyskretnego modelu liniowego
% na tle ciągłego modelu liniowego i ciągłego modelu opartego o nieliniowe
% równania różniczkowe.

% Zmienna typu string z nazwą pliku do którego zostaną zapisane kolejne pomiary.
filename = 'porownanie_ciaglych_i_dyskretnych.mat';

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
   struct( 'Legend', 'dyskretny', ...
           'LineStyle', '-', ...
           'PlotType', 'stairs', ...
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
global linearModel;

% Wczytywanie pamametrów obiektu. Zmienna wczytywana z tego pliku jest
% zmienną globalną 'tanks'.
load( 'tanksParameters.mat' );

load( 'modele/linearModel.mat' );

% Krok symulacji. Obiekt opisany jest zestawem nieliniowych, a później
% liniowych ciągłych równań różniczkowych, dlatego niezbędne dla solwerów
% jest ustawienie kroku, z jakim odbywać się będzie symulacja tych równań.
dt = 1;

% Okres dyskretyzacji.
Tp = 5;

% Czas symulacji, wektor z kolejnymi chwilami.
time = [ 0 : dt : 1000 ];
simulationLength = length(time);
% simulationLength = 100;

% Trajektorie zmiennej wejściowej obiektu - strumienia dopływającej wody.
% Kolumny macierzy są kolejnymi trajektoriami zmieniającego się strumienia
% wody dopływającej.
inputValues = [ 0.55, 0.7, 0.85, 1, 1.15, 1.3, 1.45 ];
inputTrajectories = tanks.sstate.F1 * ones( simulationLength, ...
                                            length( inputValues ) );
for i = 1 : length( inputValues )
   for j = 10 / dt : simulationLength
      inputTrajectories(j,i) = inputValues(i)*inputTrajectories(j,i);
   end
end

% Wejście obiektu jest opóźnione o czas 'tau = 60s', dlatego konieczne jest
% wprowadzenie bufora opóźniającego wpływ sygnału wejściowego.
% Bufor ten jest wykorzystywany równolegle przez wszystkie modele.

% ======== ZMIENNE WYŚWIETLAJĄCE POSTĘP SYMULACJI ================= %
simulationStep = 0;                                                 %
lettersWritten = 0;                                                 %
finishTime = simulationLength;                                      %
fprintf('\n');                                                      %
% ================================================================= %

opts = odeset( 'RelTol', 1e-3, 'AbsTol', 1e-6);

for i = 1 : length( inputValues )
   fprintf('Simulation %d out of %d.', i, length( inputValues ) );

   % Wejście obiektu jest opóźnione o czas 'tau = 60s', dlatego konieczne jest
   % wprowadzenie bufora opóźniającego wpływ sygnału wejściowego.
   % Bufor ten jest wykorzystywany równolegle przez wszystkie modele.
   inputBuffer = tanks.sstate.F1*ones( tanks.const.tau / dt, 1 );

   % Zmienne obiektu rzeczywistego opisanego układem nieliniowych równań
   % różniczkowych. Wymaganymi zmiennymi do symulacji jest wektor zmiennych stanu
   % i wartość wyjścia modelu.
   tanksStateNONLINEAR = [ tanks.sstate.V1, tanks.const.C2*(tanks.sstate.h2)^2 ];
   tanksOutputNONLINEAR = tanks.sstate.h2;

   % Zmienne modelu opisanego układem liniowych równań różniczkowych.
   % Wymaganymi zmiennymi do symulacji jest wektor zmiennych stanu
   % i wartość wyjścia modelu.
   tanksOutputCONTINUOUS = 0;
   tanksStateCONTINUOUS = zeros(2,1);

   % Zmienne modelu opisanego równaniem różnicowym uzyskanym dla liniowego
   % modelu dyskretnego z okresem dyskretyzacji równym Tp.
   tanksOutputPastDIGITAL = [ 0; 0 ];
   tanksInputPastDIGITAL = [ 0; 0 ];
   tanksOutputDIGITAL = 0;

   % Wektory wynikowe pojedynczych symulacji, do zapisu do pliku.
   plotInput = zeros( simulationLength, 1 );
   plotOutputNONLINEAR = zeros( simulationLength, 1 );
   plotOutputCONTINUOUS = tanks.sstate.h2*ones( simulationLength, 1 );
   plotOutputDIGITAL = tanks.sstate.h2*ones( simulationLength, 1 );

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
   uLin = u - tanks.sstate.F1;
   zLin = z - tanks.sstate.FD;

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
   
      [ tout, xout ] = ode45( @tanksFunction, t, tanksStateNONLINEAR, opts, u, z );
      tanksStateNONLINEAR = xout( size(xout, 1), : );

      [ tout, xout ] = ode45( @linearTanksFunction, t, tanksStateCONTINUOUS, opts, uLin, zLin );
      tanksStateCONTINUOUS = xout( size(xout, 1), : );
      tanksOutputCONTINUOUS = linearModel.C * tanksStateCONTINUOUS';

      if mod(j,Tp) == 0
         tanksOutputDIGITAL = sum( linearModel.b .* tanksOutputPastDIGITAL ) + ...
            sum( linearModel.c .* tanksInputPastDIGITAL );
         tanksOutputPastDIGITAL = [ tanksOutputDIGITAL; tanksOutputPastDIGITAL(1) ];
         tanksInputPastDIGITAL = [ inputBuffer( length(inputBuffer) ) - tanks.sstate.F1;...
            tanksInputPastDIGITAL(1) ];
      end

      u = inputBuffer( length(inputBuffer) );
      z = disturbanceTrajectory(j);
      uLin = u - tanks.sstate.F1;
      zLin = disturbanceTrajectory(j) - tanks.sstate.FD;
      inputBuffer = [ inputTrajectory(j); inputBuffer(1 : length(inputBuffer)-1 ) ];

      % fprintf( 'j=   %d;   uLin=   %f;   [ %f, %f ]\n',j,uLin,tanksInputPastDIGITAL(1),tanksInputPastDIGITAL(2));
      % fprintf( 'OK:   %d; j=   %d;   yCon=   %f;   yDig=   %f;   [ %f, %f ]\n',mod(j,Tp)==4,j,tanksOutputCONTINUOUS,tanksOutputDIGITAL,tanksOutputPastDIGITAL(1),tanksOutputPastDIGITAL(2));
      % pause;
      
      plotInput(j) = inputTrajectory(j); 
      plotOutputNONLINEAR(j) = sqrt( tanksStateNONLINEAR(2) / tanks.const.C2 );
      plotOutputCONTINUOUS(j) = plotOutputCONTINUOUS(j) + tanksOutputCONTINUOUS;
      plotOutputDIGITAL(j) = plotOutputDIGITAL(j) + tanksOutputDIGITAL;
 
   end
   load( filename, 'simulations' );
   rows = size( simulations, 1 );
   simulations{rows+1, 1} = plotInput;
   simulations{rows+1, 2} = plotOutputNONLINEAR;
   simulations{rows+1, 3} = plotOutputCONTINUOUS;
   simulations{rows+1, 4} = plotOutputDIGITAL;
   save( filename, 'simulations' );

   fprintf('\n\n');
end

fprintf('All simulations completed, drawing plots...\n');
rysujWykresySymulacyjne( filename );
fprintf('Done!\n');

