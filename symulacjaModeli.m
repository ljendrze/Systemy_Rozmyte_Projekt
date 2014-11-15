% Skrypt służący do wstępnej symulacji obiektu układu zbiorników rozpatrywanego
% w zadaniu 13. w projekcie z przedmiotu Sztuczna Inteligencja w Automatyce.

% Zmienna typu string z nazwą pliku do którego zostaną zapisane kolejne pomiary.
filename = 'obiekt_model_liniowy_i_rozmyty.mat';

% Inicjalizacja komórki zawierającej wyniki kolejnych symulacji.
simulations = cell(0);

save(filename, 'simulations');
clear simulations;

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

load( 'linearModel.mat' );

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
dt = 1; % sekundy

% Czas symulacji, wektor z kolejnymi chwilami.
time = [ 0 : dt : 1000 ];
simulationLength = length(time);

% Trajektorie zmiennej wejściowej obiektu - strumienia dopływającej wody.
% Kolumny macierzy są kolejnymi trajektoriami zmieniającego się strumienia
% wody dopływającej.
inputValues = [ 0.5, 0.6, 0.7, 0.8, 0.9, 1, 1.1, 1.2, 1.3, 1.4, 1.5 ];
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
% TODO
% inputBuffer musi zostać zresetowany przed każdą kolejną dużą iteracją
% symulacji

% ZMIENNE OBIEKTU RZECZYWISTEGO
% Zmienne obiektu rzeczywistego opisanego układem nieliniowych równań
% różniczkowych. Wymaganymi zmiennymi do symulacji jest wektor zmiennych stanu
% i wartość wyjścia modelu.
tanksStateREAL = [ tanks.sstate.V1, tanks.const.C2*(tanks.sstate.h2)^2 ];
tanksOutputREAL = tanks.sstate.h2;

% Macierze modelu liniowego.
% TODO
% Sprawdzić czy macierz A jest poprawna - wzór został skrócony.
A = [ 
   -tanks.const.alpha1*0.5*sqrt( 1 / ( tanks.const.A1*tanks.sstate.V1 ) ),...
   0;
   tanks.const.alpha1*0.5*sqrt( 1 / ( tanks.const.A1*tanks.sstate.V1 ) ),...
   -0.25*tanks.const.alpha2*( tanks.const.C2*( tanks.const.C2*( tanks.sstate.h2 )^2 )^3 )^(-1/4);
];
B = [ 1 ; 0 ];
C = [ 0, 0.5*( 1 / ( tanks.const.C2*tanks.sstate.h2 ) ); ];

% Zmienne modelu opisanego układem liniowych równań różniczkowych.
% Wymaganymi zmiennymi do symulacji jest wektor zmiennych stanu
% i wartość wyjścia modelu.
tanksStateLINEAR = [ 0; 0 ];
tanksOutputLINEAR = 0;

% Zmienne modelu rozmytego: wektor przeszłych wartości wejścia i przeszłych
% wartości wyjścia.
tanksOutputFUZZY = 0;
tanksInputPastFUZZY = zeros(2,1);
tanksOutputPastFUZZY = zeros(2,1);

% Wektory wynikowe pojedynczych symulacji, do zapisu do pliku.
plotInput = zeros( simulationLength, 1 );
plotOutputREAL = zeros( simulationLength, 1 );
plotOutputLINEAR = zeros( simulationLength, 1 );
plotOutputFUZZY = zeros( simulationLength, 1 );

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
   tanksOutputLINEAR = 0;
   tanksInputPastLINEAR = zeros(2,1);
   tanksOutputPastLINEAR = zeros(2,1);
   tanksOutputFUZZY = tanks.sstate.h2;
   tanksInputPastFUZZY = tanks.sstate.F1 * ones(2,1);
   tanksOutputPastFUZZY = tanks.sstate.h2 * ones(2,1);

   plotInput = zeros( simulationLength, 1 );
   plotOutputREAL = zeros( simulationLength, 1 );
   plotOutputLINEAR = tanks.sstate.h2*ones( simulationLength, 1 );
   plotOutputFUZZY = tanks.sstate.h2*ones( simulationLength, 1 );
   
   inputTrajectory = inputTrajectories(:,i);
   % disturbanceTrajectory = disturbanceTrajectories(:,i);
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
   
      u = inputBuffer( length(inputBuffer) );
      z = disturbanceTrajectory(j);

      tanksInputPastLINEAR = [ u - linearModel.x0; tanksInputPastLINEAR(1) ];
      tanksOutputPastLINEAR = [ tanksOutputLINEAR; tanksOutputPastLINEAR(1) ];

      tanksInputPastFUZZY = [ u; tanksInputPastFUZZY(1) ];
      tanksOutputPastFUZZY = [ tanksOutputFUZZY; tanksOutputPastFUZZY(1) ];
      
      uLin = u - tanks.sstate.F1;
      zLin = disturbanceTrajectory(j) - tanks.sstate.FD;

      inputBuffer = [ inputTrajectory(j); inputBuffer(1 : length(inputBuffer)-1 ) ];
   
      [ tout, xout ] = ode45( @tanksFunction, t, tanksStateREAL, opts, u, z );
      tanksStateREAL = xout( size(xout, 1), : );
   
      tanksOutputLINEAR = - linearModel.den(2)*tanksOutputPastLINEAR(1) - linearModel.den(3)*tanksOutputPastLINEAR(2)...
        + linearModel.num(2)*tanksInputPastLINEAR(1) + linearModel.num(3)*tanksInputPastLINEAR(2);

      [ tanksOutputFUZZY, mfunvalues ] ...
         = evaluateFuzzyModel( tanksInputPastFUZZY, tanksOutputPastFUZZY );

      % display(tanksOutputLINEAR);
      % display(tanksOutputFUZZY);
      % pause;
      % display( mfunvalues );
      % pause;

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

