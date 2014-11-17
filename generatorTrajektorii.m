% Skrypt służący do wstępnej symulacji obiektu układu zbiorników rozpatrywanego
% w zadaniu 13. w projekcie z przedmiotu Sztuczna Inteligencja w Automatyce.

addpath('../customToolbox/fuzzy/');

% Zmienna typu string z nazwą pliku do którego zostaną zapisane kolejne pomiary.
filename = 'trajektorie3.mat';

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

L = 20;
dl = 20;

% TODO:
% Zgodnie z zaleceniami Pana Marusaka warto by w regulatorze wykorzystywać model
% dyskretny pozyskany przy okresie próbkowania równym 5s - 10s. Obiekt musi być
% próbkowany dokładnie, za pomocą solwera ode45, jednak można wymusić, aby
% wektor czasu składał się z kolejnych chwil próbkowania modelu regulatora.
% Procedura ode45 posługując się zmiennym parametrem kroku zadba o to, żeby
% pomiary obiektu były dokładne.
dt = 5; % sekundy

% Czas symulacji, wektor z kolejnymi chwilami.
time = [ 0 : dt : L*dl*dt ];
time = time( 1 : length(time)-1 );
simulationLength = length(time);

minimalInput = 50;
maximalInput = 150;
inputTrajectory = 100*ones( L*dl, 1 );

for i = 2 : L
   val = rand*100 + 50;
   for j = 1 : dl
      inputTrajectory( (i-1)*dl + j ) = val;
   end
end

% Wejście obiektu jest opóźnione o czas 'tau = 60s', dlatego konieczne jest
% wprowadzenie bufora opóźniającego wpływ sygnału wejściowego.
% Bufor ten jest wykorzystywany równolegle przez wszystkie modele.
inputBuffer = tanks.sstate.F1*ones( tanks.const.tau / dt, 1 );

% ZMIENNE OBIEKTU RZECZYWISTEGO
% Zmienne obiektu rzeczywistego opisanego układem nieliniowych równań
% różniczkowych. Wymaganymi zmiennymi do symulacji jest wektor zmiennych stanu
% i wartość wyjścia modelu.
tanksStateREAL = [ tanks.sstate.V1, tanks.const.C2*(tanks.sstate.h2)^2 ];
tanksOutputREAL = tanks.sstate.h2;

opts = odeset( 'RelTol', 1e-3, 'AbsTol', 1e-6);

inputBuffer = tanks.sstate.F1*ones( tanks.const.tau / dt, 1 );
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
u = tanks.sstate.F1;

for i = 1 : simulationLength
   % ======== ZMIENNE WYŚWIETLAJĄCE POSTĘP SYMULACJI ================= %
   if i > simulationStep*(finishTime/100)                              %
      while lettersWritten > 0                                         %
         fprintf('\b');                                                %
         lettersWritten = lettersWritten - 1;                          %
      end                                                              %
      lettersWritten = fprintf('Progress:    %d%%',simulationStep);    %
      simulationStep = simulationStep + 1;                             %
   end                                                                 %
   % ================================================================= %

   t = [ time(i), time(i) + dt ];

   [ tout, xout ] = ode45( @tanksFunction, t, tanksStateREAL, opts, u, tanks.sstate.FD );
   tanksStateREAL = xout( size(xout, 1), : );

   u = inputBuffer( length(inputBuffer) );
   inputBuffer = [ inputTrajectory(i); inputBuffer(1 : length(inputBuffer)-1 ) ];

   plotInput(i) = inputTrajectory(i); 
   plotOutputREAL(i) = sqrt( tanksStateREAL(2) / tanks.const.C2 );
end

trajectories{1} = plotInput;
trajectories{2} = plotOutputREAL;
save( filename, 'trajectories' );

