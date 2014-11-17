% Skrypt służący do utworzenia modelu rozmytego obiektu rozpatrywanego w zadaniu 13. 
% w projekcie z przedmiotu Sztuczna Inteligencja w Automatyce.

global tanks;
load( 'tanksParameters.mat' );

% Przygotowanie struktury w której zapisane zostaną parametry modelu rozmytego.
fuzzyModel = cell(0);

% Nazwa pliku wyjściowego dla uzyskanego modelu rozmytego.
filename = 'modele/trapezoidFuzzyModel.mat';
fprintf('\nOutput file name:   %s\n\n',filename );

% Czas próbkowania obiektu nie musi być dokładny, zależy nam na stanie ustalonym,
% dla którego zostanie pozyskany liniowy model lokalny. Nie interesuje nas dynamika
% obiektu. Wartość podana w sekundach
dt = 20;
time = [ 0 : dt : 3000 ];
simulationLength = length(time);

% Czas próbkowania z którym zdyskretyzowany zostanie liniowy model ciągły.
Tp = 5;

% Wartości sterowania z punktów pracy wokół których pozyskane zostaną modele
% lokalne rozmytego modelu Takagi-Sugeno.
controlValues = [ 50, 75, 100, 125, 150 ];
outputSSValues = zeros( length( controlValues ), 1 );

% Bufor opóźniający wejście.
inputBuffer = tanks.sstate.F1 * ones( tanks.const.tau / dt, 1 );

% Opcje dla solwera ode45. Nie potrzeba dokładniejszych, przyjęte są wartości
% domyślne metody.
opts = odeset('RelTol', 1e-3, 'AbsTol', 1e-6);

% Główna pętla skrypty - pozyskiwanie modeli lokalnych w każdym z punktów
% pracy.
for j = 1 : length( controlValues )

   % Inicjalizacja wektorów wykorzystywanych w każdej kolejnej symulacji.
   inputTrajectory = controlValues(j) * ones( simulationLength, 1 );
   disturbanceTrajectory = tanks.sstate.FD * ones( simulationLength, 1 );
   inputBuffer = tanks.sstate.F1 * ones( tanks.const.tau / dt, 1 );
   x0 = [ tanks.sstate.V1, tanks.const.C2*(tanks.sstate.h2)^2 ];

   fprintf('Simulation %d out of %d.', j, length( controlValues ) );
   % ======== ZMIENNE WYŚWIETLAJĄCE POSTĘP SYMULACJI ================= %
   simulation_step = 0;                                                %
   letters_written = 0;                                                %
   finish_time = simulationLength;                                     %
   fprintf('\n');                                                      %
   % ================================================================= %

   % Symulacja obiektu i uzyskanie ustabilizowanej wartości wyjścia dla zadanej
   % wartości wejścia.
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
   
      [ tout, xout ] = ode45( @tanksFunction, timeInterval, x0, opts, u, z );
      x0 = xout( size(xout, 1), : );
     
      y = sqrt( x0(2) / tanks.const.C2 );
   end
   fprintf('\n\n');

   % Zapisywanie wartości wejścia, zakłócenia, wyjścia i stanu do odpowiednich
   % komórek. Informacje te zostaną zapisane w strukturach modeli lokalnych
   % modelu rozmytego.
   inputs{j} = u;
   disturbances{j} = z;
   outputs{j} = y;
   tanksStates{j} = x0;

   % Linearyzacja modelu nieliniowego w określonym punkcie pracy (u,y).
   % Elementy macierzy A,B,C zostały obliczone analitycznie i do wzorów
   % podstawiane są wartości odpowiadające punktom pracy.
   A_ss = [ 
      -tanks.const.alpha1*0.5*sqrt( 1 / ( tanks.const.A1*x0(1) ) ),...
      0;
      tanks.const.alpha1*0.5*sqrt( 1 / ( tanks.const.A1*x0(1) ) ),...
      -0.25*tanks.const.alpha2*( tanks.const.C2*( x0(2) )^3 )^(-1/4);
   ];
   B_ss = [ 1 ; 0 ];
   C_ss = [ 0, 0.5*( 1 / sqrt( tanks.const.C2*x0(2) ) ); ];

   % Odpowiednie elementy macierzy modelu liniowego zostają przetworzone do postaci
   % transmitancyjnej. Poniższe wzory także zostały wyznaczone analitycznie dla obiektu.
   num = [ A_ss(2,1)*B_ss(1,1)*C_ss(1,2) ];
   den = [ 1, ( - A_ss(1,1) - A_ss(2,2) ), A_ss(1,1)*A_ss(2,2) ];
   tf_sys = tf( num, den, 'InputDelay', tanks.const.tau );

   % Dyskretyzacja transmitancji ciągłej. Wykorzystywana jest metoda zero-order-hold,
   % i czas dyskretyzacji Tp podany na początku skryptu. Elementy num_d i den_d
   % zawierają wektory współczynników licznika wielomianów licznika i mianownika
   % transmitancji dyskretnej.
   tf_discrete = c2d( tf_sys, Tp, 'zoh' );
   num_d = tf_discrete.num{1};
   den_d = tf_discrete.den{1};

   % Model rozmyty dostosowany do algorytmu GPC, tzn. jest to model w postaci
   % równania różnicowego danego wzorem:
   %
   %    y(k+1) = b1*y(k) + b2*y(k-1) + ... + bn*y(k-n+1) + 
   %             c1*u(k) + c2*u(k-1) + ... + cm*y(k-m+1)

   % Zakłada się, że transmitancja jest transmitancją właściwą, tj. 
   % rank(num) < rand(den)   
   b = zeros( length(den_d) - 1, 1 );
   for i = 1 : length(den_d) - 1
      b(i) = - den_d(i+1) / den_d(1);
   end
   A{j} = b;

   c = zeros( length(num_d) - 1 + tf_discrete.InputDelay, 1 );
   for i = tf_discrete.InputDelay + 1 : length(num_d) - 1 + tf_discrete.InputDelay;
      c(i) = num_d( i - tf_discrete.InputDelay + 1 ) / den_d(1);
   end
   B{j} = c;

   % Pozyskiwanie odpowiedzi skokowej modelu lokalnego.
   inputPast = zeros( size( B{j} ) );
   outputsPast = zeros( length( A{j} ), 1 );
   
   D = 500;
   
   inputPast(1) = 1;
   for i = 1 : D
      S{j}(i) = sum( A{j} .* outputsPast ) + sum( B{j} .* inputPast );
      outputsPast = [ S{j}(i); outputsPast(1) ];
      inputPast = [ 1 ; inputPast( 1 : length(inputPast) - 1 ) ];
   end
end

% Typy funkcji przynależności poprzedników rozmytych.
mftypes = {'trapmf'; 'trimf'; 'trimf'; 'trimf'; 'trapmf'};

% Parametry funkcji przynależności poprzedników.
mfparams = {...
   [ -Inf; -Inf; outputs{1}; outputs{2} ];...
   [ outputs{1}; outputs{2}; outputs{3} ];...
   [ outputs{2}; outputs{3}; outputs{4} ];...
   [ outputs{3}; outputs{4}; outputs{5} ];...
   [ outputs{4}; outputs{5}; Inf; Inf ];...
};

% Tworzenie struktury modelu rozmytego do zapisu. Każda z struktur modeli
% lokalnych zawiera poniższe parametry:
%   - A - wektor wielomianu odpowiadającego poprzednim wartościom wyjść
%         w równaniu różnicowym;
%   - B - wektor wielomianu odpowiadającego poprzednim wartościom wejść
%         w równaniu różnicowym;
%   - u0 - wartość wejścia z punktu pracy pozyskanego modelu lokalnego;
%   - x0 - wektor stanu z punktu pracy pozyskanego modelu lokalnego;
%   - y0 - wartość wyjścia z punktu pracy pozyskanego modelu lokalnego;
%   - MFType - rodzaj funkcji przynależności (string);
%   - MFParams - wektor parametrów funkcji przynależności
for j = 1 : length(controlValues)
   fuzzyModel{j,1} = struct( 'A', A{j}, 'B', B{j}, 'S', S{j},...
                           'u0', inputs{j}, 'y0', outputs{j}, 'x0', tanksStates{j}, ...
                           'MFType', mftypes{j}, 'MFParams', mfparams{j} );
end

save( filename, 'fuzzyModel' );
fprintf( 'Done!\n\n' );
