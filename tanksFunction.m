function dxdt = tanksFunction( time, state, input, disturbance )
% Równania różniczkowe funkcji opisujących zachowanie kaskady zbiorników
% doprowadzone do postaci odpowiedniej dla procedury ode45.
% Zbiornik posiada opóźnione wejście, dlatego zanim wywołana zostanie
% procedura ode45 najpierw należy wykorzystać bufor o odpowiedniej długości
% do symulowania opóźniania sygnału.

global tanks;

dxdt = [ ...
   input ...
      + disturbance ...
      - tanks.const.alpha1 * sqrt( state(1) / tanks.const.A1);
   tanks.const.alpha1 * sqrt( state(1) / tanks.const.A1 ) ...
      - tanks.const.alpha2 * ( state(2) / tanks.const.C2 )^(1/4);
];
