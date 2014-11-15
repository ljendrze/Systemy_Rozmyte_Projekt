function dxdt = linearTanksFunction( time, state, input, disturbance )
% Równania różniczkowe funkcji opisujących zachowanie kaskady zbiorników
% doprowadzone do postaci odpowiedniej dla procedury ode45.
% Zbiornik posiada opóźnione wejście, dlatego zanim wywołana zostanie
% procedura ode45 najpierw należy wykorzystać bufor o odpowiedniej długości
% do symulowania opóźniania sygnału.

global linearModel;

dxdt = [ ...
   linearModel.A(1,1)*state(1) + linearModel.A(1,2)*state(2) + linearModel.B(1)*input;
   linearModel.A(2,1)*state(1) + linearModel.A(2,2)*state(2) + linearModel.B(2)*input;
];
