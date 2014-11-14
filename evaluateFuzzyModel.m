function [ output, mfunvalues ] = evaluateFuzzyModel( prevInputs, prevOutputs )

   global fuzzyModel;
   global tanks;
   realInputValue = prevInputs(1);

   % Niech w funkcji będzie odejmowana wartość wejścia z punktu pracy
   % dla każdego z modeli lokalnych, a także na końcu dodawana wartość
   % wyjścia z punktu pracy dla modelu lokalnego.

   mi_50 = evaluate_mi_50( prevOutputs(1) );
   mi_75 = evaluate_mi_75( prevOutputs(1) );
   mi_100 = evaluate_mi_100( prevOutputs(1) );
   mi_125 = evaluate_mi_125( prevOutputs(1) );
   mi_150 = evaluate_mi_150( prevOutputs(1) );

   u_50 = prevInputs - fuzzyModel.m50.x0;
   u_75 = prevInputs - fuzzyModel.m75.x0;
   u_100 = prevInputs - fuzzyModel.m100.x0;
   u_125 = prevInputs - fuzzyModel.m125.x0;
   u_150 = prevInputs - fuzzyModel.m150.x0;

   %display(u_50); display(u_75); display(u_100); display(u_125); display(u_150); pause;

   y_50 = prevOutputs - fuzzyModel.m50.y0;
   y_75 = prevOutputs - fuzzyModel.m75.y0;
   y_100 = prevOutputs - fuzzyModel.m100.y0;
   y_125 = prevOutputs - fuzzyModel.m125.y0;
   y_150 = prevOutputs - fuzzyModel.m150.y0;

   output_50 = - fuzzyModel.m50.den(2)*y_50(1) - fuzzyModel.m50.den(3)*y_50(2)...
         + fuzzyModel.m50.num(2)*u_50(1) + fuzzyModel.m50.num(3)*u_50(2)...
         + fuzzyModel.m50.y0;
   output_75 = - fuzzyModel.m75.den(2)*y_75(1) - fuzzyModel.m75.den(3)*y_75(2)...
         + fuzzyModel.m75.num(2)*u_75(1) + fuzzyModel.m75.num(3)*u_75(2)...
         + fuzzyModel.m75.y0;
   output_100 = - fuzzyModel.m100.den(2)*y_100(1) - fuzzyModel.m100.den(3)*y_100(2)...
         + fuzzyModel.m100.num(2)*u_100(1) + fuzzyModel.m100.num(3)*u_100(2)...
         + fuzzyModel.m100.y0;
   output_125 = - fuzzyModel.m125.den(2)*y_125(1) - fuzzyModel.m125.den(3)*y_125(2)...
         + fuzzyModel.m125.num(2)*u_125(1) + fuzzyModel.m125.num(3)*u_125(2)...
         + fuzzyModel.m125.y0;
   output_150 = - fuzzyModel.m150.den(2)*y_150(1) - fuzzyModel.m150.den(3)*y_150(2)...
         + fuzzyModel.m150.num(2)*u_150(1) + fuzzyModel.m150.num(3)*u_150(2)...
         + fuzzyModel.m150.y0;

   output = mi_50*output_50 + mi_75*output_75...
      + mi_100*output_100 + mi_125*output_125 + mi_150*output_150;

   mfunvalues = [ mi_50, mi_75, mi_100, mi_125, mi_150 ];

end


% Deklaracje trapezowych i trójkątnych funkcji przynależności.
% Funkcje zależne od wartości wejścia. Model nie jest zbyt dokładny w stosunku
% do modelu nieliniowego opisuwanego przez równania różniczkowe, kiedy sygnał
% wejściowy zmienia się skokowo. Lepiej obliczać siłę odpalenia reguł
% na podstawie wartości wyjścia obiektu - zmienia się w sposób bardziej płynny
% niż skokowe zmiany wejścia, które następują zarówno przy wstępnej symulacji
% obiektu, jak i będą następowały przy regulacji (regulator może sterować
% obiektem w sposób całkiem gwałtowny, co może być niepożądane.

function mfunvalue = evaluate_mi_50( output )
   global fuzzyModel;

   mfunvalue = 0;
   if output <= fuzzyModel.m50.y0
      mfunvalue = 1;
   elseif output > fuzzyModel.m50.y0 && output <= fuzzyModel.m75.y0
      mfunvalue = ( 1 / ( fuzzyModel.m50.y0 - fuzzyModel.m75.y0) )*output...
         - fuzzyModel.m75.y0 / ( fuzzyModel.m50.y0 - fuzzyModel.m75.y0 );
   end
end

function mfunvalue = evaluate_mi_75( output )
   global fuzzyModel;

   mfunvalue = 0;
   if output > fuzzyModel.m50.y0 && output <= fuzzyModel.m75.y0
      mfunvalue = ( 1 / ( fuzzyModel.m75.y0 - fuzzyModel.m50.y0) )*output...
         - fuzzyModel.m50.y0 / ( fuzzyModel.m75.y0 - fuzzyModel.m50.y0 );
   elseif output > fuzzyModel.m75.y0 && output < fuzzyModel.m100.y0
      mfunvalue = ( 1 / ( fuzzyModel.m75.y0 - fuzzyModel.m100.y0) )*output...
         - fuzzyModel.m100.y0 / ( fuzzyModel.m75.y0 - fuzzyModel.m100.y0 );
   end
end

function mfunvalue = evaluate_mi_100( output )
   global fuzzyModel;

   mfunvalue = 0;
   if output > fuzzyModel.m75.y0 && output <= fuzzyModel.m100.y0
      mfunvalue = ( 1 / ( fuzzyModel.m100.y0 - fuzzyModel.m75.y0) )*output...
         - fuzzyModel.m75.y0 / ( fuzzyModel.m100.y0 - fuzzyModel.m75.y0 );
   elseif output > fuzzyModel.m100.y0 && output < fuzzyModel.m125.y0
      mfunvalue = ( 1 / ( fuzzyModel.m100.y0 - fuzzyModel.m125.y0) )*output...
         - fuzzyModel.m125.y0 / ( fuzzyModel.m100.y0 - fuzzyModel.m125.y0 );
   end
end

function mfunvalue = evaluate_mi_125( output )
   global fuzzyModel;

   mfunvalue = 0;
   if output > fuzzyModel.m100.y0 && output <= fuzzyModel.m125.y0
      mfunvalue = ( 1 / ( fuzzyModel.m125.y0 - fuzzyModel.m100.y0) )*output...
         - fuzzyModel.m100.y0 / ( fuzzyModel.m125.y0 - fuzzyModel.m100.y0 );
   elseif output > fuzzyModel.m125.y0 && output < fuzzyModel.m150.y0
      mfunvalue = ( 1 / ( fuzzyModel.m125.y0 - fuzzyModel.m150.y0) )*output...
         - fuzzyModel.m150.y0 / ( fuzzyModel.m125.y0 - fuzzyModel.m150.y0 );
   end
end

function mfunvalue = evaluate_mi_150( output )
   global fuzzyModel;

   mfunvalue = 0;
   if output > fuzzyModel.m125.y0 && output <= fuzzyModel.m150.y0
      mfunvalue = ( 1 / ( fuzzyModel.m150.y0 - fuzzyModel.m125.y0) )*output...
         - fuzzyModel.m125.y0 / ( fuzzyModel.m150.y0 - fuzzyModel.m125.y0 );
   elseif output > fuzzyModel.m150.y0
      mfunvalue = 1;
   end
end
