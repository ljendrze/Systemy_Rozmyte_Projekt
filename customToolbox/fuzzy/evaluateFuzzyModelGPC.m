function [ localOutputs, weights, nn_weights ] = evaluateFuzzyModelGPC( fuzzyModel, x, inputPast, outputPast )
   % Model rozmyty powinien być macierzą komórkową struktur, w której będą
   % przetwarzane kolejne modele lokalne. Modele lokalne mają postać 
   % znormalizowanej odpowiedzi skokowej.
   %
   % Argumenty wejściowe funkcji:
   %    - fuzzyModel - macierz komórkowa składająca się ze struktur modeli
   %                   lokalnych i ich funkcji przynależności budujących
   %                   pełny model rozmyty.
   %    - x - element względem którego obliczane będą wagi modelu rozmytego.
   %    - inputPast - wektor poprzednich wartości wejścia obiektu
   %    - outputPast - wektor poprzednich wartości wyjścia obiektu
   %
   % Wartości zwracane przez funkcję:
   %   - output - wartość wyjścia obliczona z modelu;
   %   - weights - znormalizowane siły odpalenia reguł lokalnych;
   %   - nn_weights - oryginalne wartości funkcji przynależności, 
   %                  sprzed normalizacji;
   %
   % Struktura modelu lokalnego powinna zawierać pola:
   %    - b - wektor wartości wpływu poprzednich wyjść;
   %    - c - wektor wartości wpływu poprzednich wejść;
   %    - u0 - wartość sterowania z punktu pracy modelu lokalnego;
   %    - y0 - wartość wyjścia z punktu pracy modelu lokalnego;
   %    - x0 - wektor zmiennych stanu odpowiadających punktowi pracy
   %           modelu lokalnego;
   %    - MFType - typ funkcji przynależności, obsługiwane są wartości:
   %               'trapmf', 'trimf', 'gbellmf';
   %    - MFParams - parametry opisujące funkcję przynależności
   %
   % Model lokalny dany jest równaniem różnicowym:
   %    y(k) = b(1)*y(k-1) + b(2)*y(k-2) + c(1)*u(k-1) + c(2)*u(k-2)

   weights = zeros( size(fuzzyModel) );
   nn_weights = zeros( size(fuzzyModel) );
   localOutputs = zeros( length(fuzzyModel), 1);
   
   % Przetwarzanie wszystkich modeli lokalnych
   for i = 1 : length( fuzzyModel )
      if( strcmp( fuzzyModel{i}.MFType, 'trimf' ) == 1)
         weights(i) = evaluateTriangleMF( x, fuzzyModel{i}.MFParams );
      elseif( strcmp( fuzzyModel{i}.MFType, 'trapmf' ) == 1)
         weights(i) = evaluateTrapezoidMF( x, fuzzyModel{i}.MFParams );
      elseif( strcmp( fuzzyModel{i}.MFType, 'gbellmf' ) == 1)
         weights(i) = evaluateBellMF( x, fuzzyModel{i}.MFParams );
      end
   end
   nn_weights = weights;
   weights = weights / sum(weights);

   for i = 1 : length( fuzzyModel )
      localOutputs(i) = ...
         sum( fuzzyModel{i}.A .* ( outputPast - fuzzyModel{i}.y0 ) ) + ...
         sum( fuzzyModel{i}.B .* ( inputPast - fuzzyModel{i}.u0 ) ) + ...
         fuzzyModel{i}.y0;
   end

end
