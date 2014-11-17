function [ fvalue, FISOutput ] = recalculateFuzzyModel( mfparams, fuzzyModel, outputTrajectory, inputTrajectory )
   global tanks;

   % Wszystkie trzy parametry krzywych dzwonowych.
   index = 1;
   for i = 1 : length( fuzzyModel )
      for j = 1 : length( fuzzyModel{i}.MFParams )
         fuzzyModel{i}.MFParams(j) = mfparams( index );
         index = index + 1;
      end
   end

   % % Dwa parametry krzywych dzwonowych.
   % index = 1;
   % for i = 1 : length( fuzzyModel )
   %    for j = 1 : length( fuzzyModel{i}.MFParams ) - 1
   %       fuzzyModel{i}.MFParams(j) = mfparams( index );
   %       index = index + 1;
   %    end
   % end

   % for i = 1 : length( fuzzyModel )
   %    fuzzyModel{i}.MFParams(3) = mfparams( i );
   % end

   inputPast = tanks.sstate.F1 * ones( length( fuzzyModel{1}.B ), 1 );
   outputPast = tanks.sstate.h2 * ones( length( fuzzyModel{1}.A ), 1 );

   FISOutput = zeros( length( outputTrajectory ), 1 );

   simulationLength = length(inputTrajectory);
   
   % ======== ZMIENNE WYŚWIETLAJĄCE POSTĘP SYMULACJI ================= %
   simulation_step = 0;                                                %
   letters_written = 0;                                                %
   finish_time = simulationLength;                                     %
   fprintf('\n');                                                      %
   % ================================================================= %
   
   output = tanks.sstate.h2;
   
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
   
   
      [ localOutputs, weights ] = evaluateFuzzyModelGPC( fuzzyModel, ...
         output, inputPast, outputPast );
      output = sum( weights .* localOutputs );
   
      inputPast = [ inputTrajectory(i); inputPast( 1 : length(inputPast) - 1 ) ];
      outputPast = [ output; outputPast(1) ];
  
      FISOutput(i) = output;
      
   end

   % figure(1);
   % hold on;
   % plot(outputTrajectory);
   % plot(FISOutput,'r');

   fvalue = sum( ( outputTrajectory - FISOutput ).^2 );
end
