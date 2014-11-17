function [ MFuns ] = extractMFuns( fuzzyModel, args )

   pointsNo = length(args);
   rulesNo = length(fuzzyModel);
   MFuns = zeros( pointsNo, rulesNo );

   % ======== ZMIENNE WYŚWIETLAJĄCE POSTĘP SYMULACJI ================= %
   simulation_step = 0;                                                %
   letters_written = 0;                                                %
   finish_time = pointsNo;                                             %
   fprintf('\n');                                                      %
   % ================================================================= %

   % Przetwarzanie wszystkich argumentów.
   for i = 1 : pointsNo
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

      % Przetwarzanie wszystkich modeli lokalnych.
      for j = 1 : length( fuzzyModel )
         if( strcmp( fuzzyModel{j}.MFType, 'trimf' ) == 1)
            MFuns(i,j) = evaluateTriangleMF( args(i), fuzzyModel{j}.MFParams );
         elseif( strcmp( fuzzyModel{j}.MFType, 'trapmf' ) == 1)
            MFuns(i,j) = evaluateTrapezoidMF( args(i), fuzzyModel{j}.MFParams );
         elseif( strcmp( fuzzyModel{j}.MFType, 'gbellmf' ) == 1)
            MFuns(i,j) = evaluateGBellMF( args(i), fuzzyModel{j}.MFParams );
         end
      end
   end

   fprintf('\n\n');
end
