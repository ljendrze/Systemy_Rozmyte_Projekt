function rysujWykresySymulacyjne( filename )

load(filename);

for i = 2 : size( simulations, 2 )
   labels{i-1} = simulations{2,i}.Legend;
end

figure(1);
hold on;
grid on;
for i = 3 : size( simulations, 1 )
   for j = 2 : size( simulations, 2 );
      if strcmp( simulations{2, j}.PlotType, 'plot')
         plot( simulations{i,j}, ...
               'Color', simulations{2,j}.Color,...
               'LineStyle', simulations{2,j}.LineStyle );
      elseif strcmp( simulations{2, j}.PlotType, 'stairs')
         stairs( simulations{i,j}, ...
                 'Color', simulations{2,j}.Color,...
                 'LineStyle', simulations{2,j}.LineStyle );
      end
   end

   legend( labels, 'Location', 'Best');
end
