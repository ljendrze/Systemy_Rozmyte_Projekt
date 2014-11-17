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
               'LineStyle', simulations{2,j}.LineStyle, ...
               'LineWidth', 2 );
      elseif strcmp( simulations{2, j}.PlotType, 'stairs')
         stairs( simulations{i,j}, ...
                 'Color', simulations{2,j}.Color,...
                 'LineStyle', simulations{2,j}.LineStyle,...
                 'LineWidth', 2 );
      end
   end

   legend( labels, 'Location', 'Best');
end

figure(2);
hold on;
grid on;
for i = 3 : size( simulations, 1 )
   if strcmp( simulations{2, 1}.PlotType, 'plot')
      plot( simulations{i,1}, ...
            'Color', simulations{2,1}.Color,...
            'LineStyle', simulations{2,1}.LineStyle, ...
            'LineWidth', 2 );
   elseif strcmp( simulations{2, 1}.PlotType, 'stairs')
      stairs( simulations{i,1}, ...
              'Color', simulations{2,1}.Color,...
              'LineStyle', simulations{2,1}.LineStyle,...
              'LineWidth', 2 );
   end
end

% addpath('customToolbox/plotting');
% colors = distinguishable_colors( size(simulations,1) - 2 );
% rmpath('customToolbox/plotting');
% 
% figure(1);
% hold on;
% grid on;
% for i = 3 : size( simulations, 1 )
%    for j = 2 : size( simulations, 2 );
%       if strcmp( simulations{2, j}.PlotType, 'plot')
%          plot( simulations{i,j}, ...
%                'Color', colors(i-2,:),...
%                'LineStyle', simulations{2,j}.LineStyle, ...
%                'LineWidth', 2 );
%       elseif strcmp( simulations{2, j}.PlotType, 'stairs')
%          stairs( simulations{i,j}, ...
%                  'Color', colors(i-2,:),...
%                  'LineStyle', simulations{2,j}.LineStyle,...
%                  'LineWidth', 2 );
%       end
%    end
% 
%    legend( labels, 'Location', 'Best');
% end
