load('charakterystykiStatyczne.mat');

load('modele/trapezoidFuzzyModel.mat');

args = {
   [ 0 : 0.01 : 75 ];
   [ 50 : 0.01 : 100 ];
   [ 75 : 0.01 : 125 ];
   [ 100 : 0.01 : 150 ];
   [ 125 : 0.01 : 200 ]
};

for j = 1 : length(fuzzyModel)
   outputPast = zeros( 2, 1);
   inputPast = ones( 14, 1 );
   for i = 1 : 2000
      output = sum( fuzzyModel{j}.A .* outputPast ) + sum( fuzzyModel{j}.B .* inputPast );
      outputPast = [ output; outputPast(1) ];
   end
   gains{j} = output;
   bs{j} = fuzzyModel{j}.y0 - gains{j} * fuzzyModel{j}.u0;
   plots{j} =  gains{j}*args{j};
   plots{j} = plots{j} + bs{j};
end

figure(1);
hold on;
grid on;
plot(controlValues,controlOutputSSValues,'b', 'LineWidth', 2);

localControlValues = [ fuzzyModel{1}.u0; fuzzyModel{2}.u0; fuzzyModel{3}.u0; fuzzyModel{4}.u0; fuzzyModel{5}.u0; ];
localOutputValues = [ fuzzyModel{1}.y0; fuzzyModel{2}.y0; fuzzyModel{3}.y0; fuzzyModel{4}.y0; fuzzyModel{5}.y0; ];

addpath('customToolbox/plotting');
colors = distinguishable_colors(7);
rmpath('customToolbox/plotting');


plot(localControlValues, localOutputValues, 'dr', 'MarkerSize',5,'LineWidth',3);
for i = 1 : length(fuzzyModel)
   plot(args{i},plots{i},'Color',colors(i+2,:),'LineWidth',2,'LineStyle','--');
end

% indices = zeros(5,1);
% searchedValues = [ 50; 75; 100; 125; 150 ];
% j = 1;
% for i = 1 : length( controlValues );
% 
%    % display(controlValues(i));
%    % display(searchedValues(j));
%    % pause;
% 
%    if( abs( controlValues(i) - searchedValues(j) ) < 1e-3 )
%       display('YES!');
%       indices(j) = i;
%       j = j + 1;
%       if j > length(searchedValues)
%          break;
%       end
%    end
% end
% 
% gains = zeros(5,1);
% localControlValues = zeros(5, 1);
% localOutputValues = zeros(5, 1);
% for i = 1 : length( searchedValues )
%    localOutputValues(i) = controlOutputSSValues(indices(i));
%    gains(i) = controlOutputSSValues(indices(i)) / ...
%       controlValues(indices(i));
%    localControlValues(i) = controlValues(indices(i));
% end
% 
% 
% 
% 
% figure(1);
% hold on;
% grid on;
% plot(controlValues,controlOutputSSValues,'b');
% plot(localControlValues, localOutputValues, 'or');
% temp = gains(1)*controlValues;
% plot(controlValues, temp, 'g');
% temp = gains(2)*controlValues;
% plot(controlValues, temp, 'g');
% temp = gains(3)*controlValues;
% plot(controlValues, temp, 'g');
% temp = gains(4)*controlValues;
% plot(controlValues, temp, 'g');
% temp = gains(5)*controlValues;
% plot(controlValues, temp, 'g');


