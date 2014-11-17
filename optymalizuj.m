clear;
clc;

addpath('customToolbox/fuzzy');

global tanks;

load( 'tanksParameters.mat' );
load( 'trajektorie.mat' );

load( 'modele/gbellFuzzyModel.mat' );
fuzzyModelStarting = fuzzyModel;

outputTrajectory = trajectories{2};
inputTrajectory = trajectories{1};

% Wszystkie trzy parametry krzywych dzwonowych.
x0 = [];
for i = 1 : length( fuzzyModel )
   for j = 1 : length( fuzzyModel{i}.MFParams )
      x0 = [ x0; fuzzyModel{i}.MFParams(j) ];
   end
end
LB = [ ...
   min( 0.8*x0(1), 1.2*x0(1) ); min( 0.8*x0(2), 1.2*x0(2) ); min( 0.9*x0(3), 1.1*x0(3) ); ...
   min( 0.8*x0(4), 1.2*x0(4) ); min( 0.8*x0(5), 1.2*x0(5) ); min( 0.9*x0(6), 1.1*x0(6) ); ...
   min( 0.8*x0(7), 1.2*x0(7) ); min( 0.8*x0(8), 1.2*x0(8) ); min( 0.9*x0(9), 1.1*x0(9) ); ...
   min( 0.8*x0(10), 1.2*x0(10) ); min( 0.8*x0(11), 1.2*x0(11) ); min( 0.9*x0(12), 1.1*x0(12) ); ...
   min( 0.8*x0(13), 1.2*x0(13) ); min( 0.8*x0(14), 1.2*x0(14) ); min( 0.9*x0(15), 1.1*x0(15) ); ...
];

UB = [ ...
   max( 0.8*x0(1), 1.2*x0(1) ); max( 0.8*x0(2), 1.2*x0(2) ); max( 0.9*x0(3), 1.1*x0(3) ); ...
   max( 0.8*x0(4), 1.2*x0(4) ); max( 0.8*x0(5), 1.2*x0(5) ); max( 0.9*x0(6), 1.1*x0(6) ); ...
   max( 0.8*x0(7), 1.2*x0(7) ); max( 0.8*x0(8), 1.2*x0(8) ); max( 0.9*x0(9), 1.1*x0(9) ); ...
   max( 0.8*x0(10), 1.2*x0(10) ); max( 0.8*x0(11), 1.2*x0(11) ); max( 0.9*x0(12), 1.1*x0(12) ); ...
   max( 0.8*x0(13), 1.2*x0(13) ); max( 0.8*x0(14), 1.2*x0(14) ); max( 0.9*x0(15), 1.1*x0(15) ); ...
];

% x0 = [];
% for i = 1 : length( fuzzyModelStarting )
%    x0 = [ x0; fuzzyModelStarting{i}.MFParams(3) ];
% end

% LB = [ -Inf; 25; 45; 75; 120 ];

% UB = [ 10; 45; 65; 95; Inf ];

opts = optimset('Display', 'iter-detailed', 'Algorithm', 'interior-point', 'MaxIter', 10 );

MODEL_ROZMYTY = struct( ...
   'objective', @(x)recalculateFuzzyModel( x, ...
      fuzzyModelStarting, outputTrajectory, inputTrajectory ), ...
   'x0', x0, ...
   'Aineq', [], ...
   'bineq', [], ...
   'Aeq', [], ...
   'beq', [], ...
   'lb', LB, ...
   'ub', UB, ...
   'nonlcon', [], ...
   'options', opts, ...
   'solver', 'fmincon' ...
);

fuzzyModelOptimized = fuzzyModelStarting;
X = fmincon( MODEL_ROZMYTY );

index = 1;
for i = 1 : length( fuzzyModelOptimized )
   for j = 1 : length( fuzzyModelOptimized{i}.MFParams )
      fuzzyModelOptimized{i}.MFParams(j) = X(index);
      index = index + 1;
   end
end

% for i = 1 : length( fuzzyModelOptimized )
%    fuzzyModelOptimized{i}.MFParams(3) = X(i);
% end

fuzzyModel = fuzzyModelOptimized;
save( 'modele/fuzzyModelOptimized.mat', 'fuzzyModel' );

args = [ 0 : 0.01 : 200 ]';

MFunsOriginal = extractMFuns( fuzzyModelStarting, args );
MFunsOptimized = extractMFuns( fuzzyModelOptimized, args );

figure(1);
hold on;
for i = 1 : size( MFunsOriginal, 2 )
   plot( MFunsOriginal(:,i) );
end

for i = 1 : size( MFunsOptimized, 2 )
   plot( MFunsOptimized(:,i),'r' );
end

display('Ploting fuzzy models outputs, be patient.');
[ fvalueStarting, FISOutputStarting ] = recalculateFuzzyModel( x0, ...
      fuzzyModelStarting, outputTrajectory, inputTrajectory );
[ fvalueOptimized, FISOutputOptimized ] = recalculateFuzzyModel( X, ...
      fuzzyModelOptimized, outputTrajectory, inputTrajectory );

display( fvalueOptimized - fvalueStarting );

figure(2);
hold on;
plot(outputTrajectory,'b', 'LineWidth', 2);
plot(FISOutputStarting,'--r', 'LineWidth', 2);
plot(FISOutputOptimized,'--g', 'LineWidth', 2);

rmpath('customToolbox/fuzzy');
