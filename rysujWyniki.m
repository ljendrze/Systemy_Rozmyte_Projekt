load('wynik_symulacji.mat');

figure(1);
hold on;
for i = 1 : size( simulations, 1 )
   plot(simulations{i,2},'b');
   plot(simulations{i,3},'r');
   plot(simulations{i,4},'g');   
   if i == 1
      legend( 'nieliniowy', 'liniowy', 'rozmyty', 'Location', 'Best');
   end
end
grid on;
