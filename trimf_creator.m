xx = 1 : 0.1 : 100;
yy = zeros( length(xx), 1 );

trimf_params = [ 10, 70, 90 ];

for i = 1 : length(xx)
   if xx(i) < trimf_params(1) || xx(i) > trimf_params(3)
      yy(i) = 0;
   elseif xx(i) >= trimf_params(1) && xx(i) < trimf_params(2)
      yy(i) = (1 / ( trimf_params(2) - trimf_params(1) ) ) * (xx(i)-trimf_params(1));
   elseif xx(i) >= trimf_params(2) && xx(i) < trimf_params(3)
      yy(i) = ( -1 / ( trimf_params(3) - trimf_params(2) ) ) * (xx(i)-trimf_params(3));
   end

end
