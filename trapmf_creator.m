xx = 1 : 0.1 : 100;
yy = zeros( length(xx), 1 );

trapmf_params = [ 10, 70, 80, 90 ];

for i = 1 : length(xx)
   if xx(i) < trapmf_params(1) || xx(i) > trapmf_params(4)
      yy(i) = 0;
   elseif xx(i) >= trapmf_params(1) && xx(i) < trapmf_params(2)
      yy(i) = (1 / ( trapmf_params(2) - trapmf_params(1) ) )*( xx(i) - trapmf_params(1)) ;
   elseif xx(i) >= trapmf_params(2) && xx(i) < trapmf_params(3)
      yy(i) = 1;
   elseif xx(i) >= trapmf_params(3) && xx(i) < trapmf_params(4)
      yy(i) = ( -1 / ( trapmf_params(4) - trapmf_params(3) ) ) * ( xx(i) - trapmf_params(4) );
   end

end

plot(xx,yy);
axis([-5 105 -0.2 1.2]);




% xx = 1 : 0.1 : 100;
% yy = zeros( length(xx), 1 );
% 
% trimf_params = [ 10, 70, 90 ];
% 
% for i = 1 : length(xx)
%    if xx(i) < trimf_params(1) || xx(i) > trimf_params(3)
%       yy(i) = 0;
%    elseif xx(i) >= trimf_params(1) && xx(i) < trimf_params(2)
%       yy(i) = (1 / ( trimf_params(2) - trimf_params(1) ) ) * (xx(i)-trimf_params(1));
%    elseif xx(i) >= trimf_params(2) && xx(i) < trimf_params(3)
%       yy(i) = ( -1 / ( trimf_params(3) - trimf_params(2) ) ) * (xx(i)-trimf_params(3));
%    end
% 
% end
