function firingStrength = evaluateBellMF( x, MFParams )
%             1
%  y = ----------------
%           |x-c|(2*a)
%       1 + |---|
%           | b |
   firingStrength = ...
      ( 1 + abs( (x - MFParams(3)) / MFParams(2) )^(2*MFParams(1)) )^(-1);
end
