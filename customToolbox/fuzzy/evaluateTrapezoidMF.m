function firingStrength = evaluateTrapezoidMF( x, MFParams )
   firingStrength = 0;
   if x >= MFParams(1) && x < MFParams(2)
      firingStrength = (1 / ( MFParams(2) - MFParams(1) ) )*( x - MFParams(1)) ;
      return;
   elseif x >= MFParams(2) && x < MFParams(3)
      firingStrength = 1;
      return;
   elseif x >= MFParams(3) && x < MFParams(4)
      firingStrength = ( -1 / ( MFParams(4) - MFParams(3) ) ) * ( x - MFParams(4) );
   end
end
