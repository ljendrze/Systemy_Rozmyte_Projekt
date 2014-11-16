function firingStrength = evaluateTriangleMF( x, MFParams )
   firingStrength = 0;
   if x >= MFParams(1) && x < MFParams(2)
      firingStrength = (1 / ( MFParams(2) - MFParams(1) ) ) * ( x - MFParams(1) );
   elseif x >= MFParams(2) && x < MFParams(3)
      firingStrength = ( -1 / ( MFParams(3) - MFParams(2) ) )*( x - MFParams(3) );
   end
end
