function [X3]=compose_point(X1, X2)
  s=sin(X1(3));
  c=cos(X1(3));
  X3=[X1(1)+[c -s]*X2;
      X1(2)+ [s c]*X2];
return;