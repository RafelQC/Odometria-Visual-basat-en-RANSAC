function aux_draw_vehicle (X, robotSize)
  vertices = [1.5 -1 -1 1.5
              0    1 -1  0 ]*robotSize/2;
  vertices = compose_point(X(1:3), vertices);
  plot(vertices(1,:), vertices(2,:), 'b');hold on;
  plot(X(1), X(2), 'b.');
end
