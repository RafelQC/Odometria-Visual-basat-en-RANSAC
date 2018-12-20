function aux_plot_trajectory(X, poseSkip)
  for i=1:poseSkip:size(X,2),
      aux_draw_vehicle(X(1:3, i), 0.1);hold on;
  end;
  plot(X(1,:), X(2,:), 'k');
return;