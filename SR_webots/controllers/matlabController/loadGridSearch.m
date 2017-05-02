load gridSearch1.mat

clf
surf(GS.var1_grid, GS.var2_grid, reshape(GS.distance, size(GS.var1_grid)));
xlabel('total phase lag [rad]')
ylabel('amplitude [rad]')
zlabel('distance [m]')