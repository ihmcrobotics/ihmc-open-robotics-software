calibrationData

x = root.loggedmain.DRCEstimatorThread.StepprSensorReader.Steppr.torsoZActuator.SlowSensors.torsoZActuatorIMUMagX';
y = root.loggedmain.DRCEstimatorThread.StepprSensorReader.Steppr.torsoZActuator.SlowSensors.torsoZActuatorIMUMagY';
z = root.loggedmain.DRCEstimatorThread.StepprSensorReader.Steppr.torsoZActuator.SlowSensors.torsoZActuatorIMUMagZ';

% do the fitting
[ center, radii, evecs, v ] = ellipsoid_fit( [x y z ] );
fprintf( 'Ellipsoid center: %.3g %.3g %.3g\n', center );
fprintf( 'Ellipsoid radii : %.3g %.3g %.3g\n', radii );
fprintf( 'Ellipsoid evecs :\n' );
fprintf( '%.3g %.3g %.3g\n%.3g %.3g %.3g\n%.3g %.3g %.3g\n', ...
    evecs(1), evecs(2), evecs(3), evecs(4), evecs(5), evecs(6), evecs(7), evecs(8), evecs(9) );
fprintf( 'Algebraic form  :\n' );
fprintf( '%.3g ', v );
fprintf( '\n' );
