package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.odomEKF;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class OdometryKalmanFilterParameters
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   public final YoDouble baseProcessTranslationNoise = new YoDouble("baseProcessTranslationNoise", registry);
   public final YoDouble baseProcessVelocityXYNoise = new YoDouble("baseProcessVelocityXYNoise", registry);
   public final YoDouble baseProcessVelocityZNoise = new YoDouble("baseProcessVelocityZNoise", registry);
   public final YoDouble baseProcessOrientationNoise = new YoDouble("baseProcessOrientationNoise", registry);
   public final YoDouble baseProcessGyroBiasNoise = new YoDouble("baseProcessGyroBiasNoise", registry);
   public final YoDouble baseProcessAccelBiasNoise = new YoDouble("baseProcessAccelBiasNoise", registry);

   public final YoDouble footProcessTranslationNoise = new YoDouble("footProcessTranslationNoise", registry);
   public final YoDouble footProcessVelocityNoise = new YoDouble("footProcessVelocityNoise", registry);
   public final YoDouble footProcessOrientationNoise = new YoDouble("footProcessOrientationNoise", registry);
   public final YoDouble footProcessAccelBiasNoise = new YoDouble("footProcessAccelBiasNoise", registry);
   public final YoDouble footProcessGyroBiasNoise = new YoDouble("footProcessGyroBiasNoise", registry);


   public final YoDouble measurementIKPositionNoise = new YoDouble("measurementIKPositionNoise", registry);
   public final YoDouble measurementIKVelocityNoise = new YoDouble("measurementIKVelocityNoise", registry);
   public final YoDouble measurementIKOrientationNoise = new YoDouble("measurementIKOrientationNoise", registry);
   public final YoDouble measurementContactVelocityNoise = new YoDouble("measurementContactVelocityNoise", registry);
   public final YoDouble measurementContactAccelNoise = new YoDouble("measurementContactAccelNoise", registry);

   public OdometryKalmanFilterParameters(YoRegistry parentRegistry)
   {
      baseProcessTranslationNoise.set(0.0005);
      baseProcessVelocityXYNoise.set(0.005);
      baseProcessVelocityZNoise.set(0.005);
      baseProcessOrientationNoise.set(1e-7);
      baseProcessGyroBiasNoise.set(1e-5);
      baseProcessAccelBiasNoise.set(1e-4);

      footProcessTranslationNoise.set(1e-4);
      footProcessVelocityNoise.set(5.0);
      footProcessOrientationNoise.set(1e-5); // made up.
      footProcessGyroBiasNoise.set(1e-5); // made up
      footProcessAccelBiasNoise.set(1e-4);

      measurementIKPositionNoise.set(0.001);
      measurementIKOrientationNoise.set(1e-8);
      measurementIKVelocityNoise.set(0.05);
      measurementContactVelocityNoise.set(0.01);
      measurementContactAccelNoise.set(1.0);

      parentRegistry.addChild(registry);
   }
}
