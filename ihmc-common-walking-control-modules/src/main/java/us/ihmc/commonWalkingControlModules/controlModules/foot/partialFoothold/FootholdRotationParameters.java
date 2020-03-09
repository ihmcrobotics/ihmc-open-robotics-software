package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class FootholdRotationParameters
{
   private final YoDouble geometricDetectionAngleThreshold;
   private final static double defaultGeometricDetectionAngleThreshold = Math.toRadians(10.0);

   private final YoDouble angularVelocityAroundLoRThreshold;
   private final YoDouble footDropThreshold;
   private final YoDouble angularVelocityFilterBreakFrequency;
   private final YoDouble stableLoRAngularVelocityThreshold;
   private final YoDouble stableCoRLinearVelocityThreshold;

   private static final double defaultStableLoRAngularVelocityThreshold = 10.0;
   private static final double defaultStableCoRLinearVelocityThreshold = 0.1;
   private static final double defaultAngularVelocityAroundLoRThreshold = 0.5;
   private static final double defaultFootDropThreshold = -0.04;
   private static final double defaultAngularVelocityFilterBreakFrequency = 16.0;

   public FootholdRotationParameters(YoVariableRegistry registry)
   {
      String namePrefix = "ExplorationGeometric_";
      geometricDetectionAngleThreshold = new YoDouble(namePrefix + "AngleThreshold", registry);
      geometricDetectionAngleThreshold.set(defaultGeometricDetectionAngleThreshold);

      namePrefix = "ExplorationVelocity_";
      stableLoRAngularVelocityThreshold = new YoDouble(namePrefix + "StableLoRAngularVelocityThreshold", registry);
      stableLoRAngularVelocityThreshold.set(defaultStableLoRAngularVelocityThreshold);
      stableCoRLinearVelocityThreshold = new YoDouble(namePrefix + "StableCoRLinearVelocityThreshold", registry);
      stableCoRLinearVelocityThreshold.set(defaultStableCoRLinearVelocityThreshold);
      angularVelocityAroundLoRThreshold = new YoDouble(namePrefix + "AngularVelocityAroundLoRThreshold", registry);
      angularVelocityAroundLoRThreshold.set(defaultAngularVelocityAroundLoRThreshold);
      footDropThreshold = new YoDouble(namePrefix + "FootDropThreshold", registry);
      footDropThreshold.set(defaultFootDropThreshold);
      angularVelocityFilterBreakFrequency = new YoDouble(namePrefix + "AngularVelocityFilterBreakFrequency", registry);
      angularVelocityFilterBreakFrequency.set(defaultAngularVelocityFilterBreakFrequency);
   }

   public YoDouble getGeometricDetectionAngleThreshold()
   {
      return geometricDetectionAngleThreshold;
   }

   public YoDouble getAngularVelocityAroundLoRThreshold()
   {
      return angularVelocityAroundLoRThreshold;
   }

   public YoDouble getFootDropThreshold()
   {
      return footDropThreshold;
   }

   public YoDouble getAngularVelocityFilterBreakFrequency()
   {
      return angularVelocityFilterBreakFrequency;
   }

   public YoDouble getStableLoRAngularVelocityThreshold()
   {
      return stableLoRAngularVelocityThreshold;
   }

   public YoDouble getStableCoRLinearVelocityThreshold()
   {
      return stableCoRLinearVelocityThreshold;
   }
}
