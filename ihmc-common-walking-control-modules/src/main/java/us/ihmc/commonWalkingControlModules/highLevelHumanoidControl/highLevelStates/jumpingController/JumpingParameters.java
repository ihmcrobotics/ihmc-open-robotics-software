package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController;

import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoRegistry;

public class JumpingParameters
{
   private static final double defaultSupportDurationValue = 0.75;
   private static final double defaultFlightDurationValue = 0.2;
   private static final double defaultFootWidthValue = 0.2;

   private static final double defaultMinKneeAngleForTakeOff = Math.toRadians(20.0);

   private final DoubleParameter defaultSupportDuration;
   private final DoubleParameter defaultFlightDuration;
   private final DoubleParameter defaultFootWidth;

   private final DoubleParameter minKneeAngleForTakeOff;

   public JumpingParameters(YoRegistry registry)
   {
      defaultSupportDuration = new DoubleParameter("defaultSupportDuration", registry, defaultSupportDurationValue);
      defaultFlightDuration = new DoubleParameter("defaultFlightDuration", registry, defaultFlightDurationValue);
      defaultFootWidth = new DoubleParameter("defaultFootWidth", registry, defaultFootWidthValue);

      minKneeAngleForTakeOff = new DoubleParameter("minKneeAngleForTakeOff", registry, defaultMinKneeAngleForTakeOff);
   }


   public double getDefaultSupportDuration()
   {
      return defaultSupportDuration.getValue();
   }

   public double getDefaultFlightDuration()
   {
      return defaultFlightDuration.getValue();
   }

   public double getDefaultFootWidth()
   {
      return defaultFootWidth.getValue();
   }

   public double getMinKneeAngleForTakeOff()
   {
      return minKneeAngleForTakeOff.getValue();
   }
}
