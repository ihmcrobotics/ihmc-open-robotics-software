package us.ihmc.wanderer.controlParameters;

import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.wanderer.parameters.WandererPhysicalProperties;

public class WandererSteppingParameters implements SteppingParameters
{
   private final boolean runningOnRealRobot;

   public WandererSteppingParameters(boolean runningOnRealRobot)
   {
      this.runningOnRealRobot = runningOnRealRobot;
   }

   @Override
   public double getFootForwardOffset()
   {
      return WandererPhysicalProperties.footForward;
   }

   @Override
   public double getFootBackwardOffset()
   {
      return WandererPhysicalProperties.footBack;
   }

   @Override
   public double getInPlaceWidth()
   {
      return 0.16; //This is about the minimum width for the feet to not touch.
   }

   @Override
   public double getDesiredStepForward()
   {
      return 0.3; //0.5; //0.35;
   }

   @Override
   public double getMaxStepLength()
   {
      return runningOnRealRobot ? 0.5 : 0.4;
   }

   @Override
   public double getDefaultStepLength()
   {
      return 0.4;
   }

   @Override
   public double getMinStepWidth()
   {
      return 0.22;//0.35;
   }

   @Override
   public double getMaxStepWidth()
   {
      return 0.5; //0.5; //0.4;
   }

   @Override
   public double getStepPitch()
   {
      return 0.0;
   }

   @Override
   public double getMaxStepUp()
   {
      return 0.1;
   }

   @Override
   public double getMaxStepDown()
   {
      return 0.1;
   }

   @Override
   public double getMaxSwingHeightFromStanceFoot()
   {
      return 0.25;
   }

   @Override
   public double getMaxAngleTurnOutwards()
   {
      return Math.PI / 4.0;
   }

   @Override
   public double getMaxAngleTurnInwards()
   {
      return 0;
   }

   @Override
   public double getMinAreaPercentForValidFootstep()
   {
      return 0.5;
   }

   @Override
   public double getDangerAreaPercentForValidFootstep()
   {
      return 0.75;
   }

   @Override
   public double getFootWidth()
   {
      return WandererPhysicalProperties.footWidth;
   }

   @Override
   public double getToeWidth()
   {
      return WandererPhysicalProperties.toeWidth;
   }

   @Override
   public double getFootLength()
   {
      return WandererPhysicalProperties.footForward + WandererPhysicalProperties.footBack;
   }

   @Override
   public double getActualFootWidth()
   {
      return getFootWidth();
   }

   @Override
   public double getActualFootLength()
   {
      return getFootLength();
   }
}
