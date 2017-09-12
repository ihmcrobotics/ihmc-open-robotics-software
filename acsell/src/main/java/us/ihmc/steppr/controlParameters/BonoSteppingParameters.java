package us.ihmc.steppr.controlParameters;

import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.steppr.parameters.BonoPhysicalProperties;

public class BonoSteppingParameters implements SteppingParameters
{
   private final boolean runningOnRealRobot;

   public BonoSteppingParameters(boolean runningOnRealRobot)
   {
      this.runningOnRealRobot = runningOnRealRobot;
   }

   @Override
   public double getFootForwardOffset()
   {
      return BonoPhysicalProperties.footForward;
   }

   @Override
   public double getFootBackwardOffset()
   {
      return BonoPhysicalProperties.footBack;
   }

   @Override
   public double getInPlaceWidth()
   {
      return 0.35;
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
      // TODO The smallest the best in terms of control.
      return 0.35;//0.375;
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
      return BonoPhysicalProperties.footWidth;
   }

   @Override
   public double getToeWidth()
   {
      return BonoPhysicalProperties.toeWidth;
   }

   @Override
   public double getFootLength()
   {
      return BonoPhysicalProperties.footForward + BonoPhysicalProperties.footBack;
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
