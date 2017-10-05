package us.ihmc.thor.parameters;

import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;

public class ThorSteppingParameters implements SteppingParameters
{

   @Override
   public double getFootForwardOffset()
   {
      return ThorPhysicalProperties.footForwardForControl;
   }

   @Override
   public double getFootBackwardOffset()
   {
      return ThorPhysicalProperties.footBackForControl;
   }

   @Override
   public double getInPlaceWidth()
   {
      return 0.25;
   }

   @Override
   public double getDesiredStepForward()
   {
      return 0.5;    // 0.35;
   }

   @Override
   public double getMaxStepLength()
   {
      return 0.6;    // 0.5; //0.35;
   }

   @Override
   public double getMinStepWidth()
   {
      return 0.15;
   }

   @Override
   public double getMaxStepWidth()
   {
      return 0.6;    // 0.4;
   }

   @Override
   public double getStepPitch()
   {
      return 0.0;
   }

   @Override
   public double getDefaultStepLength()
   {
      return 0.6;
   }

   @Override
   public double getMaxStepUp()
   {
      return 0.25;
   }

   @Override
   public double getMaxStepDown()
   {
      return 0.2;
   }

   @Override
   public double getMaxSwingHeightFromStanceFoot()
   {
      return 0.30;
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
      return ThorPhysicalProperties.footWidthForControl;
   }

   @Override
   public double getToeWidth()
   {
      return ThorPhysicalProperties.footWidthForControl;
   }

   @Override
   public double getFootLength()
   {
      return ThorPhysicalProperties.footLengthForControl;
   }

   @Override
   public double getActualFootWidth()
   {
      return ThorPhysicalProperties.actualFootWidth;
   }

   @Override
   public double getActualFootLength()
   {
      return ThorPhysicalProperties.actualFootLength;
   }
}
