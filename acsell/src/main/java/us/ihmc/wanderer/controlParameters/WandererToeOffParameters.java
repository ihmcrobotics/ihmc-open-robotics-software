package us.ihmc.wanderer.controlParameters;

import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;

public class WandererToeOffParameters extends ToeOffParameters
{
   @Override
   public boolean doToeOffIfPossible()
   {
      return true;
   }

   @Override
   public boolean doToeOffIfPossibleInSingleSupport()
   {
      return false;
   }

   @Override
   public boolean checkECMPLocationToTriggerToeOff()
   {
      return false;
   }

   @Override
   public double getMinStepLengthForToeOff()
   {
      return 0.20;
   }

   /**
    * To enable that feature, doToeOffIfPossible() return true is required.
    */
   @Override
   public boolean doToeOffWhenHittingAnkleLimit()
   {
      return false;
   }

   @Override
   public double getMaximumToeOffAngle()
   {
      return Math.toRadians(45.0);
   }

}
