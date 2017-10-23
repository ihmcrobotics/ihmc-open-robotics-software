package us.ihmc.thor.parameters;

import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;

public class ThorToeOffParameters extends ToeOffParameters
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
      return true;
   }

   @Override
   public double getMinStepLengthForToeOff()
   {
      return ThorPhysicalProperties.footLengthForControl;
   }

   /**
    * To enable that feature, doToeOffIfPossible() return true is required. John parameter
    */
   @Override
   public boolean doToeOffWhenHittingAnkleLimit()
   {
      return true;
   }

   @Override
   public double getMaximumToeOffAngle()
   {
      return Math.toRadians(45.0);
   }

}
