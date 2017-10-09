package us.ihmc.escher.parameters;

import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;

public class EscherToeOffParameters extends ToeOffParameters
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
      return EscherPhysicalProperties.footLengthForControl;
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
