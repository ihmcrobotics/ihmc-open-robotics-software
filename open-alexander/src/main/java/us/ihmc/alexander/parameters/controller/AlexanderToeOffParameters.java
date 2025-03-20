package us.ihmc.alexander.parameters.controller;

import us.ihmc.alexander.parameters.model.AlexanderPhysicalProperties;
import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;

public class AlexanderToeOffParameters extends ToeOffParameters
{
   private final AlexanderPhysicalProperties alexanderPhysicalProperties;

   public AlexanderToeOffParameters(AlexanderPhysicalProperties alexanderPhysicalProperties)
   {
      this.alexanderPhysicalProperties = alexanderPhysicalProperties;
   }

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
   public double getMinStepLengthForToeOff()
   {
      return alexanderPhysicalProperties.getFootLengthForControl();
   }

   @Override
   public boolean doToeOffWhenHittingAnkleLimit()
   {
      return true;
   }

   // TODO we should investigate turning this on on hardware
   //   @Override
   //   public boolean doToeOffWhenHittingTrailingKneeLowerLimit()
   //   {
   //      return true;
   //   }

   @Override
   public double getKneeLowerLimitToTriggerToeOff()
   {
      return 0.4;
   }

}
