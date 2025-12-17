package us.ihmc.openAlexander.parameters.controller;

import us.ihmc.openAlexander.parameters.model.AlexanderPhysicalProperties;
import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;

public class ZuluToeOffParameters extends ToeOffParameters
{
   private final AlexanderPhysicalProperties alexanderPhysicalProperties;

   public ZuluToeOffParameters(AlexanderPhysicalProperties alexanderPhysicalProperties)
   {
      this.alexanderPhysicalProperties = alexanderPhysicalProperties;
   }

   @Override
   public double getMinStepLengthForToeOff()
   {
      return alexanderPhysicalProperties.getFootLengthForControl();
   }
}
