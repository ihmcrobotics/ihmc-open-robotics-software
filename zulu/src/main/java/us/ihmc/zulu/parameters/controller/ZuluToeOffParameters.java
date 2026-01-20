package us.ihmc.zulu.parameters.controller;

import us.ihmc.zulu.parameters.model.ZuluPhysicalProperties;
import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;

public class ZuluToeOffParameters extends ToeOffParameters
{
   private final ZuluPhysicalProperties zuluPhysicalProperties;

   public ZuluToeOffParameters(ZuluPhysicalProperties zuluPhysicalProperties)
   {
      this.zuluPhysicalProperties = zuluPhysicalProperties;
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
      return zuluPhysicalProperties.getFootLengthForControl();
   }

   @Override
   public boolean doToeOffWhenHittingAnkleLimit()
   {
      return true;
   }

   @Override
   public boolean doToeOffWhenHittingTrailingKneeLowerLimit()
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
