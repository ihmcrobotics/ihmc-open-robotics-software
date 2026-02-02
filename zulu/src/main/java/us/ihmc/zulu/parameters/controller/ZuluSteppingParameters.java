package us.ihmc.zulu.parameters.controller;

import us.ihmc.zulu.parameters.model.ZuluPhysicalProperties;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;

public class ZuluSteppingParameters implements SteppingParameters
{
   protected final ZuluPhysicalProperties zuluPhysicalProperties;

   public ZuluSteppingParameters(ZuluPhysicalProperties zuluPhysicalProperties)
   {
      this.zuluPhysicalProperties = zuluPhysicalProperties;
   }

   @Override
   public double getFootForwardOffset()
   {
      return zuluPhysicalProperties.getFootForwardForControl();
   }

   @Override
   public double getFootBackwardOffset()
   {
      return zuluPhysicalProperties.getFootBackForControl();
   }

   @Override
   public double getInPlaceWidth()
   {
      // TODO Needs tune up.
      return 0.22;
   }

   @Override
   public double getMaxStepLength()
   {
      return 0.7;
   }

   @Override
   public double getMinStepWidth()
   {
      return 0.12;
   }

   @Override
   public double getMaxStepWidth()
   {
      return 0.8;
   }

   @Override
   public double getDefaultStepLength()
   {
      // TODO Needs tune up.
      return 0.4;
   }

   @Override
   public double getMaxStepUp()
   {
      // TODO Needs tune up.
      return 0.25;
   }

   @Override
   public double getMaxStepDown()
   {
      // TODO Needs tune up.
      return 0.2;
   }

   @Override
   public double getMaxAngleTurnOutwards()
   {
      return 0.65;
   }

   @Override
   public double getMaxAngleTurnInwards()
   {
      // TODO Needs tune up.
      return 0.0;
   }

   @Override
   public double getFootWidth()
   {
      return zuluPhysicalProperties.getFootWidthForControl();
   }

   @Override
   public double getToeWidth()
   {
      return zuluPhysicalProperties.getToeWidthForControl();
   }

   @Override
   public double getFootLength()
   {
      return zuluPhysicalProperties.getFootLengthForControl();
   }

   @Override
   public double getActualFootWidth()
   {
      return zuluPhysicalProperties.getActualFootWidth();
   }

   @Override
   public double getActualFootLength()
   {
      return zuluPhysicalProperties.getActualFootLength();
   }
}
