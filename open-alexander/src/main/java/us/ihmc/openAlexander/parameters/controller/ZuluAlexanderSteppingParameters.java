package us.ihmc.openAlexander.parameters.controller;

import us.ihmc.openAlexander.parameters.model.AlexanderPhysicalProperties;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;

public class ZuluAlexanderSteppingParameters implements SteppingParameters
{
   protected final AlexanderPhysicalProperties alexanderPhysicalProperties;

   public ZuluAlexanderSteppingParameters(AlexanderPhysicalProperties alexanderPhysicalProperties)
   {
      this.alexanderPhysicalProperties = alexanderPhysicalProperties;
   }

   @Override
   public double getFootForwardOffset()
   {
      return alexanderPhysicalProperties.getFootForwardForControl();
   }

   @Override
   public double getFootBackwardOffset()
   {
      return alexanderPhysicalProperties.getFootBackForControl();
   }

   @Override
   public double getFootWidth()
   {
      return alexanderPhysicalProperties.getFootWidthForControl();
   }

   @Override
   public double getToeWidth()
   {
      return alexanderPhysicalProperties.getToeWidthForControl();
   }

   @Override
   public double getFootLength()
   {
      return alexanderPhysicalProperties.getFootLengthForControl();
   }

   @Override
   public double getActualFootWidth()
   {
      return alexanderPhysicalProperties.getActualFootWidth();
   }

   @Override
   public double getActualFootLength()
   {
      return alexanderPhysicalProperties.getActualFootLength();
   }
}
