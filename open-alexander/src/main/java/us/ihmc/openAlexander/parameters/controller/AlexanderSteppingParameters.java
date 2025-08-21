package us.ihmc.openAlexander.parameters.controller;

import us.ihmc.openAlexander.parameters.model.AlexanderPhysicalProperties;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;

public class AlexanderSteppingParameters implements SteppingParameters
{
   protected final AlexanderPhysicalProperties alexanderPhysicalProperties;

   public AlexanderSteppingParameters(AlexanderPhysicalProperties alexanderPhysicalProperties)
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
   public double getInPlaceWidth()
   {
      // TODO Needs tune up.
      return 0.22;
   }

   @Override
   public double getMaxStepLength()
   {
      return 0.3;
   }

   @Override
   public double getMinStepWidth()
   {
      return 0.12;
   }

   @Override
   public double getMaxStepWidth()
   {
      return 0.3;
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
