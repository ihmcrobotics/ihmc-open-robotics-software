package us.ihmc.atlas.parameters;

import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;

public class AtlasSteppingParameters implements SteppingParameters
{
   protected final AtlasJointMap jointMap;

   public AtlasSteppingParameters(AtlasJointMap jointMap)
   {
      this.jointMap = jointMap;
   }

   @Override
   public double getFootForwardOffset()
   {
      return jointMap.getPhysicalProperties().getFootForwardForControl();
   }

   @Override
   public double getFootBackwardOffset()
   {
      return jointMap.getPhysicalProperties().getFootBackForControl();
   }

   @Override
   public double getInPlaceWidth()
   {
      return 0.25 * jointMap.getModelScale();
   }


   @Override
   public double getMaxStepLength()
   {
      return 0.6 * jointMap.getModelScale(); // 0.5; //0.35;
   }

   @Override
   public double getMinStepWidth()
   {
      return 0.15 * jointMap.getModelScale();
   }

   @Override
   public double getMaxStepWidth()
   {
      return 0.6 * jointMap.getModelScale(); // 0.4;
   }

   @Override
   public double getDefaultStepLength()
   {
      return 0.6 * jointMap.getModelScale();
   }

   @Override
   public double getMaxStepUp()
   {
      return 0.25 * jointMap.getModelScale();
   }

   @Override
   public double getMaxStepDown()
   {
      return 0.2 * jointMap.getModelScale();
   }

   @Override
   public double getMaxAngleTurnOutwards()
   {
      //increased atlas turn speed defaults
      // return Math.PI / 4.0;
      return 0.6;
   }

   @Override
   public double getMaxAngleTurnInwards()
   {
      //increased atlas turn speed defaults
      //  return 0;
      return -0.1;
   }

   @Override
   public double getTurningStepWidth()
   {
      return 0.25;
   }

   @Override
   public double getFootWidth()
   {
      return jointMap.getPhysicalProperties().getFootWidthForControl();
   }

   @Override
   public double getToeWidth()
   {
      return jointMap.getPhysicalProperties().getToeWidthForControl();
   }

   @Override
   public double getFootLength()
   {
      return jointMap.getPhysicalProperties().getFootLengthForControl();
   }

   @Override
   public double getActualFootWidth()
   {
      return jointMap.getPhysicalProperties().getActualFootWidth();
   }

   @Override
   public double getActualFootLength()
   {
      return jointMap.getPhysicalProperties().getActualFootLength();
   }
}
